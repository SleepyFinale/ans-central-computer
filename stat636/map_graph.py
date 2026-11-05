"""Turn an occupancy grid into a topological graph of its free space.

Pipeline: clean the free-space mask, thin it to a skeleton, prune short spurs,
then walk the skeleton into a graph with nodes at junctions, dead ends and at
regular spacing along the branches.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, shortest_path

EIGHT = np.ones((3, 3), dtype=bool)
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class MapGraph:
    xy: np.ndarray         # node positions in the map frame (m), shape (n, 2)
    clearance: np.ndarray  # distance from node to nearest wall (m)
    lengths: np.ndarray    # symmetric edge lengths (m), 0 = no edge

    @property
    def n(self):
        return len(self.xy)

    @property
    def adjacency(self):
        return (self.lengths > 0).astype(float)

    @property
    def geodesic(self):
        return shortest_path(csr_matrix(self.lengths), directed=False)


def clean_free_space(grid, resolution, speckle_area=0.08):
    """Free-space mask with small obstacle specks removed; largest region only."""
    free = grid == 0
    # Specks of occupied or unknown cells inside free space are mostly noise.
    labels, count = ndimage.label(~free, structure=EIGHT)
    if count:
        sizes = ndimage.sum(~free, labels, np.arange(1, count + 1)) * resolution ** 2
        small = np.flatnonzero(sizes < speckle_area) + 1
        free = free | np.isin(labels, small)
    labels, count = ndimage.label(free)
    if not count:
        return free
    sizes = ndimage.sum(free, labels, np.arange(1, count + 1))
    return labels == (np.argmax(sizes) + 1)


def thin(mask):
    """Zhang-Suen thinning."""
    img = np.pad(mask.astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            p2, p3, p4 = img[:-2, 1:-1], img[:-2, 2:], img[1:-1, 2:]
            p5, p6, p7 = img[2:, 2:], img[2:, 1:-1], img[2:, :-2]
            p8, p9 = img[1:-1, :-2], img[:-2, :-2]
            ring = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            count = sum(ring[:8])
            transitions = sum(((a == 0) & (b == 1)).astype(np.uint8)
                              for a, b in zip(ring[:-1], ring[1:]))
            if step == 0:
                c1, c2 = p2 * p4 * p6, p4 * p6 * p8
            else:
                c1, c2 = p2 * p4 * p8, p2 * p6 * p8
            remove = (img[1:-1, 1:-1] == 1) & (count >= 2) & (count <= 6) \
                & (transitions == 1) & (c1 == 0) & (c2 == 0)
            if remove.any():
                img[1:-1, 1:-1][remove] = 0
                changed = True
        if not changed:
            return img[1:-1, 1:-1].astype(bool)


def _pixel_neighbours(mask, r, c):
    h, w = mask.shape
    return [(r + dr, c + dc) for dr, dc in NEIGHBOURS
            if 0 <= r + dr < h and 0 <= c + dc < w and mask[r + dr, c + dc]]


def _trace_branches(skeleton):
    """Split a skeleton into key-point clusters and the pixel chains between them."""
    degree = ndimage.convolve(skeleton.astype(int), np.ones((3, 3), int), mode='constant') - 1
    key = skeleton & (degree != 2)
    # A loop with no junction still needs one key point to start from.
    loops, count = ndimage.label(skeleton, structure=EIGHT)
    for label in range(1, count + 1):
        component = loops == label
        if not (key & component).any():
            key[tuple(np.argwhere(component)[0])] = True
    clusters, n_clusters = ndimage.label(key, structure=EIGHT)

    branches = []
    visited = np.zeros_like(skeleton)
    for r, c in np.argwhere(key):
        start = clusters[r, c]
        for nr, nc in _pixel_neighbours(skeleton, r, c):
            if key[nr, nc]:
                if clusters[nr, nc] != start and (r, c) < (nr, nc):
                    branches.append((start, clusters[nr, nc], [(r, c), (nr, nc)]))
                continue
            if visited[nr, nc]:
                continue
            chain = [(r, c), (nr, nc)]
            visited[nr, nc] = True
            end = start
            while True:
                pr, pc = chain[-1]
                steps = _pixel_neighbours(skeleton, pr, pc)
                keys = [p for p in steps if key[p] and p != chain[-2]]
                if keys and len(chain) > 2 or (keys and clusters[keys[0]] != start):
                    chain.append(keys[0])
                    end = clusters[keys[0]]
                    break
                onward = [p for p in steps if not key[p] and not visited[p]]
                if not onward:
                    break
                visited[onward[0]] = True
                chain.append(onward[0])
            branches.append((start, end, chain))
    return clusters, n_clusters, branches


def _polyline_length(chain, resolution):
    points = np.array(chain, dtype=float)
    return np.linalg.norm(np.diff(points, axis=0), axis=1).sum() * resolution


def prune_spurs(skeleton, resolution, min_length=0.4):
    """Remove dead-end branches shorter than min_length, repeatedly."""
    skeleton = skeleton.copy()
    for _ in range(20):
        degree = ndimage.convolve(skeleton.astype(int), np.ones((3, 3), int), mode='constant') - 1
        ends = skeleton & (degree == 1)
        _, _, branches = _trace_branches(skeleton)
        removed = False
        for _, _, chain in branches:
            if _polyline_length(chain, resolution) >= min_length:
                continue
            first, last = chain[0], chain[-1]
            if ends[first] == ends[last]:
                continue  # keep isolated segments and through-branches
            for pixel in chain:
                if degree[pixel] <= 2:
                    skeleton[pixel] = False
            removed = True
        if not removed:
            break
        skeleton = thin(skeleton)
    return skeleton


def build_graph(grid, resolution, origin, spacing=0.5, speckle_area=0.08, spur_length=0.4):
    """Graph of the largest free region. origin is the map-frame position of cell (0, 0)."""
    free = clean_free_space(grid, resolution, speckle_area)
    distance = ndimage.distance_transform_edt(free) * resolution
    skeleton = prune_spurs(thin(free), resolution, spur_length)
    clusters, n_clusters, branches = _trace_branches(skeleton)

    # Cluster index -> representative pixel (the member nearest the centroid).
    pixels = []
    for label in range(1, n_clusters + 1):
        members = np.argwhere(clusters == label)
        pixels.append(tuple(members[np.argmin(((members - members.mean(0)) ** 2).sum(1))]))

    edges = []
    for start, end, chain in branches:
        if start == end and len(chain) < 4:
            continue
        points = np.array(chain, dtype=float)
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1) * resolution
        total = steps.sum()
        pieces = max(1, int(round(total / spacing)))
        along = np.concatenate([[0], np.cumsum(steps)])
        previous = start - 1
        for k in range(1, pieces + 1):
            if k == pieces and end != start:
                current = end - 1
            elif k == pieces:
                current = start - 1
            else:
                index = np.searchsorted(along, total * k / pieces)
                pixels.append(tuple(np.array(chain[min(index, len(chain) - 1)])))
                current = len(pixels) - 1
            if previous != current:
                edges.append((previous, current, total / pieces))
            previous = current

    n = len(pixels)
    lengths = np.zeros((n, n))
    for i, j, weight in edges:
        lengths[i, j] = lengths[j, i] = weight if lengths[i, j] == 0 else min(lengths[i, j], weight)

    _, labels = connected_components(csr_matrix(lengths), directed=False)
    keep = np.flatnonzero(labels == np.bincount(labels).argmax()) if n else np.array([], int)
    cells = np.array(pixels, dtype=float).reshape(-1, 2)[keep]
    xy = np.asarray(origin) + (cells[:, ::-1] + 0.5) * resolution
    clearance = distance[cells[:, 0].astype(int), cells[:, 1].astype(int)]
    return MapGraph(xy=xy, clearance=clearance, lengths=lengths[np.ix_(keep, keep)])
