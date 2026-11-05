"""Seeded graph matching (FAQ), a spectral baseline, rigid-transform recovery, and map_merge's AKAZE baseline."""

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from registration import Rigid


def geodesic_kernel(graph, sigma=1.0):
    """Centred Gaussian kernel of geodesic distance, so padding with zeros is neutral."""
    K = np.exp(-graph.geodesic ** 2 / (2 * sigma ** 2))
    return K - K.mean()


def centred_adjacency(graph):
    A = 2 * graph.adjacency - 1
    np.fill_diagonal(A, 0)
    return A


def clearance_similarity(graph_a, graph_b, tau=0.15):
    diff = graph_a.clearance[:, None] - graph_b.clearance[None, :]
    return np.exp(-diff ** 2 / (2 * tau ** 2))


def _pad(M, n):
    out = np.zeros((n, n))
    out[:M.shape[0], :M.shape[1]] = M
    return out


def faq(A, B, seeds=(), C=None, weight=0.0, restarts=8, iterations=30, rng=None):
    """Frank-Wolfe on max trace(A Q B Q^T) + weight trace(C^T Q), seeds held fixed.

    Yields one candidate per restart: (matches, objective), where matches is an
    array of (a, b) pairs with padded dummy nodes dropped.
    """
    rng = np.random.default_rng() if rng is None else rng
    n_a, n_b = A.shape[0], B.shape[0]
    n = max(n_a, n_b)
    A, B = _pad(A, n), _pad(B, n)
    C = np.zeros((n, n)) if C is None else _pad(C, n)

    seed_a = [a for a, _ in seeds]
    seed_b = [b for _, b in seeds]
    free_a = np.array([i for i in range(n) if i not in set(seed_a)], dtype=int)
    free_b = np.array([j for j in range(n) if j not in set(seed_b)], dtype=int)
    s = len(seeds)
    m = n - s
    A22, B22 = A[np.ix_(free_a, free_a)], B[np.ix_(free_b, free_b)]
    # Cross term between seeded and unseeded nodes, plus the node-similarity term.
    linear = weight * C[np.ix_(free_a, free_b)]
    if s:
        linear = linear + 2 * A[np.ix_(free_a, seed_a)] @ B[np.ix_(free_b, seed_b)].T

    candidates = []
    for restart in range(restarts):
        P = np.full((m, m), 1.0 / m)
        if restart:
            noise = np.eye(m)[rng.permutation(m)]
            P = 0.5 * P + 0.5 * noise
        for _ in range(iterations):
            gradient = 2 * A22 @ P @ B22 + linear
            rows, cols = linear_sum_assignment(-gradient)
            Q = np.zeros((m, m))
            Q[rows, cols] = 1
            D = Q - P
            quad = np.sum((A22 @ D @ B22) * D)
            slope = np.sum(gradient * D)
            if slope <= 1e-9:
                break
            step = min(1.0, -slope / (2 * quad)) if quad < 0 else 1.0
            P = P + step * D
        rows, cols = linear_sum_assignment(-P)
        Q = np.zeros((m, m))
        Q[rows, cols] = 1
        objective = np.sum((A22 @ Q @ B22) * Q) + np.sum(linear * Q)
        pairs = list(seeds) + [(free_a[r], free_b[c]) for r, c in zip(rows, cols)]
        matches = np.array([(a, b) for a, b in pairs if a < n_a and b < n_b], dtype=int)
        candidates.append((matches, objective))
    return candidates


def spectral_match(A, B, seeds, dimension=6):
    """Embed each kernel, align on the seeds with orthogonal Procrustes, assign nearest."""
    def embed(M):
        values, vectors = np.linalg.eigh(M)
        order = np.argsort(-np.abs(values))[:dimension]
        return vectors[:, order] * np.sqrt(np.abs(values[order]))
    Xa, Xb = embed(A), embed(B)
    seed_a, seed_b = zip(*seeds)
    U, _, Vt = np.linalg.svd(Xa[list(seed_a)].T @ Xb[list(seed_b)])
    Xb = Xb @ (U @ Vt).T
    cost = ((Xa[:, None] - Xb[None, :]) ** 2).sum(-1)
    rows, cols = linear_sum_assignment(cost)
    return np.column_stack([rows, cols])


def fit_rigid(src, dst):
    """Least-squares rotation and translation taking src to dst (Kabsch)."""
    sc, dc = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return Rigid(np.arctan2(R[1, 0], R[0, 0]), dc - R @ sc)


def ransac_rigid(xy_a, xy_b, matches, threshold=0.35, trials=400, rng=None):
    """Robust transform from noisy node matches. Returns (T, inlier count) or (None, 0)."""
    rng = np.random.default_rng() if rng is None else rng
    if len(matches) < 3:
        return None, 0
    a, b = xy_a[matches[:, 0]], xy_b[matches[:, 1]]
    best, best_inliers = None, 0
    for _ in range(trials):
        i, j = rng.choice(len(matches), 2, replace=False)
        span = np.linalg.norm(b[i] - b[j])
        if span < 0.5 or abs(span - np.linalg.norm(a[i] - a[j])) > threshold:
            continue
        T = fit_rigid(b[[i, j]], a[[i, j]])
        inliers = np.linalg.norm(T.apply(b) - a, axis=1) < threshold
        if inliers.sum() > best_inliers:
            best, best_inliers = inliers, inliers.sum()
    if best is None or best_inliers < 3:
        return None, 0
    return fit_rigid(b[best], a[best]), int(best_inliers)


def map_agreement(grid_a, origin_a, grid_b, origin_b, resolution, T):
    """Blind check of a transform: do B's cells agree with A where they land? Uses no ground truth.

    Returns (score, conflict rate among cells known in both maps, number of such cells).
    """
    rc = np.argwhere(grid_b >= 0)
    labels = grid_b[rc[:, 0], rc[:, 1]]
    world = T.apply(origin_b + (rc[:, ::-1] + 0.5) * resolution)
    cells = np.floor((world - origin_a) / resolution).astype(int)
    inside = (cells[:, 0] >= 0) & (cells[:, 0] < grid_a.shape[1]) \
        & (cells[:, 1] >= 0) & (cells[:, 1] < grid_a.shape[0])
    landed = grid_a[cells[inside, 1], cells[inside, 0]]
    labels = labels[inside]
    both_free = np.sum((landed == 0) & (labels == 0))
    both_wall = np.sum((landed > 50) & (labels > 50))
    conflict = np.sum((landed > 50) & (labels == 0)) + np.sum((landed == 0) & (labels > 50))
    known = both_free + both_wall + conflict
    score = float(both_free + 3 * both_wall - 3 * conflict)
    return score, float(conflict / known) if known else 1.0, int(known)


def seeds_only(xy_a, xy_b, seeds):
    """Control: the transform the seeds alone give, with no matching at all."""
    pairs = np.array(seeds, dtype=int)
    return fit_rigid(xy_b[pairs[:, 1]], xy_a[pairs[:, 0]])


def akaze_seeds(grid_a, origin_a, xy_a, grid_b, origin_b, xy_b, resolution, count=5, radius=0.3):
    """Realistic seeds: AKAZE's RANSAC inliers, gates ignored, snapped to the nearest graph nodes."""
    finder = cv2.AKAZE_create()
    features = [cv2.detail.computeImageFeatures2(finder, _as_image(g)) for g in (grid_a, grid_b)]
    if min(len(f.getKeypoints()) for f in features) < 2:
        return []
    try:
        pairwise = cv2.detail_AffineBestOf2NearestMatcher(False, False, 0.3, 6).apply2(features)
    except cv2.error:
        return []
    match = next((m for m in pairwise if m.src_img_idx == 1 and m.dst_img_idx == 0), None)
    if match is None or match.H is None or match.H.size == 0:
        return []
    keys_a, keys_b = features[0].getKeypoints(), features[1].getKeypoints()
    mask = np.asarray(match.getInliers()).ravel()
    seeds, used_a, used_b = [], set(), set()
    # queryIdx indexes the source image (B), trainIdx the destination (A).
    for m, keep in sorted(zip(match.getMatches(), mask), key=lambda x: x[0].distance):
        if not keep:
            continue
        pa = origin_a + (np.array(keys_a[m.trainIdx].pt) + 0.5) * resolution
        pb = origin_b + (np.array(keys_b[m.queryIdx].pt) + 0.5) * resolution
        da, db = np.linalg.norm(xy_a - pa, axis=1), np.linalg.norm(xy_b - pb, axis=1)
        a, b = int(da.argmin()), int(db.argmin())
        if da[a] < radius and db[b] < radius and a not in used_a and b not in used_b:
            seeds.append((a, b))
            used_a.add(a)
            used_b.add(b)
            if len(seeds) == count:
                break
    return seeds


def _as_image(grid):
    # map_merge wraps the int8 grid bytes as CV_8UC1, so unknown (-1) becomes 255.
    return grid.astype(np.int8).view(np.uint8).copy()


def akaze_baseline(grid_a, origin_a, grid_b, origin_b, resolution, confidence=0.65):
    """map_merge's estimator on two grids, with its gates, without the bundle-adjust polish.

    Returns (T or None, info). T maps B's frame into A's frame.
    """
    finder = cv2.AKAZE_create()
    features = [cv2.detail.computeImageFeatures2(finder, _as_image(g)) for g in (grid_a, grid_b)]
    info = {'inliers': 0, 'matches': 0, 'confidence': 0.0}
    if min(len(f.getKeypoints()) for f in features) < 2:
        return None, info
    matcher = cv2.detail_AffineBestOf2NearestMatcher(False, False, 0.3, 6)
    try:
        pairwise = matcher.apply2(features)
    except cv2.error:
        return None, info
    # Find the entry that takes B's pixels (index 1) into A's pixels (index 0).
    match = next((m for m in pairwise if m.src_img_idx == 1 and m.dst_img_idx == 0), None)
    if match is None or match.H is None or match.H.size == 0:
        return None, info
    info = {'inliers': int(match.num_inliers), 'matches': len(match.getMatches()),
            'confidence': float(match.confidence)}
    ratio = info['inliers'] / info['matches'] if info['matches'] else 0.0
    if info['inliers'] < 8 or ratio < 0.5 or info['confidence'] < confidence:
        return None, info
    H = np.asarray(match.H, dtype=float)
    # Replace the partial-affine scale with a pure rotation; maps are metric.
    theta = np.arctan2(H[1, 0], H[0, 0])
    R = Rigid(theta, np.zeros(2)).R
    # x_a = R x_b + t in world coordinates, from p_a = R p_b + h in pixels.
    t = origin_a + resolution * H[:2, 2] - R @ origin_b
    return Rigid(theta, t), info
