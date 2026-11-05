"""Exhaustive rigid registration of two occupancy grids (rotation sweep + FFT shift search).

This is what the pseudo ground truth between runs comes from. It is too slow and
too dependent on large overlap to be the method under study, but with near-complete
maps of the same area it is a reliable reference.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Rigid:
    theta: float
    t: np.ndarray

    @property
    def R(self):
        c, s = np.cos(self.theta), np.sin(self.theta)
        return np.array([[c, -s], [s, c]])

    def apply(self, xy):
        return np.asarray(xy) @ self.R.T + self.t


def _layers(grid):
    free = (grid == 0).astype(np.float32)
    wall = (grid > 50).astype(np.uint8)
    wall = cv2.GaussianBlur(wall.astype(np.float32), (5, 5), 1.0)
    return free, wall


def _correlate(fa, fb_conj_source, shape):
    return np.fft.irfft2(np.fft.rfft2(fa, shape) * np.conj(np.fft.rfft2(fb_conj_source, shape)), shape)


def register(grid_a, origin_a, grid_b, origin_b, resolution, angles=None):
    """Return (T, score, scores_by_angle) with T mapping B's frame into A's frame."""
    if angles is None:
        angles = np.deg2rad(np.arange(0, 360, 1.0))
    free_a, wall_a = _layers(grid_a)
    free_b, wall_b = _layers(grid_b)

    h, w = grid_b.shape
    diag = int(np.ceil(np.hypot(h, w))) + 2
    centre_b = np.array([w / 2, h / 2])
    centre_out = np.array([diag / 2, diag / 2])
    shape = (grid_a.shape[0] + diag, grid_a.shape[1] + diag)
    fa_free, fa_wall = np.fft.rfft2(free_a, shape), np.fft.rfft2(wall_a, shape)

    best = (-np.inf, None, None)
    by_angle = []
    for theta in angles:
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        M = np.column_stack([R, centre_out - R @ centre_b])
        fr = cv2.warpAffine(free_b, M, (diag, diag), flags=cv2.INTER_LINEAR)
        wr = cv2.warpAffine(wall_b, M, (diag, diag), flags=cv2.INTER_LINEAR)
        fb_free, fb_wall = np.conj(np.fft.rfft2(fr, shape)), np.conj(np.fft.rfft2(wr, shape))
        # Reward free-on-free and wall-on-wall; a wall on free space is a contradiction.
        score = np.fft.irfft2(fa_free * fb_free + 3.0 * fa_wall * fb_wall
                              - 3.0 * (fa_free * fb_wall + fa_wall * fb_free), shape)
        index = np.unravel_index(np.argmax(score), score.shape)
        by_angle.append(score[index])
        if score[index] > best[0]:
            shift = np.array([index[1], index[0]], dtype=float)
            shift[0] -= shape[1] if shift[0] > shape[1] / 2 else 0
            shift[1] -= shape[0] if shift[1] > shape[0] / 2 else 0
            t = origin_a - R @ origin_b + resolution * (shift + centre_out - R @ centre_b)
            best = (score[index], theta, t)
    return Rigid(best[1], best[2]), best[0], np.array(by_angle)


def register_refined(grid_a, origin_a, grid_b, origin_b, resolution):
    coarse, _, by_angle = register(grid_a, origin_a, grid_b, origin_b, resolution)
    fine_angles = coarse.theta + np.deg2rad(np.arange(-1.5, 1.5001, 0.25))
    fine, score, _ = register(grid_a, origin_a, grid_b, origin_b, resolution, fine_angles)
    # Distinctiveness: best score from an orientation more than 15 degrees away.
    angles = np.arange(len(by_angle)) * 2 * np.pi / len(by_angle)
    gap = np.abs((angles - coarse.theta + np.pi) % (2 * np.pi) - np.pi)
    rival = by_angle[gap > np.deg2rad(15)].max()
    return fine, score, rival
