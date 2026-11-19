#!/usr/bin/env python3
"""Save the reference map that every run's coverage is measured against.

    source scripts/env/central_env.bash
    python3 entc641/reference.py REF [--robot inky]

Takes the last map the robot published in data/runs/REF/bag and writes
data/reference.npz (grid, origin, resolution) and data/reference.png. Check the
picture: the whole area with the door open should be mapped and closed off.
"""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import bagdata  # noqa: E402
import design  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_id', nargs='?', default='REF')
    ap.add_argument('--robot', default=None, help='robot whose map to use (default: the first)')
    args = ap.parse_args()

    run = bagdata.load_run(design.RUNS / args.run_id)
    robot = args.robot or run.robots[0]
    if not run.maps.get(robot):
        raise SystemExit(f'no /{robot}/map in {args.run_id}')
    snap = run.maps[robot][-1]
    known = snap.grid >= 0
    area = known.sum() * snap.res ** 2
    np.savez_compressed(design.REFERENCE_NPZ, grid=snap.grid, origin=np.array(snap.origin),
                        res=snap.res, source=f'{args.run_id}/{robot}')

    img = np.full(snap.grid.shape, 0.6)
    img[snap.grid == 0] = 1.0
    img[snap.grid > 50] = 0.0
    fig, ax = plt.subplots(figsize=(7, 7))
    h, w = snap.grid.shape
    ox, oy = snap.origin
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=(ox, ox + w * snap.res, oy, oy + h * snap.res))
    ax.set_title(f'Reference map ({args.run_id}, {robot}): {area:.1f} m² known')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(design.DATA / 'reference.png', dpi=120)
    print(f'wrote {design.REFERENCE_NPZ}: {w} x {h} cells at {snap.res} m, {area:.1f} m² known')
    print(f'look at {design.DATA / "reference.png"} before using it')


if __name__ == '__main__':
    main()
