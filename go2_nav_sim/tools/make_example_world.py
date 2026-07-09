#!/usr/bin/env python3
# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Generate a small 20x20 m test world (PGM + YAML) for go2_nav_sim.

The layout matches the coordinates in scenario_runner.py:
  * 20x20 m free area with a 0.1 m border wall,
  * a vertical divider at x=10 m with a 1.2 m doorway centred at y=9 m,
  * a 1x1 m box obstacle centred at (5, 15) m.

Run once (on the ROS2/Linux box, or anywhere with numpy):
    python3 tools/make_example_world.py [output_dir]
Default output_dir is ../worlds relative to this script.
"""

import os
import sys

import numpy as np

RES = 0.05          # m per pixel
SIZE_M = 20.0
N = int(round(SIZE_M / RES))   # 400 px
FREE, OCC = 254, 0


def m2px(m: float) -> int:
    return int(round(m / RES))


def main() -> None:
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'worlds')
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # grid[row=y, col=x], y increasing "up" (map_server origin is bottom-left).
    grid = np.full((N, N), FREE, dtype=np.uint8)

    t = m2px(0.1)
    grid[:t, :] = OCC
    grid[-t:, :] = OCC
    grid[:, :t] = OCC
    grid[:, -t:] = OCC

    # Vertical divider at x=10 with a 1.2 m doorway centred at y=9.
    xw0, xw1 = m2px(9.9), m2px(10.1)
    grid[:, xw0:xw1] = OCC
    grid[m2px(8.4):m2px(9.6), xw0:xw1] = FREE

    # Box obstacle 1x1 m centred at (5, 15).
    grid[m2px(14.5):m2px(15.5), m2px(4.5):m2px(5.5)] = OCC

    # PGM is written top row first; flip so image top = highest y.
    img = np.flipud(grid)

    pgm_path = os.path.join(out_dir, 'example.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(bytearray(f'P5\n{N} {N}\n255\n', 'ascii'))
        f.write(img.tobytes())

    yaml_path = os.path.join(out_dir, 'example.yaml')
    with open(yaml_path, 'w') as f:
        f.write(
            'image: example.pgm\n'
            f'resolution: {RES}\n'
            'origin: [0.0, 0.0, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
            'mode: trinary\n'
        )

    print(f'Wrote {pgm_path}')
    print(f'Wrote {yaml_path}  ({N}x{N} @ {RES} m/px)')


if __name__ == '__main__':
    main()
