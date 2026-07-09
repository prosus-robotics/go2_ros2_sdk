#!/usr/bin/env python3
# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Flatten a 3D point cloud (.ply) into a 2D Nav2 occupancy map (.pgm + .yaml).

The Go2's SLAM/3D scan (e.g. Open3D `3d_map.ply`) is a metric point cloud, not
something Nav2's map_server can load. This projects it to a 2D grid the same way
a horizontal LiDAR slice would see it: keep points in a height band above the
floor (walls / furniture -> obstacles), drop floor and ceiling, and bin the rest
into an occupancy grid.

    python3 ply_to_map.py 3d_map.ply --out-dir . --name office_real
    python3 ply_to_map.py 3d_map.ply --resolution 0.05 --obs-min 0.2 --obs-max 1.8

Then feed it to the sim:
    SIM_MAP=/ros2_ws/src/office_real.yaml SIM_START_X=0 SIM_START_Y=0 \
      docker compose -f docker/docker-compose.yml up sim

Assumes the cloud is Z-up and in meters (true for Open3D SLAM output). If the
result looks empty/wrong, try --up-axis y, or widen the height band.
"""

import argparse
import os
import sys

import numpy as np

_PLY_TYPES = {
    'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
    'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
    'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
    'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8',
}


def read_ply_xyz(path: str) -> np.ndarray:
    """Return an (N, 3) float array of vertex xyz from a binary/ascii PLY."""
    with open(path, 'rb') as f:
        raw = f.read()
    marker = b'end_header\n'
    end = raw.find(marker)
    if end < 0:
        raise ValueError('not a PLY (no end_header)')
    header = raw[:end].decode('ascii', errors='replace').splitlines()
    body = raw[end + len(marker):]

    fmt = 'ascii'
    props = []          # list of (name, ply_type) for the vertex element
    n_vertex = 0
    in_vertex = False
    for line in header:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'format':
            fmt = parts[1]
        elif parts[0] == 'element':
            in_vertex = (parts[1] == 'vertex')
            if in_vertex:
                n_vertex = int(parts[2])
        elif parts[0] == 'property' and in_vertex:
            # 'property <type> <name>' (list properties not supported for vertices)
            props.append((parts[-1], parts[-2]))

    names = [p[0] for p in props]
    for axis in ('x', 'y', 'z'):
        if axis not in names:
            raise ValueError(f'vertex has no {axis} property')

    if fmt == 'ascii':
        rows = body.decode('ascii').split('\n')
        vals = [r.split() for r in rows if r.strip()][:n_vertex]
        arr = np.array(vals, dtype=np.float64)
        idx = [names.index(a) for a in ('x', 'y', 'z')]
        return arr[:, idx]

    endian = '<' if 'little' in fmt else '>'
    dtype = np.dtype([(n, endian + _PLY_TYPES[t]) for n, t in props])
    verts = np.frombuffer(body, dtype=dtype, count=n_vertex)
    return np.column_stack([verts['x'], verts['y'], verts['z']]).astype(np.float64)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ply', help='input point cloud (.ply)')
    ap.add_argument('--out-dir', default='.', help='where to write <name>.pgm/.yaml')
    ap.add_argument('--name', default='real_map', help='output basename')
    ap.add_argument('--resolution', type=float, default=0.05, help='m per pixel')
    ap.add_argument('--up-axis', choices=['x', 'y', 'z'], default='z')
    ap.add_argument('--obs-min', type=float, default=0.15,
                    help='min height above floor to count as obstacle (m)')
    ap.add_argument('--obs-max', type=float, default=2.0,
                    help='max height above floor to count as obstacle (m)')
    ap.add_argument('--floor-pct', type=float, default=1.0,
                    help='percentile of height treated as the floor level')
    ap.add_argument('--min-points', type=int, default=3,
                    help='points per cell to mark it occupied')
    ap.add_argument('--margin', type=float, default=0.5, help='free border (m)')
    args = ap.parse_args(argv)

    pts = read_ply_xyz(args.ply)
    # Reorder so column 2 is "up".
    order = {'z': (0, 1, 2), 'y': (0, 2, 1), 'x': (1, 2, 0)}[args.up_axis]
    px, py, pz = pts[:, order[0]], pts[:, order[1]], pts[:, order[2]]

    floor = np.percentile(pz, args.floor_pct)
    obs = (pz > floor + args.obs_min) & (pz < floor + args.obs_max)
    print(f"points={len(pts)} floor_z={floor:.2f} "
          f"obstacle_band=[{floor + args.obs_min:.2f},{floor + args.obs_max:.2f}] "
          f"obstacle_points={int(obs.sum())}")
    if obs.sum() == 0:
        print("ERROR: no points in the obstacle band. Try --up-axis or widen "
              "--obs-min/--obs-max.", file=sys.stderr)
        return 2

    min_x, max_x = px.min() - args.margin, px.max() + args.margin
    min_y, max_y = py.min() - args.margin, py.max() + args.margin
    res = args.resolution
    w = int(np.ceil((max_x - min_x) / res))
    h = int(np.ceil((max_y - min_y) / res))

    cols = np.clip(((px[obs] - min_x) / res).astype(np.int32), 0, w - 1)
    rows = np.clip(((py[obs] - min_y) / res).astype(np.int32), 0, h - 1)
    counts = np.zeros((h, w), dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)
    occupied = counts >= args.min_points

    # 254 = free (white), 0 = occupied (black). Row 0 of the PGM is the TOP
    # (max y), so flip vertically relative to our bottom-origin binning.
    img = np.full((h, w), 254, dtype=np.uint8)
    img[occupied] = 0
    img = np.flipud(img)

    os.makedirs(args.out_dir, exist_ok=True)
    pgm_path = os.path.join(args.out_dir, args.name + '.pgm')
    yaml_path = os.path.join(args.out_dir, args.name + '.yaml')
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode('ascii'))
        f.write(img.tobytes())

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: {args.name}.pgm\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{min_x:.4f}, {min_y:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )

    print(f"wrote {pgm_path} ({w}x{h} px, {int(occupied.sum())} occupied cells)")
    print(f"wrote {yaml_path}")
    print(f"origin=({min_x:.2f},{min_y:.2f}) -> world (0,0) is "
          f"{'inside' if (min_x < 0 < max_x and min_y < 0 < max_y) else 'OUTSIDE'} "
          f"the map; pick SIM_START_X/Y in free space accordingly.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
