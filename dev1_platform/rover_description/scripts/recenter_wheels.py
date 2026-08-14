#!/usr/bin/env python3
"""Re-center wheel STLs so each mesh's bounding-box center sits at (0,0,0).
Prints the offset that was baked in (in mm) — useful for updating URDF joint origins."""
import struct
import numpy as np
import sys
from pathlib import Path


def read_binary_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80)
        (n_tri,) = struct.unpack('<I', f.read(4))
        # 50 bytes per triangle: 12 normal + 36 verts + 2 attr
        dtype = np.dtype([
            ('normal', '<f4', 3),
            ('v1', '<f4', 3),
            ('v2', '<f4', 3),
            ('v3', '<f4', 3),
            ('attr', '<u2'),
        ])
        tris = np.fromfile(f, dtype=dtype, count=n_tri)
    return header, tris


def write_binary_stl(path, header, tris):
    with open(path, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(tris)))
        tris.tofile(f)


def recenter(path):
    header, tris = read_binary_stl(path)
    verts = np.concatenate([tris['v1'], tris['v2'], tris['v3']], axis=0)
    bb_min = verts.min(axis=0)
    bb_max = verts.max(axis=0)
    center = (bb_min + bb_max) / 2.0
    # translate vertices so center -> origin
    for k in ('v1', 'v2', 'v3'):
        tris[k] -= center
    write_binary_stl(path, header, tris)
    return center, bb_max - bb_min


if __name__ == '__main__':
    meshes = Path('rover_description/meshes')
    for name in ['wheel_fr.stl', 'wheel_fl.stl', 'wheel_br.stl', 'wheel_bl.stl']:
        p = meshes / name
        c, size = recenter(p)
        print(f'{name}: baked center (mm) = ({c[0]:+8.3f}, {c[1]:+8.3f}, {c[2]:+8.3f})  size = {tuple(size)}')
