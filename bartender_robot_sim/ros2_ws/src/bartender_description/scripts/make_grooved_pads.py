#!/usr/bin/env python3
"""Generate grooved fingertip collision meshes from robotiq_description's.

Run this to regenerate bartender_description/meshes/*_finger_tip_grooved.stl
after changing the groove geometry in render_bartender_urdf.py; the results
are committed, so a normal build does not need it.

Why a modified mesh rather than the primitive boxes render_bartender_urdf.py
can also build: the stock fingertip mesh is only 224 triangles, and exactly 12
of them form the pad face that touches anything. Replacing the WHOLE fingertip
with boxes was measured to cost the whiskey bottle its placement accuracy --
it came back 12-14mm off station instead of 1-4mm, and sometimes fell -- even
with no groove at all and with the pad face's true 22x38mm extent reproduced.
Whatever the remaining 212 triangles contribute, a box does not. So this keeps
all of them byte-identical and rewrites only the 12 pad triangles into a V.

Usage: make_grooved_pads.py [output_dir]
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_bartender_urdf import (  # noqa: E402
    GROOVE_CENTRE_Z, GROOVE_DEPTH, GROOVE_HALF_HEIGHT, TIPS, stl_bbox, stl_tris,
    real_pad_face)

SRC = '/opt/ros/humble/share/robotiq_description/meshes/collision'


def quad(x_a, z_a, x_b, z_b, y0, y1, want_nx):
    """Two triangles spanning y0..y1, from (x_a, z_a) to (x_b, z_b)."""
    p = [(x_a, y0, z_a), (x_a, y1, z_a), (x_b, y1, z_b), (x_b, y0, z_b)]
    out = []
    for tri in ((p[0], p[1], p[2]), (p[0], p[2], p[3])):
        u = [tri[1][k] - tri[0][k] for k in range(3)]
        w = [tri[2][k] - tri[0][k] for k in range(3)]
        nx = u[1] * w[2] - u[2] * w[1]
        # Keep the face pointing the same way the stock pad did.
        out.append(tri if nx * want_nx >= 0 else (tri[0], tri[2], tri[1]))
    return out


def grooved(tris):
    lo, hi = stl_bbox(tris)
    inner = lo[0] if abs(lo[0]) > abs(hi[0]) else hi[0]
    out = 1.0 if (hi[0] if inner == lo[0] else lo[0]) > inner else -1.0
    (y0, y1), (z0, z1) = real_pad_face(tris, inner)

    root = inner + out * GROOVE_DEPTH
    lip = GROOVE_CENTRE_Z - GROOVE_HALF_HEIGHT
    wing = GROOVE_CENTRE_Z + GROOVE_HALF_HEIGHT
    want_nx = -out                      # pad face looks toward the bottle

    kept = [t for t in tris
            if not (max(v[0] for v in t) - min(v[0] for v in t) <= 1e-6
                    and abs(min(v[0] for v in t) - inner) <= 1e-5)]
    if len(tris) - len(kept) != 12:
        raise SystemExit(f'expected 12 pad triangles, found {len(tris)-len(kept)}')

    face = []
    if lip - z0 > 2e-4:                 # flat pad below the groove
        face += quad(inner, z0, inner, lip, y0, y1, want_nx)
    face += quad(inner, lip, root, GROOVE_CENTRE_Z, y0, y1, want_nx)
    face += quad(root, GROOVE_CENTRE_Z, inner, wing, y0, y1, want_nx)
    if z1 - wing > 2e-4:                # flat pad above the groove
        face += quad(inner, wing, inner, z1, y0, y1, want_nx)
    return kept + face


def write_stl(path, tris, header):
    with open(path, 'wb') as f:
        f.write(header.encode()[:80].ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for tri in tris:
            u = [tri[1][k] - tri[0][k] for k in range(3)]
            w = [tri[2][k] - tri[0][k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1],
                 u[2] * w[0] - u[0] * w[2],
                 u[0] * w[1] - u[1] * w[0]]
            m = math.sqrt(sum(c * c for c in n)) or 1.0
            f.write(struct.pack('<fff', *[c / m for c in n]))
            for v in tri:
                f.write(struct.pack('<fff', *v))
            f.write(struct.pack('<H', 0))


def main():
    dest = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')
    dest = os.path.normpath(dest)
    os.makedirs(dest, exist_ok=True)
    for tip in TIPS:
        side = 'left' if 'left' in tip else 'right'
        tris = stl_tris(f'{SRC}/{side}_finger_tip.stl')
        new = grooved(tris)
        path = os.path.join(dest, f'{side}_finger_tip_grooved.stl')
        write_stl(path, new, f'{side} 2F-85 fingertip, V-groove pad')
        print(f'{path}: {len(tris)} -> {len(new)} triangles')
    return 0


if __name__ == '__main__':
    sys.exit(main())
