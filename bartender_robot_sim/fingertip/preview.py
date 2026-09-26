#!/usr/bin/env python3
"""Render exported STLs to PNG, purely so the shape can be eyeballed without
opening a CAD package. Not part of the model; needs requirements-dev.txt.

Usage: preview.py build/fingertip_left.stl [more.stl ...]
"""
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402


def read_stl(path):
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        raise SystemExit(f"{path}: ASCII STL not handled")
    n = struct.unpack("<I", data[80:84])[0]
    tris = np.empty((n, 3, 3), dtype=np.float32)
    for i in range(n):
        base = 84 + i * 50 + 12
        tris[i] = np.frombuffer(data[base:base + 36], dtype="<f4").reshape(3, 3)
    return tris


def main(argv):
    paths = argv or ["build/fingertip_left.stl"]
    views = [(22, -60, "iso"), (90, -90, "top"), (0, -90, "front")]
    fig = plt.figure(figsize=(5 * len(views), 5 * len(paths)))
    for r, path in enumerate(paths):
        tris = read_stl(path)
        lo = tris.reshape(-1, 3).min(axis=0)
        hi = tris.reshape(-1, 3).max(axis=0)
        span = (hi - lo).max() / 2.0
        mid = (hi + lo) / 2.0
        for c, (elev, azim, name) in enumerate(views):
            ax = fig.add_subplot(len(paths), len(views),
                                 r * len(views) + c + 1, projection="3d")
            ax.add_collection3d(Poly3DCollection(
                tris, facecolor="#8fb3d9", edgecolor="#22406b", linewidth=0.25))
            for setlim, m in ((ax.set_xlim, 0), (ax.set_ylim, 1), (ax.set_zlim, 2)):
                setlim(mid[m] - span, mid[m] + span)
            ax.view_init(elev=elev, azim=azim)
            ax.set_box_aspect((1, 1, 1))
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(f"{Path(path).stem}  {name}", fontsize=9)
    out = Path("build/preview.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1:])
