"""Map overhead-camera pixels on the table plane to robot base-frame millimetres."""
import json
from pathlib import Path

import numpy as np


def fit(pixels, base_mm):
    """Least-squares homography from >=4 pixel/base pairs; returns (H, per-point error in mm)."""
    import cv2
    pixels, base_mm = np.asarray(pixels, np.float64), np.asarray(base_mm, np.float64)
    if len(pixels) < 4 or len(pixels) != len(base_mm):
        raise ValueError("need at least 4 matching pixel/base points")
    # Plain least squares, not RANSAC: every point is a deliberate pendant touch,
    # so an outlier is a typo the operator should see and fix, not silently drop.
    H, _ = cv2.findHomography(pixels, base_mm, 0)
    if H is None:
        raise ValueError("points are degenerate (collinear?); spread them across the table")
    errors = np.linalg.norm(apply(H, pixels) - base_mm, axis=1)
    return H, errors


def apply(H, pixels):
    """Map an (N, 2) array (or one (x, y)) of pixels to base-frame mm."""
    points = np.atleast_2d(np.asarray(pixels, np.float64))
    mapped = np.c_[points, np.ones(len(points))] @ np.asarray(H).T
    result = mapped[:, :2] / mapped[:, 2:]
    return result[0] if np.ndim(pixels) == 1 else result


def load(path: Path):
    return np.asarray(json.loads(Path(path).read_text(encoding="utf-8"))["homography"])
