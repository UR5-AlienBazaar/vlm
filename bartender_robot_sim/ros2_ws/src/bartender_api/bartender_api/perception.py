"""Classify each bottle as observed, missing or unknown from stand-camera depth."""
import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np
import yaml

from bartender_open import layout as L


class Profile(NamedTuple):
    shape: str        # 'circle' or 'square'
    size: float       # radius, or half-width across the flats
    slab: tuple       # (lo, hi) above the counter where `size` holds


# Straight body sections from each model's collision profile, above the 26mm stand wall.
PROFILES = {
    'whiskey': Profile('square', 0.0386, (0.050, 0.140)),
    'cola': Profile('circle', 0.0336, (0.090, 0.140)),
    'beer': Profile('circle', L.BEER_BODY_RADIUS, (0.040, 0.120)),
}

# Holds the whiskey's corners (0.0546) plus slop, clear of a neighbour's face (0.095).
SEARCH_R = 0.07

# ponytail: calibration knobs; TOL_MM is a placeholder until perception_error.py measures p99.
TOL_MM = 15.0
MAX_FRAME_AGE_S = 1.0
MIN_FOREGROUND_FRACTION = 0.3
# Missing needs nearly every ray through the volume valid, behind it, and unblocked.
MISSING_MIN_VALID_FRACTION = 0.9
MISSING_MIN_SEE_THROUGH_FRACTION = 0.95
MISSING_MAX_OCCLUDED_FRACTION = 0.02
FIT_ITERATIONS = 30


class Calibration(NamedTuple):
    frame_id: str
    T_world_optical: np.ndarray
    depth_range: tuple


class Frame(NamedTuple):
    depth: np.ndarray     # metres, NaN where invalid
    K: tuple              # fx, fy, cx, cy
    frame_id: str
    age_s: float


def load_calibration(path):
    """Read config/stand_camera.yaml; refuse a transform that is not rigid."""
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    T = np.array(doc['T_world_optical'], dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f'{path}: T_world_optical must be 4x4')
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-4) or \
            not np.isclose(np.linalg.det(R), 1.0, atol=1e-4):
        raise ValueError(f'{path}: T_world_optical rotation is not a rotation')
    near, far = doc['depth_range']
    return Calibration(doc['optical_frame_id'], T, (float(near), float(far)))


def decode_depth(encoding, data, height, width, step, is_bigendian):
    """Decode 32FC1 (m) or 16UC1 (mm) depth to metres with NaN invalid, else None."""
    if encoding == '32FC1':
        dtype, scale = np.dtype('>f4' if is_bigendian else '<f4'), 1.0
    elif encoding == '16UC1':
        dtype, scale = np.dtype('>u2' if is_bigendian else '<u2'), 0.001
    else:
        return None
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    depth = rows[:, :width * dtype.itemsize].copy().view(dtype)
    depth = depth.astype(np.float64) * scale
    depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
    return depth


def intrinsics(k, d):
    """Return (fx, fy, cx, cy), or None for distorted depth the pinhole model would mis-measure."""
    if any(abs(c) > 1e-9 for c in d) or k[0] <= 0.0 or k[4] <= 0.0:
        return None
    return (k[0], k[4], k[2], k[5])


def _to_optical(T, xyz):
    R, t = T[:3, :3], T[:3, 3]
    return (np.asarray(xyz, dtype=float) - t) @ R


def station_box(K, T, centre_xy, radius, z_range, shape):
    """Return ((u0, u1, v0, v1), None) bounding a station's volume in the image, or (None, why)."""
    fx, fy, cx, cy = K
    ex, ey = centre_xy
    corners = _to_optical(T, [(ex + sx * radius, ey + sy * radius, z)
                              for sx in (-1, 1) for sy in (-1, 1)
                              for z in z_range])
    if np.any(corners[:, 2] <= 0.0):
        return None, 'station is behind the camera'
    us = fx * corners[:, 0] / corners[:, 2] + cx
    vs = fy * corners[:, 1] / corners[:, 2] + cy
    h, w = shape
    u0, u1 = max(int(us.min()), 0), min(int(math.ceil(us.max())) + 1, w)
    v0, v1 = max(int(vs.min()), 0), min(int(math.ceil(vs.max())) + 1, h)
    if u0 >= u1 or v0 >= v1:
        return None, 'station is out of view'
    return (u0, u1, v0, v1), None


def _volume_interval(T, K, box, centre_xy, slab):
    """Return per-pixel optical depths where each ray enters and leaves the search volume."""
    fx, fy, cx, cy = K
    u0, u1, v0, v1 = box
    uu, vv = np.meshgrid(np.arange(u0, u1), np.arange(v0, v1))
    rays = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones(uu.shape)], -1)
    rays = rays @ T[:3, :3].T
    ox, oy, oz = T[:3, 3]
    px, py = ox - centre_xy[0], oy - centre_xy[1]
    a = rays[..., 0] ** 2 + rays[..., 1] ** 2
    b = 2.0 * (px * rays[..., 0] + py * rays[..., 1])
    c = px * px + py * py - SEARCH_R ** 2
    root = np.sqrt(np.maximum(b * b - 4.0 * a * c, 0.0))
    hits = (b * b - 4.0 * a * c) >= 0.0
    with np.errstate(divide='ignore', invalid='ignore'):
        t_in = np.where(hits, (-b - root) / (2.0 * a), np.inf)
        t_out = np.where(hits, (-b + root) / (2.0 * a), -np.inf)
        z0 = (slab[0] - oz) / rays[..., 2]
        z1 = (slab[1] - oz) / rays[..., 2]
    level = rays[..., 2] == 0.0
    inside = (slab[0] <= oz) & (oz <= slab[1])
    z_in = np.where(level, -np.inf if inside else np.inf, np.minimum(z0, z1))
    z_out = np.where(level, np.inf if inside else -np.inf, np.maximum(z0, z1))
    return (np.maximum.reduce([t_in, z_in, np.zeros(uu.shape)]),
            np.minimum(t_out, z_out))


def _nearest_on_circle(d, r):
    n = np.linalg.norm(d, axis=1, keepdims=True)
    return r * d / np.maximum(n, 1e-9)


def _nearest_on_square(d, h):
    q = np.clip(d, -h, h)
    inside = np.all(np.abs(d) < h, axis=1)
    if inside.any():
        di = d[inside]
        axis = np.argmin(h - np.abs(di), axis=1)
        rows = np.arange(len(di))
        qi = di.copy()
        qi[rows, axis] = np.sign(di[rows, axis]) * h
        q[inside] = qi
    return q


def fit_centre(points_xy, profile, towards_camera):
    """Fit the known outline to the camera-facing points by translation-only Gauss-Newton."""
    nearest = (_nearest_on_circle if profile.shape == 'circle'
               else _nearest_on_square)
    centre = np.median(points_xy, axis=0) - towards_camera * profile.size
    for _ in range(FIT_ITERATIONS):
        d = points_xy - centre
        step = np.mean(d - nearest(d, profile.size), axis=0)
        centre = centre + step
        if np.hypot(*step) < 1e-5:
            break
    d = points_xy - centre
    rms = float(np.sqrt(np.mean(np.sum(
        (d - nearest(d, profile.size)) ** 2, axis=1))))
    return centre, rms


@dataclass
class CameraPose:
    """Bottle centre fitted from depth, on the counter top, in world frame."""
    xyz: list
    confidence: float
    fit_rms_mm: float
    source: str = 'camera'


@dataclass
class Observation:
    """One bottle station as the stand camera saw it.

    /world merges these fields into the station. `occupied` and `in_place`
    are None when `observation` is 'unknown': don't know, never empty.
    """
    observation: str
    occupied: bool | None
    reason: str | None
    frame_age_s: float | None
    valid_px: int = 0
    foreground_px: int = 0
    confidence: float = 0.0
    in_place: bool | None = None
    offset_mm: float | None = None
    pose: CameraPose | None = None


def unknown(reason, frame=None, valid_px=0, foreground_px=0):
    return Observation(
        'unknown', None, reason,
        None if frame is None else round(frame.age_s, 3),
        valid_px, foreground_px)


def observe(frame, calib, profile, expected_xy):
    """Return the fields /world merges into one bottle station."""
    if frame is None:
        return unknown('no depth frame yet')
    if frame.age_s > MAX_FRAME_AGE_S:
        return unknown(f'depth frame is {frame.age_s:.1f}s old', frame)
    if frame.frame_id != calib.frame_id:
        return unknown(f'frame {frame.frame_id!r} does not match '
                       f'calibration {calib.frame_id!r}', frame)

    fx, fy, cx, cy = frame.K
    T = calib.T_world_optical
    lo, hi = (L.COUNTER_Z + z for z in profile.slab)
    ex, ey = expected_xy
    box, why = station_box(frame.K, T, expected_xy, SEARCH_R, (lo, hi),
                           frame.depth.shape)
    if box is None:
        return unknown(why, frame)
    u0, u1, v0, v1 = box

    roi = frame.depth[v0:v1, u0:u1]
    near, far = calib.depth_range
    valid = np.isfinite(roi) & (roi >= near) & (roi <= far)
    enter, leave = _volume_interval(T, frame.K, (u0, u1, v0, v1),
                                    (ex, ey), (lo, hi))
    crosses = enter < leave
    vv, uu = np.nonzero(valid)
    d = roi[valid]
    optical = np.stack([(uu + u0 - cx) / fx * d, (vv + v0 - cy) / fy * d, d],
                       axis=1)
    world = optical @ T[:3, :3].T + T[:3, 3]

    in_slab = (world[:, 2] >= lo) & (world[:, 2] <= hi)
    in_radius = np.hypot(world[:, 0] - ex, world[:, 1] - ey) <= SEARCH_R
    foreground = in_slab & in_radius
    station_depth = _to_optical(T, [(ex, ey, (lo + hi) / 2.0)])[0, 2]

    valid_crossing = valid & crosses
    with np.errstate(invalid='ignore'):
        occluded = int((valid_crossing & (roi < enter)).sum())
        see_through = int((valid_crossing & (roi > leave)).sum())
    n_crossing, n_valid_crossing = int(crosses.sum()), int(valid_crossing.sum())

    # ponytail: ignores the pitch's cos(elevation) shrink, so the fractions are slightly lenient.
    expected_px = (2.0 * profile.size * fx / station_depth) * \
        ((hi - lo) * fy / station_depth)
    n_valid, n_fg = int(valid.sum()), int(foreground.sum())

    if n_fg < MIN_FOREGROUND_FRACTION * expected_px:
        if n_crossing and \
                n_valid_crossing >= MISSING_MIN_VALID_FRACTION * n_crossing and \
                see_through >= MISSING_MIN_SEE_THROUGH_FRACTION * n_valid_crossing and \
                occluded <= MISSING_MAX_OCCLUDED_FRACTION * n_valid_crossing:
            return Observation(
                'missing', False, 'saw through the bottle volume',
                round(frame.age_s, 3), n_valid, n_fg, confidence=1.0,
                in_place=False)
        if n_valid_crossing < MISSING_MIN_VALID_FRACTION * n_crossing:
            reason = 'too few valid depth pixels'
        elif occluded > MISSING_MAX_OCCLUDED_FRACTION * n_valid_crossing:
            reason = 'something is in front of it'
        else:
            reason = 'partial view of the bottle volume'
        return unknown(reason, frame, n_valid, n_fg)

    camera_xy = T[:2, 3]
    towards = camera_xy - np.array(expected_xy)
    towards = towards / np.hypot(*towards)
    centre, rms = fit_centre(world[foreground, :2], profile, towards)
    offset_mm = float(np.hypot(centre[0] - ex, centre[1] - ey)) * 1000.0
    confidence = min(1.0, n_fg / expected_px) * \
        max(0.0, 1.0 - rms * 1000.0 / TOL_MM)
    confidence = round(confidence, 3)
    return Observation(
        'observed', True, None, round(frame.age_s, 3), n_valid, n_fg,
        confidence, in_place=offset_mm <= TOL_MM,
        offset_mm=round(offset_mm, 1),
        pose=CameraPose(
            [round(float(centre[0]), 4), round(float(centre[1]), 4),
             L.COUNTER_Z],
            confidence, round(rms * 1000.0, 1)))
