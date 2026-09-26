"""Camera-free tests for the stand-depth bottle observation contract.

The renderer below traces pinhole-camera rays through simple vertical bottle
profiles and a counter plane.  It deliberately produces the depth image the
same way a camera would, rather than handing world points straight to the
fitter: projection, back-projection, ROI selection and missing evidence all
remain under test without Gazebo or an OAK-D.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_open'))

from bartender_api import perception                       # noqa: E402
from bartender_open import layout as L                      # noqa: E402


WIDTH, HEIGHT = 320, 240
K = (400.0, 400.0, WIDTH / 2.0, HEIGHT / 2.0)
# Camera optical z looks along world +y, image right is world +x and image
# down is world -z.  It is deliberately translated and rotated from world.
T_WORLD_OPTICAL = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, -0.80],
    [0.0, -1.0, 0.0, 1.12],
    [0.0, 0.0, 0.0, 1.0],
])
CALIB = perception.Calibration(
    'synthetic_stand_optical_frame', T_WORLD_OPTICAL, (0.2, 5.0))


def _frame(depth, age_s=0.0, frame_id=CALIB.frame_id):
    return perception.Frame(depth, K, frame_id, age_s)


def _circle_depth(a, tx, ty, ex, ey, radius):
    """First optical-z intersection of each ray with a vertical cylinder."""
    qa = a * a + 1.0
    qb = 2.0 * (a * (tx - ex) + ty - ey)
    qc = (tx - ex) ** 2 + (ty - ey) ** 2 - radius ** 2
    discriminant = qb * qb - 4.0 * qa * qc
    result = np.full(a.shape, np.inf)
    possible = discriminant >= 0.0
    root = np.sqrt(np.maximum(discriminant, 0.0))
    first = (-qb - root) / (2.0 * qa)
    second = (-qb + root) / (2.0 * qa)
    result[possible & (first > 0.0)] = first[possible & (first > 0.0)]
    use_second = possible & ~(first > 0.0) & (second > 0.0)
    result[use_second] = second[use_second]
    return result


def _square_depth(a, tx, ty, ex, ey, half_width):
    """First optical-z intersection of each ray with an axis-aligned box."""
    result = np.full(a.shape, np.inf)
    for x_face in (ex - half_width, ex + half_width):
        with np.errstate(divide='ignore', invalid='ignore'):
            d = (x_face - tx) / a
        y = ty + d
        valid = (d > 0.0) & (np.abs(y - ey) <= half_width)
        result[valid] = np.minimum(result[valid], d[valid])
    for y_face in (ey - half_width, ey + half_width):
        d = np.full(a.shape, y_face - ty)
        x = tx + a * d
        valid = (d > 0.0) & (np.abs(x - ex) <= half_width)
        result[valid] = np.minimum(result[valid], d[valid])
    return result


def _render(bottles=(), dropout=0.0, seed=0):
    """Render a counter and `(profile, xy)` vertical bottle bodies."""
    fx, fy, cx, cy = K
    uu, vv = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    a, b = (uu - cx) / fx, (vv - cy) / fy
    tx, ty, tz = T_WORLD_OPTICAL[:3, 3]
    # world z = tz - b * depth; counter is the background that proves a
    # missing bottle volume is visible through.
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = (tz - L.COUNTER_Z) / b
    depth[(b <= 0.0) | (depth <= 0.0)] = np.nan

    for profile, (ex, ey) in bottles:
        if profile.shape == 'circle':
            candidate = _circle_depth(a, tx, ty, ex, ey, profile.size)
        else:
            candidate = _square_depth(a, tx, ty, ex, ey, profile.size)
        # Pixels that miss the body hold inf.  They are discarded below;
        # avoid an inf*0 warning at the optical centre while doing so.
        with np.errstate(invalid='ignore'):
            z = tz - b * candidate
        body = ((z >= L.COUNTER_Z) &
                (z <= L.COUNTER_Z + max(profile.slab) + 0.08))
        candidate[~body] = np.inf
        replace = np.isfinite(candidate) & (
            ~np.isfinite(depth) | (candidate < depth))
        depth[replace] = candidate[replace]

    if dropout:
        rng = np.random.default_rng(seed)
        depth[rng.random(depth.shape) < dropout] = np.nan
    return depth


def _observe(name, offset=(0.0, 0.0), **render_args):
    profile = perception.PROFILES[name]
    expected = L.STATIONS[name]
    actual = (expected[0] + offset[0], expected[1] + offset[1])
    depth = _render([(profile, actual)], **render_args)
    return perception.observe(_frame(depth), CALIB, profile, expected)


def test_projection_and_back_projection_round_trip_under_rotated_camera():
    world = np.array([[0.08, -0.30, 1.00], [0.12, 0.00, 1.04]])
    optical = perception._to_optical(T_WORLD_OPTICAL, world)
    assert np.all(optical[:, 2] > 0.0)
    rebuilt = optical @ T_WORLD_OPTICAL[:3, :3].T + T_WORLD_OPTICAL[:3, 3]
    assert np.allclose(rebuilt, world)


def test_known_circle_and_square_profiles_are_observed_at_their_stations():
    for name in ('whiskey', 'cola', 'beer'):
        result = _observe(name)
        assert result.observation == 'observed', (name, result)
        assert result.occupied is True
        assert result.in_place is True
        assert result.offset_mm <= 2.0
        assert result.pose.source == 'camera'
        assert result.valid_px >= result.foreground_px > 0


def test_32fc1_and_16uc1_decode_to_the_same_metres_and_invalid_pixels():
    metres = np.array([[1.0, np.nan, np.inf], [0.0, 1.234, 2.5]], dtype='<f4')
    decoded_float = perception.decode_depth(
        '32FC1', metres.tobytes(), 2, 3, 3 * 4, False)
    millimetres = np.array([[1000, 0, 0], [0, 1234, 2500]], dtype='<u2')
    decoded_uint = perception.decode_depth(
        '16UC1', millimetres.tobytes(), 2, 3, 3 * 2, False)
    assert np.allclose(decoded_float, decoded_uint, equal_nan=True)
    assert np.isnan(decoded_float[0, 1])
    assert np.isnan(decoded_float[0, 2])
    assert np.isnan(decoded_float[1, 0])
    assert perception.decode_depth('rgb8', b'', 0, 0, 0, False) is None


def test_stale_or_wrong_frame_is_unknown_not_ground_truth():
    depth = _render([(perception.PROFILES['cola'], L.STATIONS['cola'])])
    stale = perception.observe(_frame(depth, age_s=1.01), CALIB,
                               perception.PROFILES['cola'], L.STATIONS['cola'])
    wrong_frame = perception.observe(
        _frame(depth, frame_id='another_optical_frame'), CALIB,
        perception.PROFILES['cola'], L.STATIONS['cola'])
    for result in (stale, wrong_frame):
        assert result.observation == 'unknown'
        assert result.occupied is None
        assert result.in_place is None


def test_empty_visible_station_is_missing_but_depth_holes_are_unknown():
    profile, expected = perception.PROFILES['beer'], L.STATIONS['beer']
    missing = perception.observe(_frame(_render()), CALIB, profile, expected)
    holes = perception.observe(
        _frame(np.full((HEIGHT, WIDTH), np.nan)), CALIB, profile, expected)
    assert missing.observation == 'missing', missing
    assert missing.occupied is False
    assert holes.observation == 'unknown'
    assert holes.occupied is None


def test_opaque_blocker_in_front_of_the_station_is_unknown_not_missing():
    profile, expected = perception.PROFILES['beer'], L.STATIONS['beer']
    blocker = perception.Profile('circle', 0.02, (0.0, 0.2))
    camera = T_WORLD_OPTICAL[:2, 3]
    in_front = tuple(camera + 0.5 * (np.array(expected) - camera))
    for scene in ([(blocker, in_front)],
                  [(blocker, in_front), (profile, expected)]):
        result = perception.observe(_frame(_render(scene)), CALIB, profile,
                                    expected)
        assert result.observation == 'unknown', result
        assert result.occupied is None
        assert result.reason == 'something is in front of it'


def test_adjacent_bottle_does_not_become_the_target_bottle():
    expected = L.STATIONS['cola']
    adjacent = (expected[0], expected[1] + L.SLOT_PITCH)
    depth = _render([(perception.PROFILES['cola'], adjacent)])
    result = perception.observe(_frame(depth), CALIB,
                                perception.PROFILES['cola'], expected)
    assert result.observation == 'missing', result


def test_dropout_lowers_confidence_then_becomes_unknown_not_missing():
    clean = _observe('beer')
    noisy = _observe('beer', dropout=0.30, seed=7)
    sparse = _observe('beer', dropout=0.90, seed=7)
    assert noisy.observation == 'observed', noisy
    assert noisy.confidence < clean.confidence
    assert sparse.observation == 'unknown', sparse
    assert sparse.occupied is None


def test_tolerance_boundary_sets_in_place_from_the_measured_offset():
    inside = _observe('cola', offset=(0.010, 0.0))
    outside = _observe('cola', offset=(0.020, 0.0))
    assert inside.observation == outside.observation == 'observed'
    assert inside.offset_mm < perception.TOL_MM
    assert inside.in_place is True
    assert outside.offset_mm > perception.TOL_MM
    assert outside.in_place is False


def test_sim_yaml_matches_the_sdf_camera_pose_and_optical_rotation():
    root = os.path.join(os.path.dirname(__file__), *([os.pardir] * 4))
    config_path = os.path.join(
        root, 'ros2_ws', 'src', 'bartender_api', 'config',
        'stand_camera.yaml')
    config = perception.load_calibration(config_path)
    world_path = os.path.join(
        root, 'ros2_ws', 'src', 'bartender_gazebo', 'worlds', 'bar_world.sdf')
    model = next(m for m in ET.parse(world_path).iter('model')
                 if m.attrib.get('name') == 'oak_d_stand')
    x, y, z, roll, pitch, yaw = map(float, model.findtext('pose').split())
    assert roll == 0.0
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    ry = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                   [0.0, 1.0, 0.0],
                   [-math.sin(pitch), 0.0, math.cos(pitch)]])
    body_to_optical = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])
    assert np.allclose(config.T_world_optical[:3, :3],
                       rz @ ry @ body_to_optical, atol=1e-5)
    assert np.allclose(config.T_world_optical[:3, 3], (x, y, z))
    assert config.frame_id == 'stand_camera_optical_frame'
