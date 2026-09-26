"""Opt-in acceptance test for a calibrated, physically installed OAK-D.

This test is intentionally skipped in normal CI and local camera-free runs.
To run it, arrange whiskey, cola and beer upright at their layout stations,
keep both arms out of the stand camera view, launch the OAK-D depth driver,
then set `BARTENDER_HARDWARE_CAMERA=1`.  The depth image must be aligned to
RGB/rectified and its CameraInfo frame must match the hardware calibration.
"""
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_open'))

from bartender_api import perception                       # noqa: E402
from bartender_open import layout as L                      # noqa: E402


pytestmark = pytest.mark.skipif(
    os.environ.get('BARTENDER_HARDWARE_CAMERA') != '1',
    reason='requires an installed OAK-D and deliberate hardware opt-in')

DEPTH_TOPIC = '/bartender/stand_camera/depth'
CAMERA_INFO_TOPIC = '/bartender/stand_camera/camera_info'


def test_installed_oak_d_observes_all_bottles_in_place():
    """Check the live driver stream and the hand-placed calibration fixture."""
    config_path = os.environ.get('BARTENDER_STAND_CAMERA_CONFIG')
    assert config_path, ('set BARTENDER_STAND_CAMERA_CONFIG to the YAML '
                         'written by calibrate_stand_camera.py')
    calibration = perception.load_calibration(config_path)

    # ROS imports stay inside the opt-in test so a camera-free test run does
    # not even require a ROS installation.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node('stand_camera_hardware_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    latest = {'depth': None, 'info': None}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, DEPTH_TOPIC,
                             lambda msg: latest.update(depth=msg), qos)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                             lambda msg: latest.update(info=msg), qos)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not all(latest.values()):
            executor.spin_once(timeout_sec=0.2)
        assert latest['depth'] is not None, f'no image on {DEPTH_TOPIC}'
        assert latest['info'] is not None, (
            f'no CameraInfo on {CAMERA_INFO_TOPIC}')

        image, info = latest['depth'], latest['info']
        assert image.encoding == '16UC1', 'OAK-D hardware depth must be 16UC1'
        assert info.header.frame_id == calibration.frame_id
        assert (info.width, info.height) == (image.width, image.height)
        depth = perception.decode_depth(
            image.encoding, image.data, image.height, image.width,
            image.step, image.is_bigendian)
        K = perception.intrinsics(info.k, info.d)
        assert K is not None, 'depth must be rectified (CameraInfo.D == 0)'
        assert np.isfinite(depth).mean() > 0.05, 'depth frame is mostly holes'

        frame = perception.Frame(depth, K, info.header.frame_id, 0.0)
        for name, profile in perception.PROFILES.items():
            result = perception.observe(frame, calibration, profile,
                                        L.STATIONS[name])
            assert result.observation == 'observed', (name, result)
            assert result.occupied is True
            assert result.in_place is True
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
