"""Tests for twin_mirror's one decision: what to command the twin."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_bringup.twin_mirror import (  # noqa: E402
    ARM_JOINTS, KNUCKLE, KNUCKLE_MAX, KNUCKLE_MIN, twin_command,
)

ARM = [0.1, -1.2, 0.8, -1.5, 0.05, 0.3]


def test_joints_are_matched_by_name_not_position():
    """/joint_states order is the broadcaster's, not the controller's."""
    names = list(reversed(ARM_JOINTS)) + [KNUCKLE]
    positions = list(reversed(ARM)) + [0.4]
    command, knuckle = twin_command(names, positions)
    assert command == ARM + [0.4]
    assert knuckle == 0.4


def test_a_message_without_the_whole_arm_is_skipped():
    assert twin_command(ARM_JOINTS[:5], ARM[:5]) is None


def test_a_non_finite_arm_joint_is_skipped():
    assert twin_command(ARM_JOINTS, ARM[:5] + [math.nan]) is None


def test_without_a_gripper_the_last_knuckle_is_kept():
    command, knuckle = twin_command(ARM_JOINTS, ARM, last_knuckle=0.5)
    assert command[-1] == 0.5 and knuckle == 0.5


@pytest.mark.parametrize('real, twin', [
    (0.0, KNUCKLE_MIN),       # the real gripper's "open"; 0.0 kills the sim knuckle
    (-0.01, KNUCKLE_MIN),
    (0.9, KNUCKLE_MAX),
    (0.4, 0.4),
])
def test_the_knuckle_is_kept_in_the_sims_working_band(real, twin):
    command, _ = twin_command(list(ARM_JOINTS) + [KNUCKLE], ARM + [real])
    assert command[-1] == twin
