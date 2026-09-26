"""Tests for the shared radial-reach check used by both /world and /can.

See reach.py's own docstring for why this exists separately from
layout.servicing_arms(): that function answers "can this be side-grasped
off the line", which said no arm could reach the glass or the opener
holster -- both of which the real, working robot reaches every day,
because neither is side-grasped.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_open'))

from bartender_api import reach                             # noqa: E402
from bartender_open import layout as L                       # noqa: E402


def test_the_glass_is_within_reach_of_arm_a():
    glass_xy = L.STATIONS['glass']
    assert 'a' in reach.arms_within_reach(glass_xy)


def test_a_point_far_outside_the_workspace_is_reached_by_nobody():
    far_xy = (100.0, 100.0)
    assert reach.arms_within_reach(far_xy) == []


def test_reach_from_is_a_plain_straight_line_distance():
    # Arm A's own base: reach from itself is zero.
    assert reach.reach_from(L.ARM_A_ORIGIN[:2], 'a') == 0.0


def test_ur5e_max_reach_is_the_datasheet_figure_not_a_guess():
    """850mm, base to a fully extended tool at zero payload.

    Not measured in this sim -- flagged here so a future edit does not
    quietly turn it into an arbitrary tuning constant.
    """
    assert reach.UR5E_MAX_REACH == 0.850
