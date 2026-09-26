"""Tests for Arm.last_error and OpenActionServer._why / _why_any.

Before these existed, a failed sub-call (a grasp, a move) logged a specific
reason and then handed the caller only True/False; the caller's own
message ("arm B could not pick up the opener") had nothing more specific
to say, and the two most carefully distinguished failures in this project
-- GRIPPER_NOT_FOLLOWING and GRASP_STOPPED_WIDE -- collapsed back into that
one coarse sentence the moment they reached the action's result. This is
what bartender_api's error-taxonomy classifier needs to tell them apart
again.
"""
import inspect
import os
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), *([os.pardir] * 4),
    'ros2_ws', 'src', 'bartender_open'))

from bartender_open.arm import Arm                        # noqa: E402
from bartender_open import open_action_server as oas       # noqa: E402


def test_a_logged_error_is_remembered_as_last_error():
    """Arm._error must set last_error, not just log -- that is its job."""
    fake = types.SimpleNamespace(
        last_error=None,
        _log=lambda: types.SimpleNamespace(error=lambda m: None))
    Arm._error(fake, 'fingers closed on nothing 24.0mm wide')
    assert fake.last_error == 'fingers closed on nothing 24.0mm wide'


def test_a_fresh_arm_has_no_last_error_to_report():
    """Arm.__init__ must start last_error at None, not leave it unset.

    A caller that checks `arm.last_error` before anything has ever failed
    -- exactly what OpenActionServer._why does on an arm that has not
    yet logged anything -- must get a clean "nothing to say" rather than
    an AttributeError. Checked against the source rather than by
    constructing a real Arm, which needs a live rclpy node and clients.
    """
    assert 'self.last_error = None' in inspect.getsource(Arm.__init__)


def test_why_appends_the_arms_own_reason():
    arm = types.SimpleNamespace(label='arm B', last_error='fingers stalled')
    assert oas.OpenActionServer._why(arm) == ' (fingers stalled)'


def test_why_says_nothing_extra_when_the_arm_has_no_reason():
    """No reason logged must not print "(None)" -- that is worse than silence."""
    arm = types.SimpleNamespace(label='arm B', last_error=None)
    assert oas.OpenActionServer._why(arm) == ''


def test_why_any_names_which_arm_said_what():
    a = types.SimpleNamespace(label='arm A', last_error='lost the beer')
    b = types.SimpleNamespace(label='arm B', last_error=None)
    assert oas.OpenActionServer._why_any(a, b) == ' (arm A: lost the beer)'


def test_why_any_is_silent_when_neither_arm_has_a_reason():
    a = types.SimpleNamespace(label='arm A', last_error=None)
    b = types.SimpleNamespace(label='arm B', last_error=None)
    assert oas.OpenActionServer._why_any(a, b) == ''


def test_why_any_reports_both_when_both_have_something_to_say():
    """Genuinely ambiguous -- _stow moves both arms -- so show both, not one."""
    a = types.SimpleNamespace(label='arm A', last_error='could not retreat')
    b = types.SimpleNamespace(label='arm B', last_error='plan only reached 4%')
    said = oas.OpenActionServer._why_any(a, b)
    assert 'arm A: could not retreat' in said
    assert 'arm B: plan only reached 4%' in said
