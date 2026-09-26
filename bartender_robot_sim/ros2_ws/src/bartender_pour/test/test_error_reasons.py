"""Tests for PourActionServer.last_error / _why.

Mirrors bartender_open's test_error_reasons.py (Arm.last_error and
OpenActionServer._why/_why_any), for the same reason: before this, a
sub-call (a Cartesian plan, a grasp, a gripper command) logged a specific
reason and the execute_callback's own result message
("failed while pouring cola: closing_on_cola") had nothing more specific to
say. See docs/CONTROL_API.md's note on this being "real, undone work" --
this is that work.
"""
import inspect
import os
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_pour import pour_action_server as pas          # noqa: E402


def test_a_logged_error_is_remembered_as_last_error():
    """_error must set last_error, not just log -- that is its job."""
    fake = types.SimpleNamespace(
        last_error=None,
        get_logger=lambda: types.SimpleNamespace(error=lambda m: None))
    pas.PourActionServer._error(fake, 'Cartesian plan "cola tilt" timed out')
    assert fake.last_error == 'Cartesian plan "cola tilt" timed out'


def test_a_fresh_server_has_no_last_error_to_report():
    """__init__ must start last_error at None, not leave it unset.

    Checked against the source rather than by constructing a real
    PourActionServer, which needs a live rclpy node and several action
    clients.
    """
    assert 'self.last_error = None' in inspect.getsource(
        pas.PourActionServer.__init__)


def test_why_appends_the_servers_own_reason():
    node = types.SimpleNamespace(last_error='fingers stalled at 0.089')
    assert pas.PourActionServer._why(node) == ' (fingers stalled at 0.089)'


def test_why_says_nothing_extra_when_there_is_no_reason():
    """No reason logged must not print "(None)" -- that is worse than silence."""
    node = types.SimpleNamespace(last_error=None)
    assert pas.PourActionServer._why(node) == ''
