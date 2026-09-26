"""Tests for OpenActionServer._wait_for_beer_to_settle.

The bell is aimed at the cap from a bottle arm A is dangling in the air, and
that bottle keeps swinging on the pads for real seconds after the lift's own
trajectory reports done -- see BEER_SETTLE_MOVE in open_action_server.py for
the trace that measured it (63mm of drift over about 6s). A press aimed
before that dies out is aimed at a point the bottle has since left, and it is
the leading suspect for a run that put the bell 114.8mm off centre while the
grasps either side of it were clean.

This is pure Python -- a poll loop over a pose function -- so it is tested
here directly, against a bare stand-in object, the same way
test_teach_points.py drives TeachNode.command_gripper unbound. No node, no
executor, no sim.
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open'))

from bartender_open import open_action_server as oas     # noqa: E402


class _FakeClock:
    """A clock that advances only when told to, so the test does not sleep."""

    def __init__(self, start=1_000_000.0):
        self.now = start

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class _FakeLogger:

    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)


def _server(pose_sequence):
    """Build a bare stand-in carrying only what _wait_for_beer_to_settle touches.

    `pose_sequence` is popped from the front on every call to `_pose`; the
    last entry repeats once the list is exhausted, so a test only has to
    describe the interesting part of the motion.
    """
    poses = list(pose_sequence)
    logger = _FakeLogger()
    node = types.SimpleNamespace(
        _pose=lambda name: poses.pop(0) if len(poses) > 1 else poses[0],
        get_logger=lambda: logger,
    )
    return node, logger


def test_a_beer_that_has_already_stopped_settles_quickly(monkeypatch):
    """A steady reading should not cost anywhere near the full timeout."""
    clock = _FakeClock()
    monkeypatch.setattr(oas.time, 'time', clock.time)
    monkeypatch.setattr(oas.time, 'sleep', clock.sleep)
    node, logger = _server([(0.10, 0.02, 0.94)])

    where = oas.OpenActionServer._wait_for_beer_to_settle(node)

    assert where == (0.10, 0.02, 0.94)
    assert clock.now - 1_000_000.0 < oas.BEER_SETTLE_WINDOW_S + 0.2
    assert logger.warnings == []
    assert any('settled' in m for m in logger.infos)


def test_a_swinging_beer_is_not_declared_settled_mid_swing(monkeypatch):
    """Fed a steadily moving reading, it must not return early.

    If it did, it would be reporting the peak of a swing as a rest position
    -- which is exactly the bug this function exists to close.
    """
    clock = _FakeClock()
    monkeypatch.setattr(oas.time, 'time', clock.time)
    monkeypatch.setattr(oas.time, 'sleep', clock.sleep)
    # Keeps drifting by 5mm in x on every poll, forever: never quiet.
    drifting = [(0.10 + 0.005 * i, 0.0, 0.94) for i in range(2000)]
    node, logger = _server(drifting)

    where = oas.OpenActionServer._wait_for_beer_to_settle(node)

    assert clock.now - 1_000_000.0 >= oas.BEER_SETTLE_TIMEOUT_S
    assert any('had not settled' in m for m in logger.warnings)
    assert where is not None


def test_settling_needs_a_full_window_not_one_quiet_sample(monkeypatch):
    """One quiet reading sandwiched between moving ones must not pass.

    A settle check with no memory of the last window would be fooled by a
    beer that happens to be quiet at the exact instant it is polled.
    """
    clock = _FakeClock()
    monkeypatch.setattr(oas.time, 'time', clock.time)
    monkeypatch.setattr(oas.time, 'sleep', clock.sleep)
    sequence = (
        [(0.10 + 0.02 * i, 0.0, 0.94) for i in range(6)]   # still swinging
        + [(0.30, 0.0, 0.94)] * 40                          # then quiet
    )
    node, logger = _server(sequence)

    where = oas.OpenActionServer._wait_for_beer_to_settle(node)

    assert where == (0.30, 0.0, 0.94)
    assert any('settled' in m for m in logger.infos)


def test_no_pose_at_all_is_reported_as_none_not_a_crash(monkeypatch):
    """A beer that never turns up on the pose topic must not be guessed at."""
    clock = _FakeClock()
    monkeypatch.setattr(oas.time, 'time', clock.time)
    monkeypatch.setattr(oas.time, 'sleep', clock.sleep)
    node = types.SimpleNamespace(_pose=lambda name: None,
                                 get_logger=lambda: _FakeLogger())

    assert oas.OpenActionServer._wait_for_beer_to_settle(node) is None
