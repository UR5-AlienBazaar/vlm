"""Tests for the gripper model and the joint-wrapping in bartender_open.arm.

Both of these are small pure functions that decide big things, and both were
put there after a specific failure that was hard to read.

The pad-gap curve turns "the neck is 38.7mm across" into "the knuckle will
touch at 0.46 rad". Before it existed the conversion was assumed linear, and
it is not; the fingers were commanded past the object and threw the bottle.
The curve's best property is that it can be checked against numbers this
project measured independently, in sim, for two other bottles, in another
package, before it was written -- so these tests are not circular.

wrap_to_pi turns an IK solution pressed against a joint limit into the
identical pose with room to move. A Cartesian move from the unwrapped one
cannot be followed through, which shows up as a plan that reaches 57% of a
straight line for no visible reason.
"""
import math
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open'))

from bartender_open.arm import (                        # noqa: E402
    Arm, GRASP_LOOSE, GRASP_PENETRATION, GRIPPER_FULLY_CLOSED,
    GRIPPER_LIMIT_MARGIN, GRIPPER_LOWER_LIMIT, GRIPPER_OPEN_POS,
    GRIPPER_PRE_CLOSE_GAP, GRIPPER_SQUEEZE, GRIPPER_UPPER_LIMIT,
    PAD_GAP, SIDE_QUAT,
    gap_for_knuckle, knuckle_for_gap, side_quat, wrap_to_pi,
)
from bartender_open import arm as arm_module            # noqa: E402
from bartender_open import layout as L                  # noqa: E402

# Measured in simulation by bartender_pour, for its own bottles, with no
# reference to this curve: see the WHISKEY and COLA blocks in
# pour_action_server.py. The whiskey's flats are 77.2mm and it records the
# knuckle stalling at 0.089; the cola is gripped at a 59.6mm waist and it
# records contact at about 0.265.
# What bartender_pour measured in simulation, for its own bottles, with no
# reference to this curve: see the WHISKEY and COLA blocks in
# pour_action_server.py. The two are different KINDS of measurement and the
# tests below treat them as such.
#
#   the cola's is a first-CONTACT figure -- "contact comes at ~0.265 rad and
#   the joint settles at 0.27-0.29" -- so the curve should hit it head on;
#   the whiskey's 0.089 is where the joint STALLED, which is past contact by
#   however far the pads sank in, and that file records 0.2mm for its flats.
MEASURED_CONTACT = [
    ('cola waist', 0.0596, 0.265),
]
MEASURED_STALL = [
    ('whiskey flats', 0.0772, 0.089, 0.0002),
]


@pytest.mark.parametrize('what,width,measured', MEASURED_CONTACT)
def test_the_curve_hits_a_measured_contact(what, width, measured):
    """The check that this model is of the real gripper and not of itself.

    Measured in simulation, for another bottle, by another package, before
    this curve existed. It is the only evidence available that the curve can
    be trusted for a bottle nobody has gripped yet, which is exactly what it
    is used for.
    """
    assert knuckle_for_gap(width) == pytest.approx(measured, abs=0.002)


@pytest.mark.parametrize('what,width,stall,penetration', MEASURED_STALL)
def test_a_measured_stall_sits_just_past_the_predicted_contact(
        what, width, stall, penetration):
    """And the size of the gap between them is itself the recorded penetration.

    The whiskey's flats predict contact at 0.0865 against a measured 0.089
    stall. That is 0.0025 rad, which on this curve is 0.24mm of pad
    penetration -- and 0.2mm is the figure pour_action_server records for
    that bottle. The agreement is in the residual as much as in the number,
    and a curve that matched the stall exactly would be matching something it
    does not model.
    """
    assert knuckle_for_gap(width) < stall
    sunk = width - gap_for_knuckle(stall)
    assert sunk == pytest.approx(penetration, abs=0.0003)


@pytest.mark.parametrize('knuckle', [0.0, 0.07, 0.25, 0.4612, 0.55, 0.8])
def test_gap_and_knuckle_invert_each_other(knuckle):
    assert knuckle_for_gap(gap_for_knuckle(knuckle)) == pytest.approx(
        knuckle, abs=1e-9)


def test_wider_than_the_gripper_opens_clamps_to_zero():
    assert knuckle_for_gap(0.20) == 0.0


def test_narrower_than_it_closes_clamps_to_the_end():
    assert knuckle_for_gap(-0.01) == PAD_GAP[-1][0]


def test_the_curve_covers_everything_this_action_grips():
    """Both objects must fall inside the table, not off its ends."""
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert PAD_GAP[-1][1] < width < PAD_GAP[0][1]


def test_the_pre_close_leaves_the_pads_clear_of_both_objects():
    """The one fast command must never reach the object.

    It is the only move big enough to overshoot much, and an overshoot that
    reaches the object is a collision at speed rather than a grasp.
    """
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        target = knuckle_for_gap(width + GRIPPER_PRE_CLOSE_GAP)
        assert gap_for_knuckle(target) > width


def test_the_close_has_room_to_find_both_objects():
    """Contact must come before the fingers run out of travel.

    If an object is so narrow that contact is past GRIPPER_FULLY_CLOSED, a
    perfectly good grasp is reported as "closed all the way without meeting
    anything".
    """
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert knuckle_for_gap(width) < GRIPPER_FULLY_CLOSED - 0.05


def test_the_squeeze_does_not_drive_past_the_linkage():
    for width in (L.BEER_WIDTH, L.OPENER_SHAFT):
        assert knuckle_for_gap(width) + GRIPPER_SQUEEZE < 0.80


def test_the_stall_window_is_asymmetric():
    """Too wide and too narrow are different failures and get different limits.

    Wide means the step has not arrived; narrow means the fingers are not on
    the object. Penetration is real and one-sided -- a round section needs
    several millimetres of it before the joint stalls, flats need a fraction
    of one -- so the allowance below the object's width has to be much larger
    than the allowance above it.
    """
    assert GRASP_PENETRATION > GRASP_LOOSE


def test_a_held_object_is_distinguishable_from_an_empty_gripper():
    """still_holding's allowance must be inside the squeeze.

    When the object goes, the fingers travel the whole squeeze command. The
    allowance has to be comfortably less than that or losing the bottle looks
    like holding it.
    """
    allowance = GRIPPER_SQUEEZE / 2.0
    assert allowance < GRIPPER_SQUEEZE
    lost = gap_for_knuckle(knuckle_for_gap(L.BEER_WIDTH) + GRIPPER_SQUEEZE)
    assert lost < L.BEER_WIDTH - 0.010


# -- joint wrapping ---------------------------------------------------------

@pytest.mark.parametrize('angle', [0.0, 1.0, -1.0, 3.0, -3.0,
                                   6.07, -6.07, 2 * math.pi, -2 * math.pi,
                                   12.0, -12.0])
def test_wrapping_never_changes_the_pose(angle):
    """A full turn of a revolute joint is the same configuration.

    This is what makes wrapping safe rather than a fudge: forward kinematics
    depends on the angle only through its sine and cosine, so the wrapped
    value puts every link in exactly the same place, with the same
    collisions.
    """
    wrapped = wrap_to_pi(angle)
    assert math.sin(wrapped) == pytest.approx(math.sin(angle), abs=1e-12)
    assert math.cos(wrapped) == pytest.approx(math.cos(angle), abs=1e-12)


@pytest.mark.parametrize('angle', [0.0, 1.0, -1.0, 6.07, -6.07, 12.0, -12.0])
def test_wrapping_lands_inside_the_range(angle):
    assert -math.pi < wrap_to_pi(angle) <= math.pi


def test_wrapping_moves_a_limit_hugging_solution_away_from_the_limit():
    """The case this exists for.

    b_wrist_2 at -6.07 rad has 0.21 rad left before its -2pi limit, which is
    not enough to follow a 100mm straight line. The same pose at +0.21 has
    the whole range.
    """
    assert abs(wrap_to_pi(-6.07)) < 0.25
    assert abs(-6.07) > 6.0


def test_wrapping_is_idempotent():
    for angle in (0.5, -0.5, 3.0, -3.0, 6.07):
        assert wrap_to_pi(wrap_to_pi(angle)) == pytest.approx(
            wrap_to_pi(angle), abs=1e-15)


# -- the side grasp orientation ---------------------------------------------

def test_side_quat_is_the_cyclic_axis_permutation():
    """tool0's +z onto the arm's +x, its +x onto +y, its +y onto +z.

    Same quaternion bartender_pour uses. Expressed in each arm's OWN base
    frame, which is why neither arm needs a special case: for arm A it means
    approach along world +x, and for arm B -- yawed -90 degrees -- along
    world -y, in over the counter, which is the direction it wants anyway.
    """
    assert SIDE_QUAT == pytest.approx((0.5, 0.5, 0.5, 0.5), abs=1e-12)


def test_side_quat_is_a_unit_quaternion():
    for theta in (0.0, 0.3, -0.7, 1.7):
        assert sum(v * v for v in side_quat(theta)) == pytest.approx(
            1.0, abs=1e-12)


def test_side_quat_maps_tool_z_onto_the_arms_x():
    x, y, z, w = SIDE_QUAT
    # Third column of the rotation matrix: where local +z ends up.
    local_z = (2 * (x * z + y * w), 2 * (y * z - x * w),
               1 - 2 * (x * x + y * y))
    assert local_z == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


def test_side_quat_maps_tool_y_onto_the_arms_z():
    """Which is what stands a grasped bottle up along the gripper's free axis."""
    x, y, z, w = SIDE_QUAT
    local_y = (2 * (x * y - z * w), 1 - 2 * (x * x + z * z),
               2 * (y * z + x * w))
    assert local_y == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)


# -- the close loop ---------------------------------------------------------
#
# Arm.grasp is driven here through a stand-in that answers the four things it
# asks of the robot, so the LOOP is what is under test rather than the pure
# helpers above.
#
# It is tested because this loop is where a guard sat unreachable. The
# "gripper is not moving" check was written for a gripper jammed at its open
# stop, and it was placed after a width test that such a gripper always takes
# the other branch of -- so it never once ran, and the failure it exists to
# name was reported as "fingers closed all the way without meeting anything"
# instead. Nothing caught that, because nothing drove the loop.

class FakeGripper:
    """Answers Arm.grasp's questions with a scripted knuckle.

    `follow` decides where the joint ends up for a command: a real gripper
    tracks until something stops it, a jammed one never moves at all.
    """

    label = 'test arm'

    def __init__(self, follow):
        self._follow = follow
        self.position = 0.0
        self.commands = []

    # -- the four things grasp() needs -----------------------------------

    def gripper_position(self):
        return self.position

    def _send_gripper(self, command):
        self.commands.append(command)
        self.position = self._follow(command)
        return types.SimpleNamespace(accepted=True)

    def wait_for_gripper(self, command=None, timeout_s=None):
        return self.position

    def _log(self):
        return types.SimpleNamespace(
            info=lambda *a, **k: None, warn=lambda *a, **k: None,
            error=lambda *a, **k: None)

    def _error(self, message):
        """Match Arm._error: log through whatever _log is set to, and remember.

        grasp() calls self._error(...) now, not self._log().error(...)
        directly, so a fake standing in for self needs this too.
        """
        self.last_error = message
        self._log().error(message)


def grasp_with(follow, width):
    """Run the real Arm.grasp against a scripted gripper."""
    fake = FakeGripper(follow)
    return Arm.grasp(fake, width), fake


def test_a_gripper_that_never_moves_is_reported_as_not_moving(caplog):
    """The case the guard exists for: the joint stays at its open stop."""
    result, fake = grasp_with(lambda command: 0.0, L.BEER_WIDTH)
    assert result is None
    # and it gives up quickly rather than walking the command to fully closed
    assert max(fake.commands) < GRIPPER_FULLY_CLOSED


def test_a_gripper_that_never_moves_does_not_report_meeting_nothing():
    """Name the gripper, not the object.

    The message this replaced described where the opener was, and sent the
    search to the wrong place entirely.
    """
    messages = []
    fake = FakeGripper(lambda command: 0.0)
    fake._log = lambda: types.SimpleNamespace(
        info=lambda m, *a: None, warn=lambda m, *a: None,
        error=lambda m, *a: messages.append(m))
    assert Arm.grasp(fake, L.BEER_WIDTH) is None
    assert any('not following' in m for m in messages)
    assert not any('without meeting anything' in m for m in messages)


def test_a_gripper_that_closes_on_nothing_runs_to_the_end():
    """Tell an empty gripper from a jammed one.

    A gripper that DOES follow, onto thin air, is the other failure, and it
    must not borrow the message for a gripper that never moved.
    """
    messages = []
    fake = FakeGripper(lambda command: command)      # tracks perfectly
    fake._log = lambda: types.SimpleNamespace(
        info=lambda m, *a: None, warn=lambda m, *a: None,
        error=lambda m, *a: messages.append(m))
    assert Arm.grasp(fake, L.BEER_WIDTH) is None
    assert any('without meeting anything' in m for m in messages)
    assert not any('not following' in m for m in messages)


def test_closing_onto_an_object_returns_where_the_fingers_stopped():
    """The normal case: the joint tracks until the pads meet the object."""
    stall = knuckle_for_gap(L.BEER_WIDTH)

    def follow(command):
        return min(command, stall)

    reached, _ = grasp_with(follow, L.BEER_WIDTH)
    assert reached == pytest.approx(stall, abs=1e-6)


def test_a_stall_far_wider_than_the_object_is_refused():
    """Stopping on something is not the same as stopping on the right thing."""
    def follow(command):
        return min(command, knuckle_for_gap(L.BEER_WIDTH * 3))

    reached, _ = grasp_with(follow, L.BEER_WIDTH)
    assert reached is None


# ---------------------------------------------------------------------------
# THE COMMAND BAND
#
# A knuckle left at rest ON its lower joint limit (0.0) stops responding to
# gripper commands for the rest of the simulator run -- goals still accepted,
# controller still active, the joint simply never moves again. It broke about
# half of all opens before it was found. The measurements, and the list of
# explanations ruled out, are beside GRIPPER_LOWER_LIMIT in arm.py.
#
# So the fix is a number, 0.02, and a number is exactly the kind of thing
# somebody tidies back to 0.0 later. These tests are here to stop that.

# What was measured, and what the band has to stay on the right side of.
MEASURED_DEAD_AT = 0.0      # 10s parked here was already fatal
MEASURED_SAFE_AT = 0.02     # 300s parked here was fine


def test_the_open_position_is_not_the_joints_lower_limit():
    """The whole fix, in one assertion."""
    assert GRIPPER_OPEN_POS > GRIPPER_LOWER_LIMIT
    assert GRIPPER_OPEN_POS != MEASURED_DEAD_AT


def test_the_open_position_is_at_least_the_margin_measured_to_survive():
    """0.02 is not a guess; it is the smallest dwell-tested safe value."""
    assert GRIPPER_OPEN_POS >= MEASURED_SAFE_AT


def test_opening_still_clears_everything_this_robot_picks_up():
    """The margin has to be free, or it is not free.

    0.02 rad costs 1.8mm of the 85mm the pads open to, and the widest
    thing gripped here is 38.6mm. If that ever stopped being negligible
    the fix would need rethinking rather than nudging.
    """
    open_gap = gap_for_knuckle(GRIPPER_OPEN_POS)
    assert open_gap > max(L.OPENER_SHAFT, L.BEER_WIDTH) + 0.02
    assert gap_for_knuckle(GRIPPER_LOWER_LIMIT) - open_gap < 0.002


@pytest.mark.parametrize('width', [L.OPENER_SHAFT, L.BEER_WIDTH])
@pytest.mark.parametrize('name,follow', [
    ('tracks perfectly', lambda command: command),
    ('jammed at the stop', lambda command: 0.0),
    ('stops on the object', None),          # filled in below
])
def test_no_command_a_grasp_sends_lands_on_a_joint_limit(name, follow, width):
    """Drive the real close loop and watch every goal it produces.

    Checking the constant is not enough: grasp() derives its own commands
    from the pad-gap curve and from where the fingers got to, and `max(0.0,
    ...)` was how the old floor got in. This looks at what actually goes out.
    """
    if follow is None:
        stall = knuckle_for_gap(width)

        def follow(command):
            return min(command, stall)

    _, fake = grasp_with(follow, width)
    assert fake.commands, 'the loop sent nothing, so it checked nothing'
    assert min(fake.commands) >= GRIPPER_OPEN_POS, (
        f'{name}: grasp sent {min(fake.commands):.4f}, which is at or below '
        f'the open stop the gripper does not come back from')
    assert max(fake.commands) <= GRIPPER_UPPER_LIMIT


def test_send_gripper_refuses_a_command_on_the_open_stop():
    """Refused, not clamped -- and the client must never see it.

    Substituting 0.02 for a requested 0.0 would keep the robot working and
    hide the mistake from whoever wrote it, which is the trade this repo
    makes the other way round everywhere else.
    """
    sent = []
    fake = types.SimpleNamespace(
        label='test arm',
        gripper_client=types.SimpleNamespace(
            send_goal_async=lambda goal: sent.append(goal)),
        _log=lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None),
        _error=lambda msg: None)
    assert Arm._send_gripper(fake, GRIPPER_LOWER_LIMIT) is None
    assert Arm._send_gripper(fake, GRIPPER_UPPER_LIMIT + 0.01) is None
    assert sent == [], 'a refused command still reached the action client'


def test_the_refusal_says_why_rather_than_just_no():
    """Bare "out of range" would send the reader looking for a typo."""
    messages = []
    fake = types.SimpleNamespace(
        label='test arm',
        gripper_client=types.SimpleNamespace(send_goal_async=lambda goal: None),
        _log=lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None,
            error=lambda m, *a: messages.append(m)),
        _error=lambda msg: messages.append(msg))
    Arm._send_gripper(fake, 0.0)
    assert any('lower joint limit' in m for m in messages)


def test_the_band_sits_inside_the_grippers_real_joint_limits():
    """The URDF is where 0.0 and 0.8 come from; check they still are.

    robotiq_description is an installed package this repo does not control.
    If it ever re-specifies the knuckle, the constants above are describing
    a joint that no longer exists.
    """
    macro = ('/opt/ros/humble/share/robotiq_description/urdf/'
             'robotiq_2f_85_macro.urdf.xacro')
    if not os.path.exists(macro):
        pytest.skip(f'robotiq_description is not installed at {macro}')
    text = open(macro).read()
    match = re.search(
        r'name="\$\{prefix\}robotiq_85_left_knuckle_joint".*?'
        r'<limit lower="([-\d.]+)" upper="([-\d.]+)"',
        text, re.S)
    assert match, 'could not find the knuckle joint limit in the macro'
    lower, upper = float(match.group(1)), float(match.group(2))
    assert lower == GRIPPER_LOWER_LIMIT
    assert upper == GRIPPER_UPPER_LIMIT
    assert lower < GRIPPER_OPEN_POS <= GRIPPER_FULLY_CLOSED <= upper


@pytest.mark.parametrize('module,relative', [
    ('bartender_pour', 'bartender_pour/pour_action_server.py'),
    ('bartender_teach', 'bartender_teach/teach_points.py'),
])
def test_the_other_packages_carry_the_same_open_position(module, relative):
    """Three copies, because ROS packages here cannot import each other.

    Same rule as the geometry in layout.py: the copies stay, and the test
    makes them fail loudly when they drift. A package still opening to 0.0
    would kill its gripper on the first run and nothing else would notice.
    """
    path = os.path.join(REPO, 'ros2_ws', 'src', module, relative)
    if not os.path.exists(path):
        pytest.skip(f'{module} is not in this checkout')
    text = open(path).read()
    match = re.search(r'^GRIPPER_LIMIT_MARGIN = ([\d.]+)', text, re.M)
    assert match, f'{relative} has no GRIPPER_LIMIT_MARGIN'
    assert float(match.group(1)) == GRIPPER_LIMIT_MARGIN
    assert re.search(
        r'^GRIPPER_OPEN_POS = GRIPPER_LOWER_LIMIT \+ GRIPPER_LIMIT_MARGIN',
        text, re.M), (
        f'{relative} does not derive GRIPPER_OPEN_POS from the limit and '
        f'the margin, so the two can drift apart')


@pytest.mark.parametrize('clamp', [0.25, 0.44, 0.19, 0.3, 0.55, 0.7929])
def test_a_ramped_release_lands_exactly_on_the_open_position(clamp, monkeypatch):
    """The last step must BE the target, not arithmetic that nearly is.

    `start + (position - start) * i / steps` does not reach `position` in
    binary floating point. Releasing from 0.25 computes
    0.019999999999999990 for a target of 0.02 -- below GRIPPER_OPEN_POS, so
    the guard in _send_gripper refuses it and the release fails. Two of
    bartender_pour's three clamp angles do this. It was found by arithmetic
    rather than by a run, and it would have been a mystery in a log.
    """
    sent = []
    fake = types.SimpleNamespace(
        label='test arm',
        position=clamp,
        gripper_position=lambda: clamp,
        gripper_client=types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True),
        wait_for_gripper=lambda command=None, timeout_s=None: GRIPPER_OPEN_POS,
        _send_gripper=lambda p: (
            sent.append(p), types.SimpleNamespace(
                accepted=True,
                get_result_async=lambda: None))[1],
        _log=lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None),
        _error=lambda msg: None)
    monkeypatch.setattr(arm_module, 'block_on', lambda *a, **k: None)
    monkeypatch.setattr(arm_module.time, 'sleep', lambda seconds: None)
    Arm.command_gripper(fake, GRIPPER_OPEN_POS, ramp=True)
    assert sent[-1] == GRIPPER_OPEN_POS, (
        f'the release ended on {sent[-1]!r}, not {GRIPPER_OPEN_POS!r}')
    assert min(sent) >= GRIPPER_OPEN_POS


def _messages_from_grasp(follow, width, start=None):
    """Run the real close loop and collect what it said."""
    said = []
    fake = FakeGripper(follow)
    if start is not None:
        fake.position = start
    fake._log = lambda: types.SimpleNamespace(
        info=lambda *a: None, warn=lambda *a: None,
        error=lambda m, *a: said.append(m))
    return Arm.grasp(fake, width), said


def test_a_gripper_that_moved_and_then_stopped_wide_is_not_blamed():
    """The two faults the same test catches want opposite investigations.

    A gripper that never budged is a gripper fault. Fingers that closed
    perfectly well and then met something 25mm too wide are a PLACEMENT
    fault -- the arm is not where it should be, or the thing is not. An
    opener pick stopped with the pads 49.1mm apart on a 24.0mm shaft and
    the message blamed the gripper, which is the wrong half of the robot.
    """
    stall = knuckle_for_gap(L.OPENER_SHAFT * 2)      # stops far too wide

    reached, said = _messages_from_grasp(
        lambda command: min(command, stall), L.OPENER_SHAFT)
    assert reached is None
    assert said, 'the loop failed silently'
    assert not any('not following' in m for m in said), said
    assert any('on something else' in m for m in said), said


def test_a_gripper_that_never_moved_is_still_blamed():
    """The other branch, which is the one the guard was written for."""
    reached, said = _messages_from_grasp(
        lambda command: GRIPPER_OPEN_POS, L.OPENER_SHAFT)
    assert reached is None
    assert any('not following' in m for m in said), said


def test_the_two_diagnoses_are_mutually_exclusive():
    """One failure, one explanation; two would be worse than none."""
    for follow in (lambda command: GRIPPER_OPEN_POS,
                   lambda command: min(command,
                                       knuckle_for_gap(L.OPENER_SHAFT * 2))):
        _, said = _messages_from_grasp(follow, L.OPENER_SHAFT)
        assert len(said) == 1, said


@pytest.mark.parametrize('ended_at,ok', [
    (GRIPPER_OPEN_POS, True),
    (0.0, False),
    (0.005, False),
    (0.019, True),
])
def test_an_open_that_ends_on_the_stop_is_reported_there_and_then(
        ended_at, ok, monkeypatch):
    """Overshoot can still put the joint on the stop; silence cannot.

    Nothing commands 0.0 any more, but the ramp arrives at 0.5 rad/s and a
    real open was traced brushing -0.0000 before settling at 0.0200. A
    brush is survivable; being LEFT there is not, and the failure it causes
    turns up minutes later as an unexplained "gripper not following". Say
    it at the moment it happens.
    """
    said = []
    fake = types.SimpleNamespace(
        label='test arm',
        gripper_position=lambda: ended_at,
        gripper_client=types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True),
        wait_for_gripper=lambda command=None, timeout_s=None: ended_at,
        _send_gripper=lambda p: types.SimpleNamespace(
            accepted=True, get_result_async=lambda: None),
        _log=lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None,
            error=lambda m, *a: said.append(m)),
        _error=lambda msg: said.append(msg))
    monkeypatch.setattr(arm_module, 'block_on', lambda *a, **k: None)
    monkeypatch.setattr(arm_module.time, 'sleep', lambda seconds: None)
    assert Arm.command_gripper(fake, GRIPPER_OPEN_POS, ramp=True) is ok
    if not ok:
        assert any('lower stop' in m for m in said), said


def test_a_ramp_from_a_reading_below_the_floor_is_not_refused(monkeypatch):
    """A measured position is not a command, and must not become one.

    The ramp interpolates from where the fingers ARE, and the knuckle can
    be read below the band while it overshoots: a real open was traced
    through 0.0048 on its way back up to 0.0200, and every value in that
    dip is a reading some other thread can take. Interpolate a short move
    from one and the first step lands under GRIPPER_OPEN_POS, where
    _send_gripper refuses it -- so the command fails with "gripper goal
    rejected", a message about the wrong thing entirely.

    The window is narrow: it needs a reading in the dip AND a target close
    enough that one step does not clear the floor. Both halves are real,
    and the fix is to floor the interpolation origin.
    """
    sent = []
    # Reads 0.0048 -- a literal sample from that trace -- then tracks, as
    # the real joint does. A fake stuck down there would fail the
    # end-of-move checks instead and would not be testing this at all.
    reads = iter([0.0048])
    fake = types.SimpleNamespace(
        label='test arm',
        gripper_position=lambda: next(reads, 0.030),
        gripper_client=types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True),
        wait_for_gripper=lambda command=None, timeout_s=None: 0.030,
        _send_gripper=lambda p: (
            sent.append(p),
            types.SimpleNamespace(accepted=True,
                                  get_result_async=lambda: None))[1],
        _log=lambda: types.SimpleNamespace(
            info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None),
        _error=lambda msg: None)
    monkeypatch.setattr(arm_module, 'block_on', lambda *a, **k: None)
    monkeypatch.setattr(arm_module.time, 'sleep', lambda seconds: None)
    assert Arm.command_gripper(fake, 0.030, ramp=True)
    assert sent, 'nothing was sent'
    assert min(sent) >= GRIPPER_OPEN_POS, sent
