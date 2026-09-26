"""Tests for tool centre points.

The invariant everything else rests on is that rotating about a tool tip does
not move that tip. A bug there is silent -- the arm moves, the pose is valid,
and the spout is simply somewhere other than over the glass -- so it is tested
directly, across arbitrary rotations and both bottles.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.tool_frames import (                   # noqa: E402
    IDENTITY, SIDE_GRIP_AHEAD_OF_TOOL0, SPOUT_OFF_AXIS, TOOL0, TOOLS, Tool,
    bottle_spout, get_tool, grasped_bottle_tool, quat_about, quat_conj,
    quat_mul, quat_rotate, rotate_about_tcp, tcp_from_tool0, tool0_from_tcp,
    translate_tcp,
)

# The side grasp's flange orientation: tool0 +Z -> base +X, +X -> +Y, +Y -> +Z.
SIDE = (0.5, 0.5, 0.5, 0.5)
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)

SPOUT = bottle_spout('whiskey')
ORIGINS = [(0.0, 0.0, 0.0), (0.55, 0.15, 0.30), (-0.2, 0.4, 1.1)]
QUATS = [IDENTITY, SIDE,
         quat_about(Y, 0.9), quat_about(X, -1.7),
         quat_mul(quat_about(Z, 0.4), quat_about(X, 1.1))]


def approx(v):
    return pytest.approx(v, abs=1e-9)


# -- the invariant ----------------------------------------------------------

@pytest.mark.parametrize('p0', ORIGINS)
@pytest.mark.parametrize('q0', QUATS)
@pytest.mark.parametrize('axis', [X, Y, Z, (0.6, 0.0, 0.8)])
@pytest.mark.parametrize('angle', [0.0, 0.3, -1.2, 1.7, math.pi])
def test_rotating_about_the_tip_never_moves_the_tip(p0, q0, axis, angle):
    """The whole point of a tool frame. If this fails, a pour misses the glass."""
    before, _ = tcp_from_tool0(p0, q0, SPOUT)
    p1, q1 = rotate_about_tcp(p0, q0, SPOUT, quat_about(axis, angle))
    after, _ = tcp_from_tool0(p1, q1, SPOUT)
    assert after == approx(before)


@pytest.mark.parametrize('q0', QUATS)
def test_rotating_about_the_tip_does_move_the_flange(q0):
    """Sanity check on the test above.

    A transform that moved nothing at all would also pass it.
    """
    p0 = (0.55, 0.15, 0.30)
    p1, _ = rotate_about_tcp(p0, q0, SPOUT, quat_about(Y, 1.0))
    assert math.dist(p0, p1) > 0.05


def test_the_flange_swing_is_what_the_old_hand_rolled_pour_avoided():
    """Rotating about tool0 instead swings the spout through a wide arc.

    This is the failure the tool frame prevents, and it is large: the tip
    stands ~0.23m off the flange, so a 97 degree turn about the flange throws
    it about a third of a metre.
    """
    p0, q0 = (0.55, 0.15, 0.30), SIDE
    tip_before, _ = tcp_from_tool0(p0, q0, SPOUT)
    naive_q = quat_mul(quat_about(Y, 1.7), q0)
    tip_naive, _ = tcp_from_tool0(p0, naive_q, SPOUT)
    assert math.dist(tip_before, tip_naive) > 0.25


# -- round trips ------------------------------------------------------------

@pytest.mark.parametrize('p0', ORIGINS)
@pytest.mark.parametrize('q0', QUATS)
@pytest.mark.parametrize('tool', [TOOL0, SPOUT, bottle_spout('cola'),
                                  Tool('odd', (0.01, -0.2, 0.05),
                                       quat_about(X, 0.7))])
def test_tcp_and_tool0_are_inverses(p0, q0, tool):
    pt, qt = tcp_from_tool0(p0, q0, tool)
    back_p, back_q = tool0_from_tcp(pt, qt, tool)
    assert back_p == approx(p0)
    # q and -q are the same rotation; compare by what they do to a vector.
    assert quat_rotate(back_q, (0.3, -0.7, 0.2)) == approx(
        quat_rotate(q0, (0.3, -0.7, 0.2)))


@pytest.mark.parametrize('p0', ORIGINS)
@pytest.mark.parametrize('q0', QUATS)
def test_tool0_is_a_no_op(p0, q0):
    """Selecting tool0 must leave every transform the identity.

    That is what makes tool frames safe to add to an already tuned system.
    """
    pt, qt = tcp_from_tool0(p0, q0, TOOL0)
    assert pt == approx(p0) and qt == approx(q0)
    p1, q1 = rotate_about_tcp(p0, q0, TOOL0, quat_about(Y, 0.8))
    assert p1 == approx(p0)


def test_translation_does_not_depend_on_the_tool():
    """A pure translation moves flange and tip together, whatever the tool."""
    p0, q0, d = (0.5, 0.1, 0.3), SIDE, (0.02, -0.01, 0.03)
    for tool in (TOOL0, SPOUT, bottle_spout('cola')):
        p1, q1 = translate_tcp(p0, q0, tool, d)
        assert p1 == approx((0.52, 0.09, 0.33))
        assert q1 == approx(q0)
        before, _ = tcp_from_tool0(p0, q0, tool)
        after, _ = tcp_from_tool0(p1, q1, tool)
        assert after == approx((before[0] + d[0], before[1] + d[1],
                                before[2] + d[2]))


# -- the geometry the offsets encode ----------------------------------------

def test_spout_offset_matches_the_bottle_model():
    """Cross-check against the numbers in models/jack_daniels_bottle."""
    tip_z, grasp_h = 0.2990, 0.1225
    assert SPOUT.xyz == approx(
        (0.0, tip_z - grasp_h, SIDE_GRIP_AHEAD_OF_TOOL0 + SPOUT_OFF_AXIS))


def test_upright_bottle_puts_the_spout_up_and_forward():
    """With the side grasp, the tip must land above and ahead of the flange.

    This is the axis permutation, and getting it wrong yields a tool frame
    that is plausible, self-consistent, and points where the bottle is not.
    """
    tip, _ = tcp_from_tool0((0.0, 0.0, 0.0), SIDE, SPOUT)
    assert tip[0] == approx(SIDE_GRIP_AHEAD_OF_TOOL0 + SPOUT_OFF_AXIS)  # +X
    assert tip[1] == approx(0.0)                                        # on Y
    assert tip[2] == approx(0.2990 - 0.1225)                            # up


def test_tilting_the_bottle_points_the_spout_downward():
    """The pourer must lean the way the pour tilt will turn it.

    It leans toward bottle-local +X so the tilt carries it downward rather
    than uselessly sideways.
    """
    # bottle-local +X maps to tool0 local +Z, which SIDE maps onto base +X
    lean_base = quat_rotate(SIDE, (0.0, 0.0, 1.0))
    assert lean_base == approx((1.0, 0.0, 0.0))
    # ...and the pour tilt about base Y carries base +X onto base -Z.
    assert quat_rotate(quat_about(Y, math.pi / 2), lean_base) == approx(
        (0.0, 0.0, -1.0))


def test_cola_spout_is_further_from_the_flange_than_the_whiskey():
    """It is gripped much lower down, so its spout stands further off."""
    assert bottle_spout('cola').reach > SPOUT.reach


# -- registry ---------------------------------------------------------------

def test_known_tools():
    assert set(TOOLS) == {'tool0', 'whiskey_spout', 'cola_spout',
                          'workcell_whiskey', 'workcell_vodka',
                          'workcell_gin'}
    for name, tool in TOOLS.items():
        assert tool.name == name


def test_get_tool_lists_the_alternatives():
    with pytest.raises(KeyError) as exc:
        get_tool('spanner')
    assert 'whiskey_spout' in str(exc.value)


def test_bottle_spout_refuses_an_unknown_bottle():
    with pytest.raises(KeyError):
        bottle_spout('absinthe')


def test_custom_tool_from_measurements():
    t = grasped_bottle_tool('gin_spout', above_grip=0.2, off_axis=0.01)
    assert t.xyz == approx((0.0, 0.2, SIDE_GRIP_AHEAD_OF_TOOL0 + 0.01))


# -- quaternion helpers -----------------------------------------------------

@pytest.mark.parametrize('q', QUATS)
def test_conj_undoes_a_rotation(q):
    v = (0.2, -0.5, 0.9)
    assert quat_rotate(quat_mul(q, quat_conj(q)), v) == approx(v)


# -- agreement with the hand-rolled pour it replaces -------------------------

def old_rotated_offset(along_axis, theta, grip_ahead):
    """pour_action_server's original _rotated_offset, verbatim.

    Kept here as the reference the general transform has to reproduce. It was
    correct for a pour point ON the bottle's axis, which is what the bare
    mouth was; it cannot express the pourer's tip, which is 17mm to the side.
    """
    return (grip_ahead * math.cos(theta) + along_axis * math.sin(theta),
            -grip_ahead * math.sin(theta) + along_axis * math.cos(theta))


def side_quat(theta):
    """pour_action_server's side_quat, verbatim."""
    c, s = math.cos(theta / 2.0), math.sin(theta / 2.0)
    return (0.5 * (c + s), 0.5 * (c + s), 0.5 * (c - s), 0.5 * (c - s))


@pytest.mark.parametrize('along', [0.1225, 0.19, -0.1225, -0.06, 0.0])
@pytest.mark.parametrize('step', range(0, 41, 4))
def test_general_form_reduces_to_the_old_expression(along, step):
    """With an on-axis tool the new transform must equal the old arithmetic.

    This is what makes replacing it safe: the pour geometry only changes
    because the spout is off-axis, not because the maths changed underneath
    it. Any drift here would move the pour and look like a tuning problem.
    """
    theta = 1.7 * step / 40.0
    grip, glass = 0.145, (0.55, -0.15, 0.30)
    tool = grasped_bottle_tool('axis', along, off_axis=0.0, grip_ahead=grip)

    dx, dz = old_rotated_offset(along, theta, grip)
    expected = (glass[0] - dx, glass[1], glass[2] - dz)
    got, _ = tool0_from_tcp(glass, side_quat(theta), tool)
    assert got == approx(expected)


@pytest.mark.parametrize('step', range(0, 41, 5))
def test_an_off_axis_tip_is_held_where_the_on_axis_one_would_not_be(step):
    """The reason the general form is needed at all.

    Feeding the off-axis spout through the old on-axis arithmetic leaves the
    tip somewhere else, and by more than the glass can absorb.
    """
    theta = 1.7 * step / 40.0
    grip, glass = 0.145, (0.55, -0.15, 0.30)
    spout = grasped_bottle_tool('spout', 0.1765, off_axis=SPOUT_OFF_AXIS,
                                grip_ahead=grip)

    # correct: solve for the flange with the real tool
    right, _ = tool0_from_tcp(glass, side_quat(theta), spout)
    assert tcp_from_tool0(right, side_quat(theta), spout)[0] == approx(glass)

    # wrong: pretend the tip is on the axis, as the old code had to
    dx, dz = old_rotated_offset(0.1765, theta, grip)
    naive = (glass[0] - dx, glass[1], glass[2] - dz)
    landed, _ = tcp_from_tool0(naive, side_quat(theta), spout)
    assert math.dist(landed, glass) == pytest.approx(SPOUT_OFF_AXIS, abs=1e-9)


def test_workcell_spouts_sit_where_they_were_measured():
    """At the taught workcell grasp, each spout is `up` above and `ahead` past the grip point."""
    import yaml
    from bartender_teach.tool_frames import WORKCELL_SPOUT
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, 'config', 'workcell_points.yaml')
    with open(path) as f:
        pose = yaml.safe_load(f)['points']['grab_whiskey']['pose']
    p0, q0 = tuple(pose['xyz']), tuple(pose['quat_xyzw'])
    approach = quat_rotate(q0, (0.0, 0.0, 1.0))
    grip = tuple(p + SIDE_GRIP_AHEAD_OF_TOOL0 * a for p, a in zip(p0, approach))
    for bottle, (up, ahead) in WORKCELL_SPOUT.items():
        tip, _ = tcp_from_tool0(p0, q0, TOOLS[f'workcell_{bottle}'])
        d = [t - g for t, g in zip(tip, grip)]
        # Up is base +z and ahead is the approach direction (base -x) --
        # to within the ~1 degree the taught grasp is off square.
        assert abs(d[2] - up) < 0.002, bottle
        assert abs(sum(c * a for c, a in zip(d, approach)) - ahead) < 0.002, bottle
        assert abs(d[1]) < 0.002, bottle
