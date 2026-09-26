"""The shipped point file must agree with pour_action_server's fallbacks.

pour_action_server keeps a literal for every point it uses AND prefers the
taught file when it names that point. That is deliberate -- it has to come up
on a bare workspace -- but it means there are two copies of every number, and
two copies drift.

This reads both and insists they match, so an untouched workspace behaves
identically whichever path it takes. If a point is deliberately RETAUGHT to a
better value, this test is supposed to fail: update the literal to match, and
the two stay honest.

Deliberately parses pour_action_server as TEXT rather than importing it.
Importing pulls in rclpy, moveit_msgs and the generated action interfaces, so
the test would only run on a built, sourced workspace -- and this is exactly
the check you want to run before building.
"""
import ast
import os
import re

import pytest

from bartender_teach.point_store import PointStore

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(HERE)
SERVER = os.path.join(SRC, 'bartender_pour', 'bartender_pour',
                      'pour_action_server.py')
OPENER = os.path.join(SRC, 'bartender_open', 'bartender_open',
                      'open_action_server.py')
LAYOUT = os.path.join(SRC, 'bartender_open', 'bartender_open', 'layout.py')
POINTS = os.path.join(HERE, 'config', 'taught_points.yaml')

UR = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ARM = UR


def joints_for(prefix):
    return [prefix + j for j in UR]


def fallbacks():
    """Every _taught('name', [literal]) pair in the server, by source text."""
    text = open(SERVER).read()
    found = {}
    for name, literal in re.findall(
            r"_taught\(\s*'([a-z_]+)'\s*,\s*(\[[^\]]*\])\s*\)", text):
        found[name] = ast.literal_eval(literal)
    return found


def open_fallbacks():
    """Return the opener's points: _taught('name', 'prefix', L.CONST).

    Its literals live in layout.py rather than inline, so the constant is
    resolved from there. Same text-not-import reason as above -- layout.py is
    importable without ROS, but open_action_server.py is not, and reading both
    the same way keeps this runnable before a build.
    """
    if not (os.path.exists(OPENER) and os.path.exists(LAYOUT)):
        return {}
    consts = {}
    layout = open(LAYOUT).read()
    for name, literal in re.findall(
            r'^(ARM_[AB]_HOME)\s*=\s*(\[[^\]]*\])', layout, re.M):
        consts[name] = ast.literal_eval(literal)
    # One arm's rest pose may be written as a copy of the other's, which is
    # what it is now that the arms are parallel: ARM_B_HOME = list(ARM_A_HOME).
    # Resolve that rather than skipping it -- an unresolved name drops the
    # point out of `found` below, and the test then passes by checking
    # nothing, which is how arm B's home went unguarded the first time.
    for name, alias in re.findall(
            r'^(ARM_[AB]_HOME)\s*=\s*(?:list\()?(ARM_[AB]_HOME)\)?\s*$',
            layout, re.M):
        if alias in consts:
            consts[name] = list(consts[alias])
    found = {}
    for point, prefix, const in re.findall(
            r"_taught\(\s*'([a-z_]+)'\s*,\s*'([a-z_]*)'\s*,\s*L\.(\w+)\s*\)",
            open(OPENER).read()):
        if const in consts:
            found[point] = (prefix, consts[const])
    return found


def test_the_server_still_uses_taught_points():
    """If this fails the wiring was removed and the rest is meaningless."""
    assert fallbacks(), f'no _taught(...) calls found in {SERVER}'


@pytest.mark.skipif(not os.path.exists(SERVER), reason='bartender_pour absent')
def test_every_fallback_is_in_the_point_file():
    store = PointStore.load(POINTS, missing_ok=False)
    missing = [n for n in fallbacks() if n not in store]
    assert not missing, (
        f'{POINTS} does not name {missing}; the server would silently fall '
        f'back to its literals for those')


@pytest.mark.skipif(not os.path.exists(SERVER), reason='bartender_pour absent')
def test_taught_values_match_the_fallbacks():
    store = PointStore.load(POINTS, missing_ok=False)
    for name, literal in fallbacks().items():
        taught = store.get(name).joints_in_order(ARM)
        assert taught == pytest.approx(literal, abs=1e-9), (
            f"'{name}' differs: point file {taught} vs pour_action_server "
            f'fallback {literal}. If the taught value is the better one, '
            f'update the literal to match.')


# -- the opener's points, which are on both arms -----------------------------

@pytest.mark.skipif(not os.path.exists(OPENER), reason='bartender_open absent')
def test_the_opener_uses_taught_points_too():
    """Both arms' rest poses are teachable, not just the pouring arm's.

    Arm B's home is the one somebody is actually likely to want to nudge: it
    exists only for the two-armed open and was picked by hand to clear the
    counter. If this fails, the wiring was removed and nudging it means
    editing Python again.
    """
    assert open_fallbacks(), f'no _taught(...) calls found in {OPENER}'


@pytest.mark.skipif(not os.path.exists(OPENER), reason='bartender_open absent')
def test_every_opener_point_is_in_the_file_with_the_right_joints():
    store = PointStore.load(POINTS, missing_ok=False)
    for name, (prefix, literal) in open_fallbacks().items():
        assert name in store, (
            f'{POINTS} does not name {name!r}; the opener would silently fall '
            f'back to its literal')
        taught = store.get(name).joints_in_order(joints_for(prefix))
        assert taught == pytest.approx(literal, abs=1e-9), (
            f'{name!r} differs: point file {taught} vs layout.py {literal}')


@pytest.mark.skipif(not os.path.exists(OPENER), reason='bartender_open absent')
def test_each_point_is_tagged_with_the_arm_its_joints_belong_to():
    """A point's group and its joint names must agree.

    They are two statements of the same fact, and the pendant trusts the
    group first. If they disagreed, `goto` would drive one arm to the other's
    configuration -- which for these two arms means folding arm B into the
    worktop.
    """
    store = PointStore.load(POINTS, missing_ok=False)
    groups = {'': 'ur_manipulator', 'b_': 'b_ur_manipulator'}
    for name in store.names():
        point = store.get(name)
        prefix = 'b_' if all(j.startswith('b_') for j in point.joints) else ''
        assert store.group_of(point) == groups[prefix], (
            f'{name!r} is in group {store.group_of(point)!r} but its joints '
            f'({", ".join(sorted(point.joints))}) are arm '
            f'{"B" if prefix else "A"}\'s')
