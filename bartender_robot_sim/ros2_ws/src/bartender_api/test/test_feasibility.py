"""Tests for POST /can's geometry, over the real bartender_open.layout.

The one bug this caught before it shipped: the glass is gripped by
nothing (a bottle is tilted OVER it), so testing its reachability with
layout.servicing_arms -- built for a fixed side-grasp tool orientation on
the bottle line -- said no arm could reach a glass the real, working pour
skill reaches every day. Radial reach is the right test for a vessel nothing
grips; the approach window is the right test for a bottle something does.
Both directions are covered below so a reintroduction of that mix-up fails
a test instead of shipping.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_open'))

from bartender_api import feasibility as F                # noqa: E402
from bartender_open import layout as L                     # noqa: E402


def test_whiskey_into_the_glass_is_feasible():
    """The real, working pour -- if this says no, the check is wrong."""
    result = F.can('pour', {'bottle': 'whiskey', 'glass': 'glass'})
    assert result['ok'] is True
    assert result['arm'] == 'a'


def test_cola_into_the_glass_is_feasible():
    result = F.can('pour', {'bottle': 'cola', 'glass': 'glass'})
    assert result['ok'] is True


def test_the_glass_is_reachable_by_radial_distance_not_approach_window():
    """Regression test for the bug this file's docstring describes.

    The glass sits at an arm-frame y well outside APPROACH_WINDOW (it is
    placed there deliberately, to clear the pour's sweep -- see
    STATION_GLASS in layout.py) but well within radial reach. A check that
    applies the approach window to the glass fails every pour; one that
    checks radial distance passes the ones that actually work.
    """
    glass_xy = L.STATIONS['glass']
    in_a = L.to_arm((glass_xy[0], glass_xy[1], L.COUNTER_Z),
                    L.ARM_A_ORIGIN, L.ARM_A_YAW)
    lo, hi = L.APPROACH_WINDOW
    assert not (lo <= in_a[1] <= hi), (
        'the glass moved inside APPROACH_WINDOW; this test no longer '
        'exercises the bug it was written for')
    assert F.can('pour', {'bottle': 'whiskey', 'glass': 'glass'})['ok']


def test_an_unknown_bottle_is_refused_by_name():
    result = F.can('pour', {'bottle': 'gin', 'glass': 'glass'})
    assert result['ok'] is False
    assert result['reasons'][0]['code'] == 'UNKNOWN_STATION'


def test_beer_is_a_station_but_not_an_ingredient():
    """Geometry alone would say yes; the recipe says no. Recipe wins.

    Mirrors pour_action_server.RECIPE = (WHISKEY, COLA) -- restated here,
    not imported, for the same reason layout.py's own geometry is restated
    rather than shared (bartender_pour is a separate ROS package). If that
    tuple ever grows, this constant has to be updated by hand; nothing
    catches drift automatically the way test_layout.py does for geometry.
    """
    result = F.can('pour', {'bottle': 'beer', 'glass': 'glass'})
    assert result['ok'] is False
    assert result['reasons'][0]['code'] == 'UNKNOWN_STATION'
    assert 'not an ingredient' in result['reasons'][0]['detail']


def test_opening_the_beer_is_feasible():
    assert F.can('open', {'bottle': 'beer'}) == {'ok': True}


def test_opening_anything_else_is_refused():
    result = F.can('open', {'bottle': 'whiskey'})
    assert result['ok'] is False
    assert result['reasons'][0]['code'] == 'UNKNOWN_STATION'


def test_an_unknown_verb_is_refused_not_a_crash():
    result = F.can('mix', {})
    assert result['ok'] is False
    assert 'mix' in result['reasons'][0]['detail']


def test_missing_args_do_not_crash():
    """POST /can with no args at all is a malformed-ish request, not a 500."""
    assert F.can('pour', None)['ok'] is False
    assert F.can('open', None) == {'ok': True}  # 'open' defaults bottle='beer'


def test_a_station_out_of_reach_gets_out_of_reach_not_approach_window():
    """The glass's own failure mode is a reach distance, not a window.

    Exercised directly against a station placed far enough out that no
    arm's radial reach covers it, using the real UR5E_MAX_REACH bound
    rather than a made-up one.
    """
    far_xy = (0.20, -0.55 - 2 * F.UR5E_MAX_REACH)
    reach_a = F.reach_from(far_xy, 'a')
    assert reach_a > F.UR5E_MAX_REACH, 'test station is not actually far enough'
    arms = F.arms_within_reach(far_xy)
    assert arms == []
