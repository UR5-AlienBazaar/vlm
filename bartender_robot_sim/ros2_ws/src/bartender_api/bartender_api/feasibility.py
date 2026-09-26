"""POST /can: "could you?", answered from geometry alone.

No motion, no live robot state, no planning-time cost -- everything here
comes from bartender_open.layout, the same source of truth /world and the
skills themselves are built on, so this can never disagree with what the
robot would actually do. See docs/CONTROL_API.md, "2. Feasibility".

ALREADY_OPEN and the rest of the error taxonomy are deliberately NOT
answered here: they depend on live simulator state (has this cap already
come off?), and /can's whole value is that it costs nothing to ask before
touching the robot. Combine this with /world's live "occupied" field for
the rest of the picture.
"""
from bartender_open import layout as L

from .reach import UR5E_MAX_REACH, arms_within_reach, reach_from

KNOWN_VERBS = ('pour', 'open')

# The only capped bottle the scene has today. bartender_open's
# open_action_server refuses anything else with this same wording.
OPENABLE_BOTTLES = ('beer',)

# What pour_action_server's own RECIPE actually pours (bartender_pour is a
# separate ROS package and cannot be imported here, the same reason
# layout.py's own geometry gets restated rather than shared -- see
# test_layout.py, which is what keeps a restated copy honest). 'beer' is a
# real bottle-line station but not an ingredient; geometry alone would say
# yes to pouring it, and that would be a wrong answer this cheaply avoided.
POURABLE_BOTTLES = ('whiskey', 'cola')


def _station_xy(name):
    return L.STATIONS.get(name) if name else None


def _arms_that_can_reach(name):
    """Arms whose approach window covers this BOTTLE station, within reach.

    None if the station does not exist; otherwise a (possibly empty) list.
    APPROACH_WINDOW is specifically about side-grasping a bottle off the
    line with a fixed tool orientation (see its own docstring in
    layout.py) -- it does not apply to the glass, which nothing grips; see
    arms_within_reach (imported from .reach) for that.
    """
    xy = _station_xy(name)
    if xy is None:
        return None
    return [a for a in L.servicing_arms(xy)
            if reach_from(xy, a) <= UR5E_MAX_REACH]


def _reason(code, detail, **measurements):
    r = {'code': code, 'detail': detail}
    if measurements:
        r['measurements'] = measurements
    return r


def _refuse(reasons, alternatives=None):
    out = {'ok': False, 'reasons': reasons}
    if alternatives:
        out['alternatives'] = alternatives
    return out


def can(verb, args=None):
    """Answer {"ok": bool, "reasons": [...], "alternatives": [...]?}."""
    args = args or {}
    if verb not in KNOWN_VERBS:
        return _refuse([_reason(
            'UNKNOWN_STATION',
            f'"{verb}" is not a known verb; try one of '
            f'{", ".join(KNOWN_VERBS)}')])
    if verb == 'pour':
        return _can_pour(args.get('bottle'), args.get('glass'))
    return _can_open(args.get('bottle', 'beer'))


def _can_open(bottle):
    if bottle not in OPENABLE_BOTTLES:
        return _refuse([_reason(
            'UNKNOWN_STATION',
            f'no capped bottle called "{bottle}"; the scene has one, and '
            f'it is "beer"')])
    # Both arms' roles in "open" are fixed by the choreography (arm A
    # holds, arm B works the opener), not chosen per station, so there is
    # no arm-assignment question the way there is for "pour" -- geometry
    # alone says yes. Whether the bottle is ALREADY open is live state,
    # not geometry; see the module docstring.
    return {'ok': True}


def _can_pour(bottle, glass):
    reasons = []
    bottle_xy = _station_xy(bottle)
    if bottle_xy is None:
        reasons.append(_reason('UNKNOWN_STATION', f'no station called "{bottle}"'))
    elif bottle not in POURABLE_BOTTLES:
        reasons.append(_reason(
            'UNKNOWN_STATION',
            f'"{bottle}" is a bottle-line station but not an ingredient; '
            f'try one of {", ".join(POURABLE_BOTTLES)}'))
    glass_xy = _station_xy(glass)
    if glass_xy is None:
        reasons.append(_reason('UNKNOWN_STATION', f'no station called "{glass}"'))
    if reasons:
        return _refuse(reasons)

    bottle_arms = _arms_that_can_reach(bottle)
    glass_arms = arms_within_reach(glass_xy)
    lo, hi = L.APPROACH_WINDOW
    if not bottle_arms:
        reasons.append(_reason(
            'OUT_OF_APPROACH_WINDOW',
            f'{bottle} is at world y={bottle_xy[1]:+.2f}; no arm serves y '
            f'in [{lo:.2f}, {hi:.2f}]',
            station_y=bottle_xy[1], window=[lo, hi]))
    if not glass_arms:
        reasons.append(_reason(
            'OUT_OF_REACH',
            f'{glass} is {min(reach_from(glass_xy, a) for a in ("a", "b")):.3f}m '
            f'from the nearest arm base (limit {UR5E_MAX_REACH:.2f})',
            reach_m=round(min(reach_from(glass_xy, a) for a in ('a', 'b')), 3),
            reach_limit_m=UR5E_MAX_REACH))
    if reasons:
        return _refuse(reasons)

    common = sorted(set(bottle_arms) & set(glass_arms))
    if common:
        return {'ok': True, 'arm': common[0]}

    for arm in sorted(set(bottle_arms) | set(glass_arms)):
        near, far = (bottle, glass) if arm in bottle_arms else (glass, bottle)
        far_xy = glass_xy if far == glass else bottle_xy
        far_reach = reach_from(far_xy, arm)
        reasons.append(_reason(
            'NO_ARM_CAN_DO_BOTH',
            f'arm {arm} reaches {near} but {far} is {far_reach:.3f}m from '
            f'its base (limit {UR5E_MAX_REACH:.2f})',
            reach_m=round(far_reach, 3), reach_limit_m=UR5E_MAX_REACH))
    return _refuse(reasons, alternatives=_pour_alternatives(bottle, glass))


def _pour_alternatives(failed_bottle, glass):
    """Other bottle slots that CAN be poured with this same glass.

    Cheap, because it is the same geometry already computed above, just
    run over every other named slot -- and it is what turns a refusal into
    a next step rather than a dead end.
    """
    glass_xy = _station_xy(glass)
    glass_arms = set(arms_within_reach(glass_xy) if glass_xy else [])
    alternatives = []
    for name in POURABLE_BOTTLES:
        if name == failed_bottle:
            continue
        arms = set(_arms_that_can_reach(name) or [])
        if arms & glass_arms:
            alternatives.append({'verb': 'pour',
                                 'args': {'bottle': name, 'glass': glass}})
    return alternatives
