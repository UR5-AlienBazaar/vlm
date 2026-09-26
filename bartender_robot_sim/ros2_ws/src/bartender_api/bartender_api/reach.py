"""Shared radial-reach check: can an arm's kinematic chain get to xy at all.

Deliberately NOT bartender_open.layout.servicing_arms(). That function
answers a narrower question -- can this xy be SIDE-GRASPED, with the fixed
tool orientation the bottle line is picked up with (see APPROACH_WINDOW's
own docstring in layout.py) -- and it says no arm can reach the glass or
the opener holster, because nothing side-grasps either of them: a bottle
is tilted over the glass, and the opener is picked up with a vertical
descent. Both world.py's `reachable_by` and feasibility.py's /can use this
module for exactly those stations, and layout.servicing_arms() for
stations that really are side-grasped off the line.
"""
import math

from bartender_open import layout as L

# UR5e published max reach, base to a fully extended tool at zero payload.
# Not measured in this project's own sim like everything in layout.py is --
# it is the robot's datasheet figure, used only as a sanity bound before a
# plan is even attempted.
UR5E_MAX_REACH = 0.850


def reach_from(xy, arm):
    """Straight-line distance from `arm`'s base to world (x, y)."""
    origin = L.ARM_A_ORIGIN if arm == 'a' else L.ARM_B_ORIGIN
    return math.hypot(xy[0] - origin[0], xy[1] - origin[1])


def arms_within_reach(xy):
    """Every arm ('a', 'b') whose base is within UR5e reach of xy."""
    return [a for a in ('a', 'b') if reach_from(xy, a) <= UR5E_MAX_REACH]
