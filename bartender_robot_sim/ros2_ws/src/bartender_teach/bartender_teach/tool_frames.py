"""Tool centre points, and the transforms that let you control one.

A tool frame is a rigid offset from tool0. Once you name one, you can ask two
questions that are otherwise fiddly:

  - where is the tool tip, given where the flange is?      tcp_from_tool0
  - where must the flange go to put the tip there?         tool0_from_tcp

and do the thing this module exists for:

  - turn the tool WITHOUT moving its tip                   rotate_about_tcp

That last one is the point. A grasped bottle is a tool whose tip is the pour
spout, and a pour is a rotation about that tip: the spout stays over the glass
while the bottle swings around it. Rotating about tool0 instead swings the
spout through an arc 0.2m wide, which is off the glass and onto the counter.

Why this exists as a module rather than more arithmetic in the pour server:
pour_action_server already solved this once, by hand, for one case. Its
_rotated_offset() is a 2x2 rotation with the comment "dy is always zero because
the tilt is about Y" -- correct, and true only for that tilt, that grasp and
that bottle. Anything else (a spout that is off the bottle's axis, a jog about
X in the teach pendant, a second grasp style) needs the general form, and two
implementations of the same transform would disagree eventually. So the
general one lives here and both callers use it.

Conventions: quaternions are (x, y, z, w) throughout, to match geometry_msgs.
Offsets are expressed in tool0's own frame, so they do not change as the arm
moves -- that is what makes them a property of the tool rather than of a pose.
"""
import math
from typing import NamedTuple

IDENTITY = (0.0, 0.0, 0.0, 1.0)


# ---- quaternion and vector helpers ----------------------------------------
# These live here rather than in teach_points because the transforms below are
# the reason they exist; teach_points imports them back.

def quat_mul(a, b):
    """Hamilton product a*b: apply b, then a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    """Return the inverse rotation.

    Only valid for a unit quaternion, which every quaternion here is: they all
    come from quat_about or from a pose.
    """
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def quat_about(axis, angle):
    """Rotation of `angle` radians about `axis` (which must be unit length)."""
    s = math.sin(angle / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


class Tool(NamedTuple):
    """A named point (and orientation) rigidly attached to tool0."""

    name: str
    xyz: tuple = (0.0, 0.0, 0.0)        # offset from tool0, in tool0's frame
    quat_xyzw: tuple = IDENTITY         # tool orientation relative to tool0
    note: str = ''

    @property
    def reach(self):
        """How far the tip stands off the flange, metres."""
        return math.sqrt(sum(c * c for c in self.xyz))


# tool0 itself. Selecting this makes every transform below the identity, so
# the pendant behaves exactly as it did before tool frames existed -- which is
# what makes this safe to add to a tuned system.
TOOL0 = Tool('tool0', note='the flange itself; no offset')


# ---- transforms -----------------------------------------------------------

def tcp_from_tool0(p0, q0, tool):
    """Tool tip pose, given the flange pose. Returns (xyz, quat_xyzw)."""
    return (vadd(p0, quat_rotate(q0, tool.xyz)),
            quat_mul(q0, tool.quat_xyzw))


def tool0_from_tcp(pt, qt, tool):
    """Return the flange pose that puts the tool tip at (pt, qt).

    The inverse of tcp_from_tool0, and what you send to a Cartesian planner,
    which only ever talks about a real link.
    """
    q0 = quat_mul(qt, quat_conj(tool.quat_xyzw))
    return (vsub(pt, quat_rotate(q0, tool.xyz)), q0)


def rotate_about_tcp(p0, q0, tool, delta):
    """Flange pose after rotating by `delta` about the tool tip.

    The tip keeps its position exactly; the flange swings around it. `delta`
    is applied in the BASE frame (pre-multiplied), because that is what
    somebody watching the robot means by "tilt it back" -- post-multiplying
    would turn it about the tool's own axes instead.
    """
    tip, _ = tcp_from_tool0(p0, q0, tool)
    q0n = quat_mul(delta, q0)
    return (vsub(tip, quat_rotate(q0n, tool.xyz)), q0n)


def translate_tcp(p0, q0, tool, delta_xyz):
    """Flange pose after moving the tool tip by `delta_xyz` in the base frame.

    Present for symmetry and for callers that think in tip poses. A pure
    translation moves flange and tip by the same vector whatever the tool is,
    so this cannot depend on `tool` -- and a jog that quietly did depend on it
    would be a bug.
    """
    return (vadd(p0, delta_xyz), q0)


# ---- the tools this robot actually has ------------------------------------
#
# A bottle becomes a tool the moment it is grasped, and its tip is the pour
# spout fitted in its neck (see the pourer block in each models/*/model.sdf).
#
# Deriving the offset, for the SIDE grasp, where side_quat(0) maps
#     tool0 local +Z -> base +X   (the approach direction)
#     tool0 local +X -> base +Y   (the finger-closing axis)
#     tool0 local +Y -> base +Z   (up)
# and the bottle stands upright on base +Z with its spout leaning toward +X:
#
#     the bottle's axis is GRIP_AHEAD_OF_TOOL0 along base +X from tool0, and
#     the grip point on it is level with tool0. Relative to that grip point
#     the spout tip is `above_grip` up (base +Z) and `off_axis` out (base +X).
#     So in base terms the tip is (grip_ahead + off_axis, 0, above_grip) from
#     tool0, which in tool0's own axes is (0, above_grip, grip_ahead+off_axis).
#
# That is all grasped_bottle_tool() does, and writing it out is worth it
# because getting the axis permutation wrong produces a tool frame that is
# plausible, self-consistent and points somewhere the bottle is not.

SIDE_GRIP_AHEAD_OF_TOOL0 = 0.145    # must match pour_action_server

# Spout tip in BOTTLE-local coordinates, from the models. Same pourer part on
# both bottles, so the off-axis lean is shared and only the height differs.
SPOUT_OFF_AXIS = 0.0171
SPOUT_TIP_Z = {'whiskey': 0.2990, 'cola': 0.3040}
GRASP_HEIGHT = {'whiskey': 0.1225, 'cola': 0.060}


def grasped_bottle_tool(name, above_grip, off_axis=SPOUT_OFF_AXIS,
                        grip_ahead=SIDE_GRIP_AHEAD_OF_TOOL0, note=''):
    """Tool frame for the pour spout of a bottle held in the side grasp."""
    return Tool(name, (0.0, above_grip, grip_ahead + off_axis),
                IDENTITY, note)


def bottle_spout(bottle):
    """Build the spout tool for 'whiskey' or 'cola' from the model."""
    if bottle not in SPOUT_TIP_Z:
        raise KeyError(f'no spout geometry for {bottle!r}; '
                       f'known: {", ".join(sorted(SPOUT_TIP_Z))}')
    return grasped_bottle_tool(
        f'{bottle}_spout',
        above_grip=SPOUT_TIP_Z[bottle] - GRASP_HEIGHT[bottle],
        note=f'pour spout of the {bottle} bottle, held in the side grasp')


# ---- the workcell's bottles -------------------------------------------------
#
# The one-arm workcell grasps from the side too, but tool0 is turned
# differently there. At the taught grab_whiskey pose (workcell_points.yaml):
#     tool0 local +X -> base +Z   (up)
#     tool0 local +Y -> base +Y   (along the bottle row)
#     tool0 local +Z -> base -X   (the approach direction, `jog tz`)
# Same Robotiq 2F-85 straight on tool0, so the grip point is the same
# SIDE_GRIP_AHEAD_OF_TOOL0 in front of the flange.
#
# Measured on the real bottles, from the grip point: the spout is `up` above
# it and `ahead` further along tz. There is no model to derive these from,
# unlike the bar's; re-measure here if a bottle or its grasp height changes.
WORKCELL_SPOUT = {           # bottle: (up, ahead) in metres
    'whiskey': (0.070, 0.020),
    'vodka': (0.085, 0.015),
    'gin': (0.065, 0.020),
}


def workcell_spout(bottle, grip_ahead=SIDE_GRIP_AHEAD_OF_TOOL0):
    """Build the spout tool of a workcell bottle held in its taught grasp."""
    up, ahead = WORKCELL_SPOUT[bottle]
    return Tool(f'workcell_{bottle}', (up, 0.0, grip_ahead + ahead), IDENTITY,
                f'workcell: spout of the {bottle} bottle, {up * 1000:.0f}mm '
                f'up and {ahead * 1000:.0f}mm along tz from the grip point')


# Orientation is left as identity on both. Only the tip POSITION is used --
# the pour drives the tilt angle itself, and pinning the tool's orientation
# too would over-constrain it. The transforms carry orientation anyway, so a
# tool that needs it (a driver bit, an angled nozzle) just fills it in.
TOOLS = {
    TOOL0.name: TOOL0,
    'whiskey_spout': bottle_spout('whiskey'),
    'cola_spout': bottle_spout('cola'),
    **{f'workcell_{b}': workcell_spout(b) for b in WORKCELL_SPOUT},
}


def get_tool(name):
    try:
        return TOOLS[name]
    except KeyError:
        raise KeyError(
            f"no tool named '{name}'. Known: {', '.join(sorted(TOOLS))}"
        ) from None
