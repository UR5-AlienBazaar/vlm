"""PourDrink action server: pick, carry, tilt, pour, return.

Run once per bottle in RECIPE, for Phase 1 (known bottles, one known glass,
no perception).

Pours a whiskey and coke: the spirit first, then the mixer, both into the one
serving glass. Adding a bottle means adding a Bottle entry to RECIPE -- its
position, height, where on it to grip, and how far to squeeze -- and nothing
else; every pose in the cycle is derived from those.

STATUS (default side grasp). Measured, one whiskey-and-coke per run:

    bottles        pads      pourer   drinks   whiskey returned
    1.0/0.8kg      mu 1.2    no        3/3          3/3
    1.35/1.05kg    mu 1.2    no        3/6          4/7
    1.35/1.05kg    mu 1.8    no        7/7          6/7
    1.35/1.05kg    mu 1.8    YES       4/8          8/8

The last row is the speed pourer being fitted, and it is a genuine drop in
completed drinks that is worth understanding before anyone "fixes" it.

The whiskey half is unaffected and in fact perfect: 8/8 back on its station.
Every one of the four failures is the same event, the cola slipping out
mid-cycle and the grip check catching it ("cola lost: fingers have closed to
0.4400").

That is NOT a new fault. The cola has been thrown during the place-and-release
in almost every run of every configuration above, including the original one
-- tracing a passing run shows it held correctly through the whole tilt (90.7
degrees) and the pour, then flung to the floor on release. What the pourer
changed is only WHEN the slip happens: earlier, so the grip check now trips
before the drink is finished rather than after. The underlying weakness, and
the number of bottles left on the floor, are the same as they were.

So the thing to fix is still the cola's place-and-release, not the pourer, and
not the pour geometry -- the spout holds the glass axis to 0.00mm through the
entire tilt, which is checked numerically. Nothing here checks where a bottle
ENDS UP, which is why this was easy to miss for so long.

KNOWN BAD: the cola's place-and-release, as above. It is the largest remaining
defect in the default path and the obvious next piece of work.

STATUS (BARTENDER_GRASP=hook): works end to end and pours real drinks, but
only 1-2 runs in 6, so it is NOT the default. What was measured:

  - Picking by the neck is excellent on its own: 4/4 lifts, 1 degree of tilt,
    2-8mm of drift, and a control with the collar removed lifted the bottle
    0mm in 3/3 -- so the ledge really is carrying it and friction contributes
    nothing.
  - Pouring works too. The tilt does NOT shake the bottle off the ledge, which
    was the worry: traced through a full 97 degree pour, the knuckle holds
    steady and the bottle comes back upright.
  - What it costs is ENTRY CLEARANCE, and that is the thing to fix next. The
    collar has to pass between the two tips on the run-in, and the opening is
    85mm less twice the tip's protrusion. A tip deep enough to swallow a 42mm
    collar leaves 43-47mm of opening, i.e. 0.5-2.5mm a side, which is the same
    order as the placement error. Most failures are the run-in clipping a
    bottle rather than the grasp or the pour.

  The fix is a design change, not tuning: make the ledge the TOP FACE of the
  tip instead of a step partway up it. Then the collar never passes between
  the tips at all -- approach around the bare neck, close, and lift until the
  collar lands on the tip. The tip only has to swallow the 33mm neck, which
  needs far less depth and leaves far more stroke.

Motion strategy
---------------
Gross transit between open-space configurations goes through MoveIt's
/move_action (OMPL/RRTConnect). Everything that happens near the bottle or
the glass goes through /compute_cartesian_path + /execute_trajectory instead.

That split is deliberate, and so is the fact that each goal starts by pushing
the counter, bottle and glass into the planning scene as CollisionObjects.
They exist as Gazebo models, which MoveIt cannot see; until they were
published, a joint-space plan between two poses either side of the bottle was
free to sweep the arm straight through it, and RRTConnect being randomised it
did exactly that -- measured, 6 transits out of 6 knocked the bottle over
before the grasp had even begun.

The Cartesian segments still run with avoid_collisions=False, because the
grasp itself has to reach inside that envelope and touch the bottle. What
keeps those segments safe is that they are straight lines, computed in one
shot, not a randomised search. So: the planning scene protects the gross
transits, and the straight lines protect the close work.

Grasping
--------
Bottles are grasped from the SIDE, around the body, near their centre of mass
-- not from above by the neck. Gripping the neck works mechanically, but it
puts the pads about 15mm below the mouth, so a pour runs straight over the
fingertips. Gripping the body puts the mouth 0.12-0.14m clear of the pads,
which is also simply how you hold a bottle to pour it.

That costs clearance. Analytic FK over the Robotiq chain (joint origins from
the URDF, pad face from left_finger_tip.stl) gives a pad-to-pad opening of
85.0mm at 0 rad, closing roughly linearly to 0 at the 0.8 rad limit:

                      width   clearance   pads meet it at
    whiskey body     77.2mm      3.9mm/side     0.086 rad
    cola body        67.2mm      8.8mm/side     0.190 rad
    (whiskey neck)   33.0mm                     0.510 rad

The same model puts contact with the 33mm neck at 0.51 rad, which is exactly
where the old neck grasp was measured to clamp -- that agreement is why these
numbers are trusted.

The whiskey bottle is square, so its clearance depends on yaw: a square 77.2mm
across the flats presents 77.2*(cos y + sin y), which reaches the pads' 85.0mm
at just over 6 degrees, beyond which the grasp is geometrically impossible.
Phase 1 has no perception, so it assumes the spawn yaw of zero and relies on
placing the bottle back down cleanly; measured yaw after a full cycle is ~0.1
degrees. The cola bottle is round and has no such limit. Anything that
disturbs the whiskey between runs will break the next grasp, and Phase 2 will
have to measure yaw rather than assume it.

The whiskey used to creep ~6mm further from the robot on every cycle, so that
by the fifth run only the edge of the pad still reached it and the grasp
failed. That turned out to be an artefact of DART's default FCL narrowphase
rather than anything about the grasp: with the bullet collision detector (see
bar_world.sdf) six consecutive cycles hold the bottle within 5mm of its
station with no trend. Since everything here works from a hardcoded bottle
pose and nothing measures where the bottle actually is, that margin is worth
re-checking after any change to the gripper, the bottle model or the physics
settings. The grip checks below make a failure honest rather than silent.

The closing command is intentionally past the body surface and is NOT waited
on. A clamping goal can never satisfy goal_tolerance (the commanded angle is
inside the object), so it legitimately stays active -- and staying active is
exactly what keeps the squeeze on. Waiting for it to finish would either hang
or, with a short stall_timeout, latch the hold position at first contact and
drop the bottle. See the gripper_controller notes in
bartender_description/config/controllers.yaml.

Threading
---------
Nested action calls (this server calling the MoveGroup/ExecuteTrajectory and
GripperCommand action clients) require a MultiThreadedExecutor with a
ReentrantCallbackGroup -- see main(). They block on a plain threading.Event
rather than rclpy.spin_until_future_complete(): that call grabs rclpy's
process-global executor internally, which conflicts with this node already
being spun by our own MultiThreadedExecutor (a node must only ever be
associated with one executor) and deadlocks the nested call. Blocking a
worker thread on an Event is safe instead, since the executor's other threads
stay free to service the nested client's response/result callbacks.
"""
import math
import os
import threading
import time
from typing import NamedTuple

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject, Constraints, JointConstraint, MotionPlanRequest,
    PlanningOptions, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from bartender_pour_interfaces.action import PourDrink
from bartender_teach.point_store import (
    PointStore, PointStoreError, default_points_path,
)
from bartender_teach.tool_frames import (
    Tool, grasped_bottle_tool, tool0_from_tcp,
)

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

ARM_GROUP_NAME = 'ur_manipulator'
EEF_LINK = 'tool0'
PLANNING_FRAME = 'base_link'
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'

# Object positions in base_link. Arm A is spawned at world (-0.45, -0.40,
# 0.9) with no rotation, so base_link x = world x + 0.45, base_link
# y = world y + 0.40, and base_link z = world z - 0.9.
#
# The bar's layout is decided in bartender_open/layout.py -- a bottle line
# at world x = 0.08 with five slots at a 0.15 pitch, and a working station
# for each arm off it -- and restated here in arm A's frame because this
# cannot import that one. bartender_open/test_layout.py parses this file and
# checks the three numbers below against it, so they cannot drift.
#
# The bottle line lands at base_link x = 0.53 for every slot, which is why
# WHISKEY and COLA share an x and differ only in y.
#
# The glass is NOT on the line and not on the arm's centreline either. It is
# at base_link (0.65, -0.15) -- further out than the line, and offset in y
# so that the pour, which lays the bottle back 0.30 behind the glass at
# bottle height, sweeps across a part of the line where there is no slot.
# layout.pour_sweep_clearance() is the check; it is 0.25 against the 0.095
# of bottle envelope that has to fit in it.
GLASS_XY = (0.65, -0.15)
# serving_glass is a 0.09-tall, 0.04-radius cylinder standing on the counter,
# so its rim is at GLASS_HEIGHT and everything carried over it must clear that.
GLASS_HEIGHT = 0.09

# ---------------------------------------------------------------------------
# Taught points.
#
# Every configuration below started life as a literal obtained by running
# /compute_ik by hand, reading a branch off the console, normalising it and
# pasting it in. `ros2 run bartender_teach teach` does that job by driving the
# arm there and naming it, and writes the result to a YAML file -- so if that
# file names a point, it wins over the literal here.
#
# The literals are KEPT, not deleted, and they are what runs if the file is
# missing, unreadable, or does not mention the point. This server has to come
# up on a bare workspace, and a point file is an editable text file that a
# person can get wrong; falling back to a value known to work beats refusing
# to start. The shipped point file is seeded with these exact numbers, so on
# an untouched workspace taking either path gives the same motion.
#
# Which source won is recorded here and logged once at startup, because
# "the arm went somewhere different after I taught a point" should be
# answerable without reading either file.
POINT_SOURCES = {}


def _taught(name, fallback):
    """Joint list for `name` from the taught-point file, else `fallback`."""
    try:
        point = _POINTS.get(name)
    except PointStoreError:
        POINT_SOURCES[name] = 'built-in (not in the point file)'
        return fallback
    try:
        joints = point.joints_in_order(ARM_JOINTS)
    except PointStoreError as exc:
        # An incomplete point is a broken point. Say so loudly and keep going
        # on the literal rather than driving a partly-specified pose.
        POINT_SOURCES[name] = f'built-in (taught point unusable: {exc})'
        return fallback
    POINT_SOURCES[name] = f'taught, from {_POINTS.path}'
    return joints


try:
    _POINTS = PointStore.load(default_points_path())
except PointStoreError as _exc:
    _POINTS = PointStore(default_points_path())
    POINT_SOURCES['*'] = f'point file unreadable, using built-ins: {_exc}'

HOME_JOINTS = _taught('home', [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

# How far along tool0 +Z the grasped bottle's axis sits. This is NOT a free
# choice and it is NOT the same number as the old neck grasp used.
#
# The 2F-85 is narrower BEHIND its pads than at them: sweeping the whole
# gripper's collision meshes against a 77.2mm-wide slab shows the pads clear it
# by 3.9mm a side, but the inner knuckle links reach in to 29.8mm and the palm
# to 6.7mm. A 33mm neck fits deep in the throat of the gripper; a 77mm body has
# to be held out near the fingertips or the linkage hits it first. Measured
# clearance of everything except the pads, against the 0.0386 half-width:
#
#   depth   0.1205   0.1380   0.1400   0.1450   0.1520
#   open    0.0067   0.0380   0.0453   0.0453   0.0453
#   closing 0.0067   0.0344   0.0344   0.0409   0.0442
#
# 0.1205 was tried first (copied from the neck grasp) and the inner knuckles
# knocked the bottle flat on the run-in before the fingers ever closed.
# 0.1450 keeps 6.7mm of margin on the linkage while still laying 43mm of the
# 57mm pad face on the bottle.
#
# BARTENDER_GRASP=hook switches all of this to the neck-hooking fingertips in
# ../../../../../fingertip (build the robot with BARTENDER_PAD_MODE=hook to
# match). Those carry the bottle on a ledge under its collar instead of on pad
# friction, so the geometry is completely different: the grip is at the COLLAR
# rather than mid-body, and the tool sits 0.1386 from the bottle's axis because
# that is where the hook's pocket axis falls, measured by FK along the knuckle
# chain and confirmed in sim (the knuckle stalls at 0.288-0.303 on a 33mm neck
# against a predicted 0.268).
GRASP_STYLE = os.environ.get('BARTENDER_GRASP', 'side')

GRIP_AHEAD_OF_TOOL0 = 0.1386 if GRASP_STYLE == 'hook' else 0.145

# Straight-line run-in along +X. The whiskey's pads clear its body by only
# 3.9mm a side, which is less than the Cartesian error a joint-space goal at
# JOINT_TOLERANCE can leave behind, so the arm descends to an exact Cartesian
# pose beside the bottle first and only then translates in. Going straight from
# the joint waypoint into the grasp means starting the run-in a few mm
# off-centre and sweeping a fingertip through the bottle on the way.
APPROACH_BACKOFF = 0.10
# Height the transit holds before dropping to grasp height: clear of both
# bottles' shoulders and well out of the planner's way.
APPROACH_Z = 0.30

# Heights are specified at the POUR SPOUT, because that is the part with
# somewhere it has to be: clear of the glass rim while carrying, just above it
# to pour. It used to be the bottle's bare mouth; both bottles now carry a
# speed pourer (see the pourer block in models/*/model.sdf) whose tip is the
# point liquid actually leaves from, and it is neither the mouth nor on the
# bottle's axis.
#
# SPOUT_ABOVE_MOUTH is the same on both bottles because it is the same part,
# so CARRY_SPOUT_Z below works out to the identical tool0 height the mouth
# formulation gave -- this change moves the POUR point without moving the
# carry.
SPOUT_ABOVE_MOUTH = 0.054    # pourer tip above the bottle's lip, along its axis
SPOUT_OFF_AXIS = 0.0171      # ...and to the side of it, toward bottle-local +X
# Height of the carried bottle's MOUTH while it crosses the bar, and with
# it the whole transit: the lift ends here and tilt_pose(0) is at the same
# height, so a bottle travels from its slot to the glass on the level.
#
# 0.61, raised from 0.40 when the bar was rebuilt around a bottle line.
#
# 0.40 only had to clear the glass, because on the old counter the bottles
# stood apart and a carry never passed over one. In a line they are 0.15
# apart and the glass is off to the side, so every carry crosses the line:
# measured on this layout, the cola's diagonal to the glass passes 59mm from
# the standing whiskey in plan view, against the 95mm of bottle envelope
# that would have to fit there. Going round is not available -- the glass
# cannot be moved far enough sideways without leaving the arm's reach -- so
# the carry goes over instead.
#
# The number is derived rather than picked. The tallest thing standing on
# the counter is the cola with its pour spout at 0.3055, the carried bottle
# hangs below its own mouth by its full height, and 0.05 is the same margin
# APPROACH_Z leaves for a joint-space move to overshoot by:
#
#     CARRY_MOUTH_Z >= 0.3055 + 0.05 + 0.250 (the taller bottle) = 0.6055
#
# That leaves the whiskey's base riding at 0.365 and the cola's at 0.360,
# both clear of anything they pass. It costs reach and was checked for it:
# the furthest the flange gets is 0.70 from the shoulder, over the glass.
CARRY_MOUTH_Z = 0.61
CARRY_SPOUT_Z = CARRY_MOUTH_Z + SPOUT_ABOVE_MOUTH
POUR_SPOUT_Z = 0.14          # 50mm above the rim

# NOT 0.0, AND THE DIFFERENCE MATTERS. robotiq_85_left_knuckle_joint is
# revolute with limit=[0.0, 0.8], and a knuckle left at rest ON the lower
# limit stops responding to gripper commands for the rest of the simulator
# run -- goals still accepted, controller still active, joint never moves
# again. Measured: parked at 0.000 for 10s it was already dead, while 0.020
# survived 300s and 0.300 survived 90s; the upper limit is harmless. The
# measurements and everything that was ruled out are written up beside
# GRIPPER_LOWER_LIMIT in bartender_open/arm.py, which carries the same
# constant. Both have to move together.
#
# WHAT THE MARGIN COSTS HERE, which is not nothing and should not be
# written off as if it were. The pads open to 83.2mm at 0.02 rad instead of
# 85.0 at 0.0, and the whiskey's 77.2mm flats are the tightest thing this
# robot handles: the run-in clearance goes from 3.9mm a side to 3.0mm. That
# is a 23% cut in the narrowest tolerance in the system, and it is accepted
# because the alternative is a gripper that dies outright on roughly half
# of runs.
#
# CHECKED, and only as far as this: two whiskey grasps on one run clamped
# at 0.0899 and 0.0888 rad against the 0.089 this file has always recorded,
# and both poured and released cleanly. So the narrower opening does not
# move where the pads meet the bottle. The same run's COLA half failed
# both times, to the slip this file's header already documents ("cola
# lost: fingers have closed to 0.4400"), so it says nothing either way --
# and a full whiskey and coke has NOT been completed since this changed.
#
# If that clearance ever becomes the binding constraint, the thing to do is
# measure whether a smaller margin (0.01, 0.005) also survives a long dwell
# -- neither has been tested -- not to quietly put this back to 0.0.
GRIPPER_LOWER_LIMIT = 0.0       # from the URDF; do not command it
GRIPPER_LIMIT_MARGIN = 0.02
GRIPPER_OPEN_POS = GRIPPER_LOWER_LIMIT + GRIPPER_LIMIT_MARGIN
GRIPPER_UPPER_LIMIT = 0.8       # from the URDF; safe to sit on, unlike 0.0
GRIPPER_MAX_EFFORT = 100.0
# Long enough for the fingers to stop against the bottle before the stall
# angle is read back. These waits are wall-clock, so they mattered little when
# the simulator was running at RTF 0.1; now that it keeps up they are a real
# share of the cycle, and were trimmed (settle 2.5 -> 1.0, dwell 0.2 -> 0.1)
# and re-verified rather than left at values chosen when they were free.
GRIPPER_SETTLE_S = 1.0
# Close in increments rather than one jump to the clamp angle. A single goal
# makes the controller drive the full commanded step at once, and the pads
# arrive with enough momentum to knock the bottle over before they can pinch
# it -- measured: one-shot close topples it to 73 deg every time, while the
# same close walked up in 0.02 rad steps stalls cleanly on the body at 0.089
# and leaves the bottle upright and gripped. Each increment preempts the last;
# only the final one is left active, to hold the squeeze.
GRIPPER_CLOSE_STEP = 0.02
GRIPPER_STEP_DWELL_S = 0.1
# Opening has to be walked down for the same reason. Letting a loaded grip go
# in one command turns the contact that was holding the bottle into a shove:
# measured, the bottle is placed upright and correctly, then flung 42mm
# forward and onto its side as the pads spring apart. The settle afterwards is
# so the retreat does not start while it is still rocking.
GRIPPER_RELEASE_SETTLE_S = 0.5
# A closing goal that reaches its commanded angle means nothing was between
# the fingers. A real grasp stalls the joint short of the command.
GRASP_STALL_MARGIN = 0.05

# Pour tilt about the base_link Y axis -- which, in the side grasp, is the
# finger-closing axis, so the bottle tips in the plane containing its own axis
# and the approach direction, away from the gripper. The mouth is held over the
# glass throughout and the pads trail 0.12m behind it.
#
# The side grasp is also far easier on reach than the old overhead one: tool0
# stays 0.47-0.54m from base_link across the whole tilt, against the UR5e's
# 0.85m limit. The overhead grasp ran to 0.834m and could not complete more
# than about half a tilt at any subdivision. That headroom is why this can go
# past vertical, which is what actually pours.
POUR_TILT_RAD = 1.7          # ~97 deg
TILT_WAYPOINTS = 16


class Bottle(NamedTuple):
    """Everything the pick-pour-return cycle needs to know about one bottle.

    Only the grip DEPTH along tool0 (GRIP_AHEAD_OF_TOOL0) is shared, because
    that is a property of the gripper rather than of what it is holding: 0.145
    is inside the safe window for both bottles.
    """

    name: str
    xy: tuple            # bottle axis in base_link
    height: float
    grasp_height: float  # up from the bottle's base
    envelope_radius: float   # conservative proxy for the planning scene
    clamp_pos: float     # knuckle angle to command (past first contact)
    # Joint-space pose to transit to, held APPROACH_Z above the grasp rather
    # than beside it. Down at grasp height the gripper sits in a pocket ~41mm
    # from the bottle and RRTConnect kept returning paths that grazed it and
    # were thrown out after time-parameterisation ("Motion plan was found but
    # it seems to be invalid"); from up here the planner has room, and the two
    # segments that actually approach the bottle are straight lines.
    #
    # Every joint is normalised into [-pi, pi]. /compute_ik hands back
    # arbitrary branches, often out near +/-2pi, which describe the same pose
    # but force long wrist sweeps that drag the gripper through forearm_link
    # and get rejected as self-collisions. Of the branches that reach each
    # pose, these are the ones that also keep every link above the counter:
    # rejected alternatives put wrist_1 23mm above the worktop and a forearm
    # 166mm BELOW it, neither of which the planner would warn about.
    approach_joints: list
    # Outer radius of the well this bottle is returned into, or 0.0 for a
    # bottle that stands free. Only the planning scene uses it: the stand is
    # 26mm tall and every Cartesian segment near the counter is already
    # running with avoid_collisions=False, so this constrains the joint-space
    # transits and nothing else. It comes from make_bottle_stands.py's
    # outer_radius(), and it is WIDER than envelope_radius -- a bottle
    # envelope alone does not describe the thing now standing around its base.
    stand_radius: float = 0.0

    @property
    def mouth_above_grip(self):
        return self.height - self.grasp_height

    @property
    def grasp_x(self):
        return self.xy[0] - GRIP_AHEAD_OF_TOOL0

    @property
    def approach_x(self):
        return self.grasp_x - APPROACH_BACKOFF

    @property
    def carry_z(self):
        return CARRY_MOUTH_Z - self.mouth_above_grip

    @property
    def standing_height(self):
        """How tall the bottle actually is with its pourer fitted.

        `height` is the glass bottle alone, and it is what the grasp geometry
        is built on, so it stays as it is. But the thing standing on the
        counter is 54mm taller than that, and the planning scene has to be
        told the real number or MoveIt will happily route a transit through
        the pourer -- which it cannot see, and which is exactly the class of
        bug the planning scene was added to prevent in the first place.
        """
        return self.height + SPOUT_ABOVE_MOUTH

    @property
    def spout_above_grip(self):
        """Pourer tip up the bottle's axis from the grip point."""
        return self.height + SPOUT_ABOVE_MOUTH - self.grasp_height

    @property
    def spout_tool(self):
        """Give the grasped bottle as a tool, tipped at its pour spout."""
        return _axis_point_tool(f'{self.name}_spout', self.spout_above_grip,
                                SPOUT_OFF_AXIS)


# Grip at mid-height, where model.sdf puts the centre of mass, so the bottle is
# balanced in the fingers and the pour develops almost no torque about the
# grip. Square in cross-section, so its envelope uses the circumscribed radius
# and the grasp is yaw-sensitive (see the module docstring). 77.2mm across the
# flats means the pads meet it at ~0.086 rad, so the clamp angle less 0.086 is
# the squeeze.
#
# 0.25, i.e. ~0.16 rad of squeeze past contact. When the bottle was made
# heavier (1.0 -> 1.35kg, see its model.sdf) this was deliberately NOT raised
# to match, and the reason is worth keeping.
#
# The heavier bottle does fail differently. At the 97 degree pour angle
# gravity acts along the pad faces, so friction carries the whole weight --
# 13.2N against the 9.8N that 1.0kg needed -- and it was seen to slip out
# mid-pour, the grip check finding the fingers closed to the full commanded
# 0.2500 with the bottle on the floor at 118 degrees.
#
# The obvious response is to squeeze proportionally harder, clamp
# 0.086 + 0.164*1.35 = 0.31. That is the wrong lever. The knuckle already
# stalls on the flats within 0.2mm, so a higher command does not close the
# fingers any further; it only raises the position error and the effort the
# controller pushes into a contact that has already bottomed out. Commanded
# angle past contact is not a grip-force dial, and driving it harder feeds
# DART a deeper penetration to resolve.
#
# Shear capacity is mu times the normal force, and mu was the half of that
# which had never been revisited for a heavier bottle. So PAD_MU carries the
# extra weight instead -- see the notes in
# bartender_description/scripts/render_bartender_urdf.py, which record what
# each setting measured.
WHISKEY = Bottle(
    name='whiskey',
    xy=(0.53, 0.10),
    height=0.245,
    grasp_height=0.1225,
    envelope_radius=0.0546,      # 0.0386 * sqrt(2), the square's corners
    clamp_pos=0.25,
    approach_joints=_taught(
        'whiskey_approach',
        [-0.1905, -2.1401, 2.1357, 0.0044, 1.3803, 0.0000]),
    stand_radius=0.0740,
)

# The cola bottle is round, so nothing here depends on its yaw. It is gripped
# at its WAIST (z=0.060, 59.6mm across) rather than at the wider bulge, which
# is what makes the grasp work at all.
#
# Gripping the 67.2mm bulge was tried first and failed in a way no clamp angle
# fixed. A smooth cylinder between two flat pads is a badly conditioned
# contact: the joint needs ~6mm of pad penetration before it stalls, where the
# whiskey's flat faces stall within 0.2mm. Squeeze harder and the bottle rolls
# out sideways and is flung off the counter (traced: spun 111 degrees, ended
# on the floor); squeeze less and it slips back out during the lift. Measured
# across four runs each, clamp 0.30 / 0.35 / 0.42 all lost the bottle.
#
# At the waist the pads have 13mm of clearance a side instead of 9mm, and the
# 67mm sections above and below are stops the closed pads cannot pass, so slip
# is bounded by geometry at ~6mm rather than by friction. Contact comes at
# ~0.265 rad and the joint settles at 0.27-0.29, so penetration is down to
# 0.005-0.025 from the 0.065 the bulge grip needed -- a far better conditioned
# contact.
#
# clamp_pos has to be read against where the joint actually STALLS, which is
# set by geometry, not by this number: this only says how hard to keep
# squeezing once the bottle has stopped the fingers. On the stock flat pads
# the stall is ~0.28, so 0.44 both squeezes and stays clear of
# GRASP_STALL_MARGIN.
#
# If you switch the gripper to the grooved pads (BARTENDER_PAD_MODE=boxes, see
# bartender_description/scripts/render_bartender_urdf.py) this MUST become
# 0.55. Seating into the groove closes the fingers about 5mm further and moves
# the stall to a measured 0.369-0.393, and |0.44 - 0.39| is exactly
# GRASP_STALL_MARGIN -- so good grasps get condemned mid-pour as
# "checking_grip_cola". The reverse is just as bad: 0.55 on flat pads
# over-squeezes and throws the bottle out of the fingers.
#
# Configurations tried and measured, full drinks completed. On FLAT pads:
# bulge grip at clamp 0.30 -> 1/4, 0.35 -> 0/4, 0.42 -> 3/6, bulge with the
# bottle's friction raised to 1.5 -> 0/5, waist grip at 0.36 -> 1/5, at
# 0.44 -> 2/5. With the grooved pads, see the STATUS block at the top.
#
# Gripping this low does cost something: the mouth is then 0.19 above the pads
# (further from the pour, which is good) but the centre of mass is 0.05 ABOVE
# the grip rather than at it, so unlike the whiskey this one is not balanced
# in the fingers.
COLA = Bottle(
    name='cola',
    xy=(0.53, 0.25),
    height=0.250,
    grasp_height=0.060,
    envelope_radius=0.0400,
    clamp_pos=0.44,
    approach_joints=_taught(
        'cola_approach',
        [0.4901, -1.7912, 2.0955, -0.3043, 2.0609, 0.0000]),
    stand_radius=0.0560,
)

# Poured in this order, into the one glass: spirit first, mixer on top, which
# is both how the drink is made and the order that keeps the heavier bottle's
# transit away from a glass that is already full.
# ---------------------------------------------------------------------------
# Neck-hook variants of the same two bottles.
#
# grasp_height is the point on the bottle's axis level with tool0. The hook's
# ledge sits 1.8mm below tool0, and it is set 3mm under the collar's underside
# so that the lift brings the two into contact rather than pinching the collar
# on the way in. So
#     grasp_height = collar_underside - 3mm + 1.8mm
# Collar undersides come from the bottle models: 0.232 on the whiskey,
# 0.237 on the cola.
#
# clamp_pos has to clear the stall by more than GRASP_STALL_MARGIN, or the
# grip check reads a perfectly good grasp as an empty gripper. Read against
# the LOADED stall, not the free one, and the difference is what matters here:
# the pads meet a 33mm neck at 0.288-0.303, but once the ledge takes the
# bottle's weight the linkage closes further and the whiskey settles at
# 0.38-0.39.
#
# So 0.32 fails immediately ("nothing is between them"), and 0.42 fails
# INTERMITTENTLY, which is worse: |0.42 - 0.389| = 0.031 is inside the margin,
# so whether a good grasp is condemned depends on exactly when the check
# samples. That is what made this look flaky rather than wrong. 0.55 clears
# the loaded value by 0.16 while still squeezing.
#
# approach_joints are reused from the side grasp. They only have to put the
# arm near the pre-grasp before a Cartesian move takes over, and moving the
# grip from mid-body to the collar shifts that pose by 6mm in x.
HOOK_LEDGE_BELOW_TOOL0 = 0.0018
HOOK_SET_UNDER_COLLAR = 0.003
HOOK_CLAMP = 0.55


def _hook(bottle: Bottle, collar_underside: float) -> Bottle:
    return bottle._replace(
        grasp_height=collar_underside - HOOK_SET_UNDER_COLLAR
        + HOOK_LEDGE_BELOW_TOOL0,
        clamp_pos=HOOK_CLAMP,
    )


if GRASP_STYLE == 'hook':
    WHISKEY = _hook(WHISKEY, 0.232)
    COLA = _hook(COLA, 0.237)

RECIPE = (WHISKEY, COLA)
# pour_amount_ml in the goal is the SPIRIT measure; the mixer follows at this
# ratio, so one number still describes the whole drink. 50ml + 150ml is a
# standard whiskey and coke.
MIXER_RATIO = 3.0

# Obstacles published into the MoveIt planning scene, in base_link.
#
# Without these, the gross joint-space transits plan straight through the
# scenery: measured, 6 out of 6 moves from home to beside_bottle swept the
# gripper through the bottle, shoving it up to 60mm and spinning it by as much
# as 97 degrees before the grasp had even started. Nothing about that is
# intermittent -- MoveIt simply had no idea any of it was there.
#
# Each bottle carries its own envelope_radius (see RECIPE); both are published
# as cylinders, conservative at any yaw. These constrain only the joint-space
# transits: every Cartesian segment runs with avoid_collisions=False, because
# the grasp itself has to be allowed to reach in and touch the bottle.
# The bar top, in base_link: layout.COUNTER_SIZE at layout.COUNTER_CENTRE,
# which is world (0.08, 0) and 1.76 x 1.6. It carries both arms now, so it
# is also what fills the volume arm B's pedestal used to occupy.
COUNTER_BOX = ((0.53, 0.40, -0.45), (1.76, 1.6, 0.9))
GLASS_ENVELOPE_RADIUS = 0.05
# Height of a bottle stand, from make_bottle_stands.py (HOLD_H + LEAD_H).
STAND_HEIGHT = 0.026

# No fluid simulation, so pouring is a dwell proportional to the amount. This
# used to be capped at 2s, which silently turned any larger request into 50ml
# in the result; the cap is now a floor instead, so 150ml of mixer really does
# take 6s and the reported total is the amount actually asked for.
MIN_POUR_S = 0.5
ASSUMED_ML_PER_SECOND = 25.0

JOINT_TOLERANCE = 0.01
PLANNING_TIME_S = 5.0
PLAN_ATTEMPTS = 6
CARTESIAN_STEP = 0.005
MIN_CARTESIAN_FRACTION = 0.95
# execute_trajectory refuses to run a trajectory whose first point is more
# than allowed_start_tolerance (0.01 rad) from the live robot state, so every
# segment waits for the arm to actually stop BEFORE it plans. Before rather
# than after, because the arm can also be moving for reasons this server did
# not cause -- settling under gravity right after the sim comes up is enough
# to make the very first goal of a session fail. Waiting on measured velocity
# rather than sleeping a fixed time: the observed overshoot at the end of a
# transit reached 0.11 rad, and any constant short enough to not waste time
# was too short to cover it.
ARM_SETTLE_VELOCITY = 0.01    # rad/s, per joint
ARM_SETTLE_TIMEOUT_S = 5.0


def side_quat(theta: float = 0.0):
    """Give the tool0 orientation for the side grasp, as (x, y, z, w).

    Rotated by theta about the base_link Y axis. At theta=0 this maps
    tool0's local +Z (the approach direction) onto base +X, its local +X
    (the finger-closing axis) onto base +Y, and its local +Y
    onto base +Z -- so the bottle stands up along tool0's free axis with the
    fingers straddling it in Y. That is the cyclic axis permutation, i.e.
    (0.5, 0.5, 0.5, 0.5).
    """
    c, s = math.cos(theta / 2.0), math.sin(theta / 2.0)
    return (0.5 * (c + s), 0.5 * (c + s), 0.5 * (c - s), 0.5 * (c - s))


SIDE_QUAT = side_quat(0.0)


def _axis_point_tool(name: str, above_grip: float,
                     off_axis: float = 0.0) -> Tool:
    """Place a point on (or beside) the grasped bottle, as a tool frame.

    `above_grip` runs up the bottle's axis from the grip point, `off_axis`
    out toward bottle-local +X. Negative `above_grip` reaches down toward the
    base. See tool_frames.grasped_bottle_tool for the axis permutation.
    """
    return grasped_bottle_tool(name, above_grip, off_axis,
                               grip_ahead=GRIP_AHEAD_OF_TOOL0)


def tilt_pose(theta: float, bottle: 'Bottle') -> Pose:
    """tool0 pose partway through the pour tilt, for this bottle.

    Solved backwards from where the SPOUT has to be rather than forwards from
    the flange: the spout tip is pinned over the glass for the whole rotation
    and lowered from CARRY_SPOUT_Z to POUR_SPOUT_Z as the tilt progresses, and
    tool0 goes wherever that puts it.

    This used to be hand-rolled trigonometry over the bottle's axis, which was
    right while the pour point WAS on that axis. The pourer's tip is 17mm to
    the side of it, so the offset no longer lies in the plane the tilt turns
    in, and the general transform is needed. tool0_from_tcp is exactly that,
    and it reduces to the old expression when off_axis is zero -- there is a
    test pinning that.
    """
    fraction = min(1.0, theta / POUR_TILT_RAD) if POUR_TILT_RAD else 0.0
    spout_z = CARRY_SPOUT_Z + (POUR_SPOUT_Z - CARRY_SPOUT_Z) * fraction
    quat = side_quat(theta)
    xyz, _ = tool0_from_tcp((GLASS_XY[0], GLASS_XY[1], spout_z), quat,
                            bottle.spout_tool)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quat
    return pose


class PourActionServer(Node):

    def __init__(self):
        super().__init__('pour_action_server')
        cb_group = ReentrantCallbackGroup()

        self._joint_state = None
        # The last thing this node logged as an error, so a caller that only
        # sees the coarse "failed while pouring {bottle}: {stage}" result
        # message has somewhere to find out why. Mirrors Arm.last_error in
        # bartender_open, which exists for the same reason -- see
        # docs/CONTROL_API.md's note on this being the piece still missing.
        self.last_error = None
        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10,
            callback_group=cb_group)

        self._move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb_group)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory', callback_group=cb_group)
        self._gripper_client = ActionClient(
            self, GripperCommand, 'gripper_controller/gripper_cmd',
            callback_group=cb_group)
        self._cartesian_client = self.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=cb_group)
        self._scene_client = self.create_client(
            ApplyPlanningScene, 'apply_planning_scene', callback_group=cb_group)

        self._action_server = ActionServer(
            self, PourDrink, 'pour_drink',
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        for name in ('home', 'whiskey_approach', 'cola_approach', '*'):
            if name in POINT_SOURCES:
                self.get_logger().info(f'point {name}: {POINT_SOURCES[name]}')
        self.get_logger().info('pour_action_server ready')

    def _on_joint_state(self, msg):
        self._joint_state = msg

    # ---- motion helpers -------------------------------------------------

    def _error(self, message):
        """Log an error and remember it verbatim as `last_error`.

        Changes nothing about what gets logged or when -- every call site
        here used to say self.get_logger().error(message) directly. See
        Arm._error in bartender_open, which this copies for the same reason.
        """
        self.last_error = message
        self.get_logger().error(message)

    @staticmethod
    def _block_on(future, timeout_sec: float):
        """Block the calling thread until `future` is done.

        Without spinning anything ourselves: this node's own
        MultiThreadedExecutor is already spinning in another thread and will
        service the callback that completes `future`.
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        done.wait(timeout=timeout_sec)
        return future.result() if future.done() else None

    @staticmethod
    def _collision_object(object_id: str, primitive, xyz) -> CollisionObject:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        pose.orientation.w = 1.0
        obj = CollisionObject()
        obj.header.frame_id = PLANNING_FRAME
        obj.id = object_id
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _publish_obstacles(self) -> bool:
        """Put the counter, bottles, stands and glass into the planning scene.

        Re-sent per goal rather than once at startup, so the scene is restored
        if move_group restarts. Each bottle object stays where that bottle
        stands: while one is actually in the gripper its object is stale, but
        every motion in that window is Cartesian and unchecked, and by the time
        a joint-space plan runs again it really is back on the counter. The
        OTHER bottle is meanwhile correct, which is what stops the transit to
        the second bottle from sweeping through the first.
        """
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self._error('apply_planning_scene not available')
            return False

        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [
            self._collision_object(
                'bar_counter',
                SolidPrimitive(type=SolidPrimitive.BOX,
                               dimensions=list(COUNTER_BOX[1])),
                COUNTER_BOX[0]),
            *[self._collision_object(
                bottle.name,
                SolidPrimitive(type=SolidPrimitive.CYLINDER,
                               dimensions=[bottle.standing_height,
                                           bottle.envelope_radius]),
                (*bottle.xy, bottle.standing_height / 2.0)) for bottle in RECIPE],
            self._collision_object(
                'glass',
                SolidPrimitive(type=SolidPrimitive.CYLINDER,
                               dimensions=[GLASS_HEIGHT, GLASS_ENVELOPE_RADIUS]),
                (*GLASS_XY, GLASS_HEIGHT / 2.0)),
            # The stands. Unlike the bottles these never move and are never
            # picked up, so their objects are never stale -- which also means
            # they still guard the station while the bottle that stands there
            # is in the gripper and its own object has gone out of date.
            *[self._collision_object(
                f'{bottle.name}_stand',
                SolidPrimitive(type=SolidPrimitive.CYLINDER,
                               dimensions=[STAND_HEIGHT, bottle.stand_radius]),
                (*bottle.xy, STAND_HEIGHT / 2.0))
              for bottle in RECIPE if bottle.stand_radius],
        ]
        response = self._block_on(
            self._scene_client.call_async(ApplyPlanningScene.Request(scene=scene)),
            timeout_sec=10.0)
        if response is None or not response.success:
            self._error('failed to apply planning scene obstacles')
            return False
        return True

    def _move_to_joints(self, name: str, positions) -> bool:
        joint_constraints = [
            JointConstraint(
                joint_name=joint, position=pos,
                tolerance_above=JOINT_TOLERANCE, tolerance_below=JOINT_TOLERANCE,
                weight=1.0,
            )
            for joint, pos in zip(ARM_JOINTS, positions)
        ]

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest(
            group_name=ARM_GROUP_NAME,
            goal_constraints=[Constraints(joint_constraints=joint_constraints)],
            allowed_planning_time=PLANNING_TIME_S,
            num_planning_attempts=5,
        )
        goal.planning_options = PlanningOptions(plan_only=False)

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self._error('move_action server not available')
            return False

        # RRTConnect is randomised and regularly returns a path that only fails
        # validation after time-parameterisation (typically a forearm_link /
        # right finger self-collision). Resampling clears it.
        for attempt in range(PLAN_ATTEMPTS):
            # Settle before EVERY attempt, not just the first. A failed attempt
            # often leaves the arm part-way along the trajectory it was told to
            # abandon, and retrying instantly (the retries were firing 40ms
            # apart) means the next goal preempts a move still in progress and
            # is itself invalidated for deviating from a moving start state --
            # a failure loop that could burn all six attempts on a goal that
            # was perfectly reachable, and did.
            self._wait_until_arm_settled()
            send_future = self._move_group_client.send_goal_async(goal)
            goal_handle = self._block_on(send_future, timeout_sec=15.0)
            if goal_handle is None or not goal_handle.accepted:
                continue
            result = self._block_on(goal_handle.get_result_async(), timeout_sec=60.0)
            if result is not None and result.result.error_code.val == 1:
                return True
            code = result.result.error_code.val if result is not None else 'no result'
            self.get_logger().warn(
                f'plan to "{name}" attempt {attempt + 1}/{PLAN_ATTEMPTS} failed '
                f'(error_code {code}), retrying')
        self._error(f'MoveGroup goal to "{name}" failed')
        return False

    def _move_cartesian(self, x: float, y: float, z: float, quat=SIDE_QUAT,
                        label: str = '', may_stall: bool = False) -> bool:
        """Straight line of tool0 to the given base_link pose."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = quat
        return self._follow_cartesian([pose], label, may_stall)

    def _pour_tilt(self, bottle: Bottle, reverse: bool = False) -> bool:
        """Rotate the grasped bottle to the pour angle, or back upright.

        Subdivided so the planner interpolates the rotation smoothly.
        """
        angles = [POUR_TILT_RAD * i / TILT_WAYPOINTS
                  for i in range(TILT_WAYPOINTS + 1)]
        if reverse:
            angles.reverse()
        return self._follow_cartesian(
            [tilt_pose(a, bottle) for a in angles[1:]],
            f'{bottle.name} untilt' if reverse else f'{bottle.name} tilt')

    def _follow_cartesian(self, poses, label: str = '',
                          may_stall: bool = False) -> bool:
        if self._joint_state is None:
            self._error('no /joint_states yet, cannot seed Cartesian plan')
            return False
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            self._error('compute_cartesian_path service not available')
            return False
        self._wait_until_arm_settled()

        request = GetCartesianPath.Request()
        request.header.frame_id = PLANNING_FRAME
        # start_state is deliberately left empty so move_group seeds from its
        # own current state. Seeding it from this node's cached /joint_states
        # raced the arm settling: the plan would come back at fraction 1.00 and
        # then execute_trajectory would reject it with "start point deviates
        # from current robot state more than 0.01".
        request.group_name = ARM_GROUP_NAME
        request.link_name = EEF_LINK
        request.max_step = CARTESIAN_STEP
        request.jump_threshold = 0.0
        # The bottle/glass/counter are not in the planning scene at all, so
        # collision checking here would only ever catch self-collisions while
        # costing us plan fraction on the very segments we most need to be
        # straight. The straight line itself is what keeps us off the objects.
        request.avoid_collisions = False
        request.waypoints = list(poses)

        response = self._block_on(
            self._cartesian_client.call_async(request), timeout_sec=30.0)
        if response is None:
            self._error(f'Cartesian plan "{label}" timed out')
            return False
        if response.fraction < MIN_CARTESIAN_FRACTION:
            self._error(
                f'Cartesian plan "{label}" only reached {response.fraction:.2f} of the path')
            return False

        # Worth logging: comparing this against how long the segment actually
        # takes is how the simulator's contact cost was found (see the physics
        # note in bar_world.sdf). They should now agree within ~5%.
        points = response.solution.joint_trajectory.points
        if points:
            span = points[-1].time_from_start
            self.get_logger().info(
                f'segment "{label}": {len(points)} points, '
                f'{span.sec + span.nanosec * 1e-9:.2f}s')

        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            self._error('execute_trajectory server not available')
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal_handle = self._block_on(
            self._execute_client.send_goal_async(goal), timeout_sec=15.0)
        if goal_handle is None or not goal_handle.accepted:
            self._error(f'Cartesian execution "{label}" rejected')
            return False
        result = self._block_on(goal_handle.get_result_async(), timeout_sec=60.0)
        if result is None or result.result.error_code.val != 1:
            code = result.result.error_code.val if result is not None else '-'
            if may_stall:
                # EXPECTED not to arrive; see the `lowering` step. The plan
                # still had to come back complete above -- a short plan means
                # the path was unreachable, which is an error however it ends
                # -- but the controller refusing the last millimetres is the
                # intended outcome here and is logged, not failed on.
                self.get_logger().info(
                    f'"{label}" did not arrive (error_code {code}), which is '
                    f'what setting something down on the counter looks like')
                return True
            self._error(
                f'Cartesian execution "{label}" failed (error_code {code})')
            return False
        return True

    def _send_gripper(self, position: float):
        """Send one gripper goal, refusing any that would park on the stop.

        Refused rather than clamped, as elsewhere in this repo: quietly
        substituting GRIPPER_OPEN_POS for a requested 0.0 would hide the
        fact that 0.0 kills the joint for the rest of the run. Nothing here
        asks for it any more; the guard is for whatever is written next.
        """
        if not GRIPPER_OPEN_POS <= float(position) <= GRIPPER_UPPER_LIMIT:
            self._error(
                f'refusing gripper command {position:.4f} rad. The usable '
                f'band is {GRIPPER_OPEN_POS:.2f}..{GRIPPER_UPPER_LIMIT:.2f}; '
                f'resting on the lower joint limit stops the knuckle '
                f'responding for the rest of the run.')
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        return self._block_on(
            self._gripper_client.send_goal_async(goal), timeout_sec=10.0)

    def _command_gripper(self, position: float, clamp: bool = False,
                         ramp: bool = False) -> bool:
        """Drive the gripper.

        clamp and ramp both walk the move in GRIPPER_CLOSE_STEP increments,
        which is what keeps the pads from arriving (or leaving) hard enough to
        knock the bottle about. They differ in how they finish: a clamp leaves
        its last goal running, because a closing goal that has bottomed out on
        the bottle can never report success and keeping it active is what
        maintains grip force; a ramped release waits for the open to complete
        and then lets the bottle settle.

        Use ramp for any move that starts in contact. A plain single-shot
        command is fine only in free air.
        """
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self._error('gripper action server not available')
            return False

        start = self._gripper_position()
        if math.isnan(start):
            start = GRIPPER_OPEN_POS
        # Floored, because `start` is only the interpolation origin and a
        # MEASURED one: the knuckle overshoots and can be read at 0.0178
        # mid-brush (see GRIPPER_LOWER_LIMIT and bartender_open/arm.py). Left alone,
        # the first interpolated step would come out under GRIPPER_OPEN_POS
        # and _send_gripper would refuse it, failing the whole command with
        # "gripper goal rejected" -- a message about the wrong thing.
        start = max(GRIPPER_OPEN_POS, start)
        steps = (max(1, int(math.ceil(abs(position - start) / GRIPPER_CLOSE_STEP)))
                 if (clamp or ramp) else 1)

        goal_handle = None
        for i in range(1, steps + 1):
            # Intermediate goals are not waited on: each preempts the last, and
            # past first contact a closing one stalls by design.
            # The LAST step is `position` itself and not the same
            # arithmetic as the others. start + (position - start) * i/steps
            # does not land exactly on position in binary floating point:
            # releasing from the whiskey's 0.25 clamp computes
            # 0.019999999999999990 for a target of 0.02, which is below
            # GRIPPER_OPEN_POS and gets refused -- so the release would
            # fail, on two of the three clamp angles in this file, for a
            # reason no log line would explain.
            here = (position if i == steps
                    else start + (position - start) * i / steps)
            goal_handle = self._send_gripper(here)
            if goal_handle is None or not goal_handle.accepted:
                self._error('gripper goal rejected')
                return False
            if i < steps:
                time.sleep(GRIPPER_STEP_DWELL_S)

        if clamp:
            time.sleep(GRIPPER_SETTLE_S)
            reached = self._gripper_position()
            if abs(position - reached) < GRASP_STALL_MARGIN:
                self._error(
                    f'grasp failed: fingers reached {reached:.4f} rad against a '
                    f'command of {position:.4f}, so nothing is between them')
                return False
            self.get_logger().info(
                f'clamped at {reached:.4f} rad, {position - reached:.4f} short '
                f'of the command (goal left active)')
            return True

        ok = self._block_on(
            goal_handle.get_result_async(), timeout_sec=20.0) is not None
        if ramp:
            time.sleep(GRIPPER_RELEASE_SETTLE_S)
        return ok

    def _still_holding_bottle(self, bottle: 'Bottle') -> bool:
        """Report whether the bottle is still in the fingers.

        Same test as the grasp check: a held bottle keeps the joint stalled
        short of the commanded angle, so if the fingers have since closed to
        the command, the bottle went somewhere between then and now. Without
        this the pour reports success even when the bottle was dropped mid
        cycle and is lying on the counter.
        """
        reached = self._gripper_position()
        if abs(bottle.clamp_pos - reached) < GRASP_STALL_MARGIN:
            self._error(
                f'{bottle.name} lost: fingers have closed to {reached:.4f} rad, '
                f'the commanded {bottle.clamp_pos:.4f}, so they are now empty')
            return False
        return True

    def _wait_until_arm_settled(self) -> None:
        """Block until every arm joint reports (near) zero velocity."""
        deadline = time.time() + ARM_SETTLE_TIMEOUT_S
        while time.time() < deadline:
            js = self._joint_state
            if js is not None and len(js.velocity) >= len(js.name):
                try:
                    speeds = [abs(js.velocity[js.name.index(j)]) for j in ARM_JOINTS]
                except ValueError:
                    speeds = []
                if speeds and max(speeds) < ARM_SETTLE_VELOCITY:
                    return
            time.sleep(0.05)
        self.get_logger().warn(
            'arm still moving after %.1fs; planning the next segment anyway'
            % ARM_SETTLE_TIMEOUT_S)

    def _gripper_position(self) -> float:
        js = self._joint_state
        if js is None or GRIPPER_JOINT not in js.name:
            return float('nan')
        return js.position[js.name.index(GRIPPER_JOINT)]

    # ---- the pour state machine ------------------------------------------

    def _pour_from(self, bottle: Bottle, amount_ml: float, step) -> bool:
        """One bottle's pick -> carry -> tilt -> pour -> return cycle.

        Every pose is derived from the Bottle, so the two bottles differ only
        in their entries in RECIPE. Deliberately ends with the arm clear and
        high rather than at home: when another bottle follows, going home in
        between is a wasted transit.
        """
        steps = [
            ('opening_gripper', self._command_gripper, (GRIPPER_OPEN_POS,), {}),
            ('approaching', self._move_to_joints,
             (bottle.name, bottle.approach_joints), {}),
            ('descending_beside', self._move_cartesian,
             (bottle.approach_x, bottle.xy[1], bottle.grasp_height),
             {'label': f'{bottle.name} descend'}),
            ('closing_on', self._move_cartesian,
             (bottle.grasp_x, bottle.xy[1], bottle.grasp_height),
             {'label': f'{bottle.name} run-in'}),
            ('grasping', self._command_gripper, (bottle.clamp_pos,),
             {'clamp': True}),
            ('lifting', self._move_cartesian,
             (bottle.grasp_x, bottle.xy[1], bottle.carry_z),
             {'label': f'{bottle.name} lift'}),
            # Straight to the pose the tilt starts from, which puts the bottle
            # upright over the glass however far away it was standing.
            ('moving_to_glass', self._follow_cartesian,
             ([tilt_pose(0.0, bottle)],), {'label': f'{bottle.name} to glass'}),
            ('tilting_to_pour', self._pour_tilt, (bottle,), {}),
        ]
        for state, fn, args, kwargs in steps:
            if not step(f'{state}_{bottle.name}', fn, *args, **kwargs):
                return False

        step(f'pouring_{bottle.name}', lambda: True)
        time.sleep(max(MIN_POUR_S, amount_ml / ASSUMED_ML_PER_SECOND))

        return_steps = [
            ('checking_grip', self._still_holding_bottle, (bottle,), {}),
            ('returning_upright', self._pour_tilt, (bottle,), {'reverse': True}),
            ('returning', self._move_cartesian,
             (bottle.grasp_x, bottle.xy[1], bottle.carry_z),
             {'label': f'{bottle.name} back'}),
            # may_stall: THE BOTTLE STOPS ON THE COUNTER, so this move
            # cannot arrive and must not be failed on.
            #
            # It looked like it arrived for as long as the arm controllers
            # had no goal tolerance and reported SUCCEEDED wherever they
            # stopped (see constraints: in bartender_description's
            # controllers.yaml). With the tolerance in place the truth shows
            # up: measured on a full pour, the place ends with wrist_1 held
            # 0.029 rad -- about 6mm at the bottle -- short of its command,
            # steady, not decaying. That is the arm pressing the bottle onto
            # a counter that is already holding it up.
            #
            # Nothing is lost by exempting it. The evidence that the place
            # worked is the bottle's own pose afterwards, not the
            # controller's opinion, and this is the ONLY segment in the pour
            # that ends in contact -- the retreat and clear that follow are
            # free-air moves and are still checked.
            ('lowering', self._move_cartesian,
             (bottle.grasp_x, bottle.xy[1], bottle.grasp_height),
             {'label': f'{bottle.name} place', 'may_stall': True}),
            ('releasing', self._command_gripper, (GRIPPER_OPEN_POS,),
             {'ramp': True}),
            # Straight back out along -X, the reverse of the run-in: the pads
            # are still only millimetres from the body, so lifting away here
            # would drag the bottle over with them. Only once clear does the
            # arm rise.
            ('retreating', self._move_cartesian,
             (bottle.approach_x, bottle.xy[1], bottle.grasp_height),
             {'label': f'{bottle.name} retreat'}),
            ('clearing', self._move_cartesian,
             (bottle.approach_x, bottle.xy[1], APPROACH_Z),
             {'label': f'{bottle.name} clear'}),
        ]
        for state, fn, args, kwargs in return_steps:
            if not step(f'{state}_{bottle.name}', fn, *args, **kwargs):
                return False
        return True

    def _recover_to_safe(self, bottle: Bottle) -> None:
        """Get the arm out of the bottle's envelope and back home, best effort.

        Called after a failed cycle.

        Aborting in place leaves the gripper parked wherever it stopped, which
        for anything from the run-in onwards is INSIDE that bottle's published
        envelope. Every joint-space plan from such a start state is then
        rejected instantly, so one failed cycle poisoned every goal after it --
        measured, three runs in six failed at the very first transit, having
        never touched a bottle, purely because the previous run had left the
        arm sitting in the cola.

        Each step is attempted and its failure logged rather than propagated:
        the goal has already failed, and a recovery that only partly works
        still leaves the arm better off than not trying.
        """
        self.get_logger().warn(f'recovering from failed {bottle.name} cycle')
        steps = (
            ('release', lambda: self._command_gripper(GRIPPER_OPEN_POS, ramp=True)),
            ('back out', lambda: self._move_cartesian(
                bottle.approach_x, bottle.xy[1], bottle.grasp_height,
                label='recover back')),
            ('rise', lambda: self._move_cartesian(
                bottle.approach_x, bottle.xy[1], APPROACH_Z,
                label='recover up')),
            ('home', lambda: self._move_to_joints('home', HOME_JOINTS)),
        )
        for label, run in steps:
            try:
                if not run():
                    self.get_logger().warn(f'recovery step "{label}" failed')
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().warn(f'recovery step "{label}" raised: {exc}')

    def execute_callback(self, goal_handle):
        feedback = PourDrink.Feedback()
        # Progress is shared out over the bottles in RECIPE plus the two
        # bookend steps, so it still runs 0..1 whatever the recipe holds.
        total_steps = len(RECIPE) * 16 + 2
        done = [0]

        def step(state: str, fn, *args, **kwargs) -> bool:
            done[0] += 1
            feedback.state = state
            feedback.progress = min(1.0, done[0] / total_steps)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'pour_drink: {state}')
            return fn(*args, **kwargs)

        if not step('publishing_obstacles', self._publish_obstacles):
            goal_handle.abort()
            return self._fail_result(
                f'failed at step: publishing_obstacles{self._why()}')

        # The goal's pour_amount_ml is the spirit measure; the mixer follows at
        # MIXER_RATIO, so the reported total is what actually went in the glass.
        spirit_ml = max(0.0, goal_handle.request.pour_amount_ml)
        amounts = [spirit_ml] + [spirit_ml * MIXER_RATIO] * (len(RECIPE) - 1)

        poured_ml = 0.0
        for bottle, amount in zip(RECIPE, amounts):
            if not self._pour_from(bottle, amount, step):
                failed_at = feedback.state
                # Snapshot before recovery runs: _recover_to_safe issues more
                # motion and gripper commands of its own, and a failure in
                # one of those would otherwise overwrite last_error with the
                # recovery's own reason rather than the one that actually
                # aborted the pour.
                reason = self._why()
                self._recover_to_safe(bottle)
                goal_handle.abort()
                return self._fail_result(
                    f'failed while pouring {bottle.name}: {failed_at}{reason}')
            poured_ml += amount

        if not step('returning_home', self._move_to_joints, 'home', HOME_JOINTS):
            goal_handle.abort()
            return self._fail_result(f'failed at step: returning_home{self._why()}')

        goal_handle.succeed()
        result = PourDrink.Result()
        result.success = True
        result.message = ' then '.join(
            f'{a:.0f}ml {b.name}' for b, a in zip(RECIPE, amounts))
        result.estimated_poured_ml = poured_ml
        return result

    def _why(self):
        """Give ' (reason)' if this node has a logged reason, else ''.

        Mirrors OpenActionServer._why in bartender_open, for the same
        purpose: a coarse "failed while pouring X: stage" result message
        should not discard the specific thing a sub-call already worked out.
        """
        return f' ({self.last_error})' if self.last_error else ''

    @staticmethod
    def _fail_result(message: str) -> PourDrink.Result:
        result = PourDrink.Result()
        result.success = False
        result.message = message
        result.estimated_poured_ml = 0.0
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PourActionServer()
    # >=2 threads: one to run the action server callback, one free to
    # service the nested MoveGroup/GripperCommand client spins it triggers.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
