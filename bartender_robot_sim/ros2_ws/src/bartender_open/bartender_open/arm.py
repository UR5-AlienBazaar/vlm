"""One arm's worth of MoveIt and gripper plumbing, parameterised by prefix.

WHY THIS IS NOT bartender_pour's CODE
-------------------------------------
bartender_pour has the same plumbing and it is not shared. That is a real
duplication and worth being honest about rather than dressing up.

The pour server's helpers are methods that read module-level constants:
ARM_JOINTS, ARM_GROUP_NAME, EEF_LINK, PLANNING_FRAME, GRIPPER_JOINT. With one
arm that was the simplest thing that worked. Making them serve two arms means
turning every one of those into an attribute, which means touching every
method in a 1200-line file that encodes a pour cycle tuned over many measured
runs -- for the benefit of a sequence that shares none of that tuning.

So the shape of the plumbing is copied and the constants that were tuned
against contact are NOT: the gripper step size, the settle times, the stall
margin and the approach discipline are all the same values, and where they
are the same they say so, because if one of them is ever retuned both places
need it. If a third arm or a third action appears, that is the moment to lift
this into one module and migrate the pour server onto it; doing it now would
mean re-validating the pour against a refactor it does not need.

WHAT IS DIFFERENT HERE
----------------------
One thing, and it is the reason this exists at all: `follow_cartesian` can be
told that failure is an expected outcome. Every move bartender_pour makes is
meant to reach where it was sent. The press at the end of this sequence is
not -- it is a commanded position 6mm inside a bottle that is being held
still, so the trajectory controller is SUPPOSED to end up short and report a
goal-tolerance violation. Treating that as an error would fail every
successful open.
"""
import math
import random
import threading
import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint,
    PlanningOptions, PositionConstraint,
)
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import (
    GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

# THE KNUCKLE MUST NEVER BE PARKED ON ITS LOWER JOINT LIMIT.
#
# This is the one gripper constant that is NOT bartender_pour's, and the
# difference is the entire fix for a defect that used to break about half of
# all opens. bartender_pour has been corrected to match; see the identical
# note there.
#
# The knuckle joint is limit=[0.0, 0.8] (robotiq_2f_85_macro.urdf.xacro).
# Drive it to 0.0 -- fully open, which is exactly what "open the gripper"
# used to ask for -- and leave it there, and it stops responding to gripper
# commands FOR THE REST OF THE SIMULATOR RUN. Measured on single stacks,
# both arms side by side so the two conditions share a simulator:
#
#   parked at 0.300 for 90s, then commanded 0.50  ->  moved, in 2.5s
#   parked at 0.000 for 90s, then commanded 0.50  ->  never moved
#   parked at 0.020 for 90s, then commanded 0.50  ->  moved, in 5.0s
#   parked at 0.000 for 10s, then commanded 0.50  ->  never moved
#
# It is the LOWER limit specifically. Parking on the upper one is harmless,
# which is why the close side of this needs no margin of its own:
#
#   parked at 0.800 for 90s, then commanded 0.50  ->  moved, in 3.5s
#
# What it is NOT, each ruled out by measurement rather than by reading:
#   - not stale feedback. Gazebo's own link poses, on
#     /world/bar_world/dynamic_pose/info, show the fingertip stationary to
#     four decimals while the command is 0.5. The finger really is not
#     moving.
#   - not the controller. `ros2 control list_controllers` reports the
#     gripper active throughout, and it goes on accepting goals -- which is
#     why this presented as "goal accepted, joint never moves".
#   - not the simulator slowing down. /clock holds a steady 0.21 real-time
#     factor across a 120s window in which the joint does not move at all.
#   - not the arm. The same arm's six joints keep tracking normally.
#   - not the number 0.0. The arm joints sit at 0.0 at home for minutes and
#     move afterwards; their limits are nowhere near it.
#
# It IS a simulator artefact rather than a controller misconfiguration,
# which is the question worth settling before working around it. Three
# things say so: the controller goes on writing (it stays active and keeps
# accepting goals), Gazebo's ground truth agrees the finger is not moving,
# and the joint can be brought back by DISTURBING it -- moving arm B's
# wrist while its knuckle was dead with a stale 0.5 command pending left
# the knuckle at 0.500. A joint that resumes when something shakes it was
# not being integrated, which is a physics-side state and not something
# ros2_control has a setting for.
#
# What is NOT explained: the arm also moves between the open and the grasp
# in the sequences that failed, and there the gripper stayed dead. So
# "any motion revives it" is too strong -- it has been seen once. The
# avoidance below does not depend on knowing which, and none of the failures
# ever recurred once the joint stopped being parked on the stop.
#
# So: the joint is dead only when it comes to rest ON the lower limit, and
# 0.02 rad of margin is enough to avoid it -- 0.020 survived dwells of 90s
# and 300s, 0.000 died at 10s, 90s and 300s. It costs 1.8mm of pad gap --
# 83.2mm open instead of 85.0 -- against a widest gripped object of
# 38.6mm here, so nothing in this package comes close to noticing.
# bartender_pour is tighter and says what it costs there.
GRIPPER_LOWER_LIMIT = 0.0       # from the URDF; do not command it
GRIPPER_LIMIT_MARGIN = 0.02
GRIPPER_OPEN_POS = GRIPPER_LOWER_LIMIT + GRIPPER_LIMIT_MARGIN

# The rest ARE bartender_pour's values, and deliberately so -- they were
# measured against this gripper on these bottles. See the comments beside
# each one in pour_action_server.py for what was measured and why.
GRIPPER_MAX_EFFORT = 100.0
GRIPPER_SETTLE_S = 1.0
GRIPPER_CLOSE_STEP = 0.02
GRIPPER_STEP_DWELL_S = 0.1
GRIPPER_RELEASE_SETTLE_S = 0.5
GRASP_STALL_MARGIN = 0.05

JOINT_TOLERANCE = 0.01
PLANNING_TIME_S = 5.0
PLAN_ATTEMPTS = 6
CARTESIAN_STEP = 0.005
MIN_CARTESIAN_FRACTION = 0.95

# HOW FAR A DESCENT MAY DIP ITS OWN ARM INTO THE WORKTOP. None of it.
#
# Both arm bases are bolted to the bar top, so in either arm's frame z = 0 IS
# the counter surface (see the frame note in layout.py) and no part of a
# working arm has any business below it.
#
# This is checked because the straight-line descents here run with
# avoid_collisions=False -- deliberately, since they finish in contact with
# the thing being picked up and a collision-aware path would refuse the last
# few millimetres. The cost of that opt-out is that nothing else was looking
# at the rest of the arm. Traced on the opener pick: compute_cartesian_path
# returned 100% of the path, the controller reported "Goal reached,
# success!", and the flange stopped 267mm from where it was sent at
# (0.3291, 0.0208, 0.3406) instead of (0.365, 0, 0.077) -- identical to a
# tenth of a millimetre across runs, because the descent was driving
# b_wrist_1_link 22.7mm UNDER the bar and the arm simply jammed on it.
# An IK probe at that grasp pose finds six branches, and four of them put
# wrist_1 below the counter, so picking one by luck is the normal case.
#
# It is the approach configuration that decides which of those branches the
# descent runs through, and approach_then already enumerates approaches and
# asks each one whether the descent is reachable. This makes it ask the
# second question too.
DESCENT_FLOOR_Z = 0.0
# Link ORIGINS, not geometry: a check on the origins is crude, and it is
# enough, because an arm that has put a wrist origin under the worktop is
# already far past grazing it. Sampling rather than every waypoint keeps the
# cost to a dozen FK calls per candidate branch; a dip deep enough to stop
# the arm is tens of millimetres wide and cannot hide between samples.
DESCENT_SAMPLES = 12
# Everything from the shoulder out. The base links cannot move, and the
# fingers are checked through tool0, which they hang off.
DESCENT_CHECK_LINKS = (
    'shoulder_link', 'upper_arm_link', 'forearm_link',
    'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0',
)
ARM_SETTLE_VELOCITY = 0.01
# Joint-space moves run at a third speed (see JOINT_MOVE_SCALING), so they
# take correspondingly longer to come to rest.
ARM_SETTLE_TIMEOUT_S = 9.0

# How many IK solutions to look at before picking one. See solve_ik: the
# solver is randomised and hands back whichever branch it lands on, and some
# branches for these targets fold the arm into its own base. Asking several
# times and keeping the best is much cheaper than discovering that after the
# arm has been sent there.
IK_ATTEMPTS = 8
# How far the seed is jittered on the attempts after the first, in radians.
# Big enough to land the solver in a different basin, small enough that what
# comes back is still a nearby posture.
IK_SEED_JITTER = 0.8

# How much of the arm's velocity and acceleration limits a joint-space move
# is allowed to use.
#
# A third, which is slow, and it buys the one thing this sequence cannot do
# without: a path that is actually the path that was planned. A collision-
# checked plan is only worth what the execution tracks it to, and a
# position-only trajectory controller in simulation cuts corners at speed --
# measured, arm A swept its open fingers through a standing bottle on the way
# to a pose 135mm above it, from a plan checked against a keep-out volume
# 100mm wider than the bottle, which means the real path left the planned one
# by something over 70mm.
#
# Cartesian segments are not scaled here. They are short, they are already
# slow because they are dense in waypoints, and the press at the end has to
# push rather than glide to a halt.
JOINT_MOVE_SCALING = 0.33

# Pause between failed planning attempts.
RETRY_BACKOFF_S = 1.5

# How far the flange may be from where a move was told to put it before the
# move counts as not having arrived. 4mm: bigger than the tracking error
# these segments normally leave, and small enough that the pad is still on
# the feature it was aimed at -- the tightest of those is the beer's neck,
# where the pad has 4.5mm of clearance to the shoulder step below it.
ARRIVAL_TOLERANCE = 0.004

# PAD GAP AGAINST KNUCKLE ANGLE, in metres and radians.
#
# The 2F-85's linkage is not proportional and assuming it was cost two runs.
# Measured by forward kinematics on the rendered URDF, taking the distance
# between the two fingertip link origins and subtracting the 50.52mm the pad
# faces are inset from them -- which is the number that makes the gap come
# out at the 85.00mm the gripper is specified to open to.
#
# It reproduces both figures bartender_pour measured in sim independently:
# the whiskey's 77.2mm flats touch at 0.088 here against a recorded 0.089
# stall, and the cola's 59.6mm waist at 0.264 against a recorded ~0.265. So
# it can be trusted for an object nobody has gripped yet, which is what it is
# for.
PAD_GAP = (
    (0.00, 0.08500), (0.05, 0.08056), (0.10, 0.07596), (0.15, 0.07119),
    (0.20, 0.06626), (0.25, 0.06120), (0.30, 0.05601), (0.35, 0.05071),
    (0.40, 0.04531), (0.45, 0.03983), (0.50, 0.03426), (0.55, 0.02865),
    (0.60, 0.02298), (0.65, 0.01729), (0.70, 0.01158), (0.75, 0.00586),
    (0.80, 0.00016),
)

# Closing behaviour. These replace the fixed clamp angle bartender_pour uses:
# rather than predicting where the fingers will stop and commanding past it,
# close in steps, wait for each one to actually arrive, and notice when one
# does not. See Arm.grasp.
# How much wider than the object the ONE fast closing command may leave the
# pads. It has to cover that command's overshoot, because the fast close is
# the only move here big enough to have much: commanded to 0.351 rad it has
# been read settling at 0.470, which on a 38.6mm neck is 37.6mm -- the pads
# past the object before the careful part of the close had begun. 25mm of
# margin absorbs that; the 0.02 steps after it barely overshoot at all.
GRIPPER_PRE_CLOSE_GAP = 0.025
# Step size for the fast part of the close. Not one jump: a single large
# position command on this linkage overshoots badly and erratically --
# commanded to 0.226 rad it was read settling at 0.463, twice as far, with
# the pads past the object before the careful part of the close had started.
# Five steps of 0.05 cover the same ground with an overshoot small enough to
# ignore, and cost about a second.
GRIPPER_FAST_STEP = 0.05
GRIPPER_STALL_GAP = 0.03        # command minus achieved that means "stopped"
# ...and how little a 0.02 step has to advance the joint to agree with that.
#
# Being short of the command is not enough on its own. At FIRST CONTACT the
# fingers are momentarily stopped too, and a stall declared there has no bite
# in it: measured, grips accepted at 37.4 and 37.6mm on a 38.6mm neck -- one
# millimetre of penetration -- which held through the lift and then let the
# bottle slide 28mm when the other arm pushed on it. The same close left
# alone goes on to settle at 34.8mm, which is 4mm of bite and does not slip.
#
# A step that barely moves the joint is the thing that means the fingers have
# arrived somewhere they cannot leave. A free step advances about 0.02; this
# is well under that and well over the creep of a pad sinking into a round
# section.
GRIPPER_CONFIRM_MOVE = 0.008
# How long to let the fingers settle into the object under the squeeze
# command before reading off the grasp. Generous, because this is the one
# measurement the rest of the sequence is judged against.
GRIPPER_SQUEEZE_S = 2.5
# How far the command may get ahead of the joint before the conclusion is
# that the gripper is not moving at all, rather than that it has stopped on
# something. Contact never produces a gap anything like this: the command is
# only ever GRIPPER_CLOSE_STEP ahead of a joint that is following, and
# GRIPPER_SQUEEZE ahead of one that has bottomed out.
#
# Without it, a gripper that never budged reports "fingers closed all the way
# without meeting anything" -- which is what the COMMAND did, and says
# nothing about the fingers. Two runs failed that way and the message sent
# the search to the wrong place entirely, to where the bottle was rather than
# to whether the gripper was working.
GRIPPER_UNRESPONSIVE = 0.25
GRIPPER_STEP_TIMEOUT_S = 2.0    # how long one 0.02 step is given to arrive
# The knuckle is "settled" when it has not MOVED more than this over a window
# this long -- position, not velocity.
#
# Velocity was the obvious test and it is wrong here. The joint overshoots
# and rings, and at the top of an overshoot its velocity is exactly zero, so
# a velocity test reports "settled" at the worst possible instant: measured,
# a pre-close commanded to 0.351 rad was read back as 0.508, which is a pad
# gap of 33mm for an object of 39mm -- an impossible reading that was then
# taken as a grasp, and the bottle was left standing on the counter while the
# arm lifted nothing.
GRIPPER_SETTLED_MOVE = 0.003
GRIPPER_SETTLED_WINDOW_S = 0.40
# How close to the commanded angle counts as having ARRIVED, as opposed to
# having stopped short of it. Worth separating: a step that arrives can be
# returned on at once, while a step that has stopped has to be watched for
# the whole settle window before that is believed. Without the distinction,
# the window has to be short enough not to waste a second on every one of
# thirty free steps -- and a short window calls a slow creep "stopped", which
# is how grips with 0.9mm of bite came to be accepted as stalls.
GRIPPER_ARRIVED = 0.010
GRIPPER_SQUEEZE = 0.18          # how far past the measured stall to command
# Unlike the open stop, the closed one is safe to sit on (see
# GRIPPER_LOWER_LIMIT), so this is not a margin -- it is just where the
# linkage runs out: PAD_GAP puts the pads 0.2mm apart at 0.80.
GRIPPER_UPPER_LIMIT = 0.8       # from the URDF
GRIPPER_FULLY_CLOSED = 0.78     # reaching this means nothing is in the way
# What the pad gap has to be for a stall to be believed, relative to the
# width of the thing being gripped. The two limits are NOT symmetric, because
# the two ways of being wrong are completely different.
#
# Too WIDE means the fingers have not reached the object yet and the step
# simply has not arrived -- the knuckle lags a moving command badly, so a
# slow step looks exactly like contact if all you check is that the joint is
# short of where it was told to be. Observed: a stall accepted at 42.5mm on a
# 38.6mm neck, which is the pads stopping 2mm clear of either side of it. The
# grip held just long enough to fail under the press. So a wide reading is
# not contact; it is "keep closing".
#
# Too NARROW means the pads have closed further than the object can account
# for, which means they are not on it. Some overlap is real and expected:
# the solver resolves contact by penetration, and a round section needs
# several millimetres of it before the joint stalls -- bartender_pour records
# ~6mm for the cola against 0.2mm for the whiskey's flats. 8mm allows for
# that and still rules out an empty gripper.
GRASP_LOOSE = 0.002
GRASP_PENETRATION = 0.008

JOINT_SUFFIXES = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)


def side_quat(theta: float = 0.0):
    """tool0 orientation for a side grasp, as (x, y, z, w).

    Identical to bartender_pour's: at theta=0 it maps tool0's +z (approach)
    onto the arm's own +x, tool0's +x (the closing axis) onto +y, and tool0's
    +y onto +z, so an upright object stands along tool0's free axis with the
    pads either side of it.

    Because it is expressed in the ARM'S OWN base frame, the same quaternion
    means something different in the world for each arm, and both meanings
    are the ones wanted: arm A approaches along world +x, arm B -- yawed -90
    degrees -- approaches along world -y, i.e. in over the counter from
    behind it. That is why neither arm needs a special case.
    """
    c, s = math.cos(theta / 2.0), math.sin(theta / 2.0)
    return (0.5 * (c + s), 0.5 * (c + s), 0.5 * (c - s), 0.5 * (c - s))


SIDE_QUAT = side_quat(0.0)


def knuckle_for_gap(gap: float) -> float:
    """Knuckle angle that opens the pads to `gap` metres, by interpolation."""
    if gap >= PAD_GAP[0][1]:
        return PAD_GAP[0][0]
    for (k0, g0), (k1, g1) in zip(PAD_GAP, PAD_GAP[1:]):
        if g1 <= gap <= g0:
            return k0 + (k1 - k0) * (g0 - gap) / (g0 - g1)
    return PAD_GAP[-1][0]


def gap_for_knuckle(knuckle: float) -> float:
    """Invert the above: how far apart the pads are at a given angle."""
    if knuckle <= PAD_GAP[0][0]:
        return PAD_GAP[0][1]
    for (k0, g0), (k1, g1) in zip(PAD_GAP, PAD_GAP[1:]):
        if k0 <= knuckle <= k1:
            return g0 + (g1 - g0) * (knuckle - k0) / (k1 - k0)
    return PAD_GAP[-1][1]


def wrap_to_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi].

    THE SINGLE MOST USEFUL LINE IN THIS FILE, and it took a failed run to
    find, so it is worth saying what it does.

    Every UR5e joint has limits of +/-2pi, so for any reachable pose there
    are joint values differing by a full turn that put the flange in exactly
    the same place -- same forward kinematics, same collisions, same
    everything. MoveIt's IK returns whichever of them it happens to land on,
    and it lands on ones near +/-2pi often.

    A configuration sitting against a limit cannot be followed through. The
    first run of this sequence failed with

        Cartesian plan "opener run-in" only reached 0.57 of the path

    on a 100mm straight line, from a pose goal that had put b_wrist_2 at
    -6.07 rad against a -6.283 limit. There was nothing wrong with the
    target and IK existed at every point along it; the arm simply had 0.2 rad
    of room left in the direction it needed to turn. Wrapping the same
    solution to -6.07 + 2pi = 0.21 -- the identical pose -- took the same
    path to 1.00.

    So: solve IK, wrap, and move there in joint space. That also makes the
    approach configuration deterministic instead of whatever the sampling
    planner picked this time, which is worth having on its own.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_at(xyz, quat=SIDE_QUAT) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quat
    return pose


def block_on(future, timeout_sec: float):
    """Wait for a future without spinning anything.

    The node's own MultiThreadedExecutor is spinning in another thread and is
    what will service the callback that completes it; spinning here as well
    deadlocks.
    """
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    done.wait(timeout=timeout_sec)
    return future.result() if future.done() else None


class Arm:
    """Drives one UR5e + 2F-85 through move_group and the gripper action.

    `prefix` is the tf_prefix the description gave this arm: '' for arm A and
    'b_' for arm B. Everything else is derived from it, which is the point --
    there is no table mapping arms to group names to controller names to
    frames that could be filled in inconsistently.
    """

    def __init__(self, node, prefix: str, label: str, cb_group,
                 home_joints=None):
        self.node = node
        self.prefix = prefix
        self.label = label
        # Seed for IK. A seed matters: the solver returns the branch nearest
        # to it, so seeding from a fixed, sane configuration is what makes
        # the same target give the same answer run after run. Seeding from
        # the live state instead would make it depend on what the arm did
        # last.
        self.home_joints = list(home_joints) if home_joints else None
        self.joints = [prefix + j for j in JOINT_SUFFIXES]
        self.group = prefix + 'ur_manipulator'
        self.eef_link = prefix + 'tool0'
        self.frame = prefix + 'base_link'
        self.gripper_joint = prefix + 'robotiq_85_left_knuckle_joint'

        self.move_group_client = ActionClient(
            node, MoveGroup, 'move_action', callback_group=cb_group)
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, 'execute_trajectory', callback_group=cb_group)
        self.gripper_client = ActionClient(
            node, GripperCommand, f'{prefix}gripper_controller/gripper_cmd',
            callback_group=cb_group)
        # Not used to send anything: it exists so this arm can be asked
        # whether its trajectory controller is up. move_group talks to the
        # same server, and it does not wait for it -- see wait_for_controllers.
        self.control_client = ActionClient(
            node, FollowJointTrajectory,
            f'{prefix}ur_arm_controller/follow_joint_trajectory',
            callback_group=cb_group)
        self.cartesian_client = node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=cb_group)
        self.ik_client = node.create_client(
            GetPositionIK, 'compute_ik', callback_group=cb_group)
        self.validity_client = node.create_client(
            GetStateValidity, 'check_state_validity', callback_group=cb_group)
        self.fk_client = node.create_client(
            GetPositionFK, 'compute_fk', callback_group=cb_group)
        # The last thing this arm logged as an error, so a caller that only
        # got a coarse "could not pick up the opener" back from a failed
        # sub-call has somewhere to find out why. Nothing reads this today
        # except bartender_api's error-taxonomy classifier; open_action_server
        # discarded it before that existed, which is a real gap -- see
        # docs/CONTROL_API.md.
        self.last_error = None

    # -- state ------------------------------------------------------------

    def _log(self):
        return self.node.get_logger()

    def _error(self, message):
        """Log an error and remember it verbatim as `last_error`.

        Changes nothing about what gets logged or when -- every call site
        here used to say self._log().error(message) directly. It exists so
        the specific reason a grasp or a move failed survives past the
        boolean/None a caller like open_action_server sees, instead of being
        lost the instant the log line scrolls by.
        """
        self.last_error = message
        self._log().error(message)

    def joint_state(self):
        return self.node.joint_state

    def gripper_position(self) -> float:
        js = self.joint_state()
        if js is None or self.gripper_joint not in js.name:
            return float('nan')
        return js.position[js.name.index(self.gripper_joint)]

    def wait_until_settled(self) -> None:
        """Block until every joint of THIS arm reports near-zero velocity.

        Per-arm rather than for the whole robot, which matters here in a way
        it did not with one arm: the other arm is usually holding a bottle
        still while this one moves, and waiting for it too would be waiting
        for nothing. It also has to be per-arm in the other direction --
        planning a segment for this arm while it is still coasting gets the
        trajectory rejected for deviating from the current state.
        """
        deadline = time.time() + ARM_SETTLE_TIMEOUT_S
        while time.time() < deadline:
            js = self.joint_state()
            if js is not None and len(js.velocity) >= len(js.name):
                try:
                    speeds = [abs(js.velocity[js.name.index(j)])
                              for j in self.joints]
                except ValueError:
                    speeds = []
                if speeds and max(speeds) < ARM_SETTLE_VELOCITY:
                    return
            time.sleep(0.05)
        self._log().warn(f'{self.label}: still moving after '
                         f'{ARM_SETTLE_TIMEOUT_S:.1f}s; planning anyway')

    # -- motion -----------------------------------------------------------

    def move_to_joints(self, name: str, positions) -> bool:
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest(
            group_name=self.group,
            goal_constraints=[Constraints(joint_constraints=[
                JointConstraint(joint_name=joint, position=float(pos),
                                tolerance_above=JOINT_TOLERANCE,
                                tolerance_below=JOINT_TOLERANCE, weight=1.0)
                for joint, pos in zip(self.joints, positions)])],
            allowed_planning_time=PLANNING_TIME_S,
            num_planning_attempts=5,
            max_velocity_scaling_factor=JOINT_MOVE_SCALING,
            max_acceleration_scaling_factor=JOINT_MOVE_SCALING,
        )
        goal.planning_options = PlanningOptions(plan_only=False)

        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self._error('move_action server not available')
            return False

        # RRTConnect is randomised and regularly returns a path that only
        # fails validation after time-parameterisation. Resampling clears it.
        # Settle before EVERY attempt: a failed attempt leaves the arm
        # part-way along the trajectory it abandoned, and retrying instantly
        # preempts a move still in progress.
        for attempt in range(PLAN_ATTEMPTS):
            self.wait_until_settled()
            handle = block_on(self.move_group_client.send_goal_async(goal),
                              timeout_sec=15.0)
            if handle is None or not handle.accepted:
                continue
            result = block_on(handle.get_result_async(), timeout_sec=60.0)
            if result is not None and result.result.error_code.val == 1:
                return True
            code = result.result.error_code.val if result is not None else '-'
            self._log().warn(
                f'{self.label}: plan to "{name}" attempt '
                f'{attempt + 1}/{PLAN_ATTEMPTS} failed (error_code {code})')
            # Back off. Some failures cost nothing to produce -- a controller
            # that is not up yet returns instantly -- and retrying at that
            # rate spends the whole budget inside a third of a second.
            time.sleep(RETRY_BACKOFF_S)
        self._error(f'{self.label}: MoveGroup goal to "{name}" failed')
        return False

    def flange_position(self):
        """Where this arm's flange actually is, in its own base frame.

        Asks move_group rather than working it out, so it is the same
        kinematics everything else here is planned against. Used to check
        that a move arrived, which is not the same question as whether it
        was accepted: a Cartesian segment is allowed to come back at
        MIN_CARTESIAN_FRACTION, and on a 215mm descent that is 10mm of
        "success" -- enough to put the pads on the wrong feature of a
        bottle.
        """
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            return None
        js = self.joint_state()
        if js is None:
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = self.frame
        request.fk_link_names = [self.eef_link]
        request.robot_state.joint_state = js
        response = block_on(self.fk_client.call_async(request), timeout_sec=10.0)
        if response is None or not response.pose_stamped:
            return None
        where = response.pose_stamped[0].pose.position
        return (where.x, where.y, where.z)

    def arrived_at(self, xyz, tolerance=ARRIVAL_TOLERANCE) -> bool:
        """Report whether the flange got to within `tolerance` of xyz."""
        where = self.flange_position()
        if where is None:
            self._log().warn(f'{self.label}: cannot check where the flange is')
            return True
        off = math.dist(where, xyz)
        if off > tolerance:
            self._error(
                f'{self.label}: flange is {off * 1000:.1f}mm from where it '
                f'was sent -- at {tuple(round(v, 4) for v in where)} instead '
                f'of {tuple(round(v, 4) for v in xyz)}')
            return False
        return True

    def wait_for_controllers(self, timeout_sec: float = 30.0) -> bool:
        """Block until this arm's controllers are actually reachable.

        move_group does NOT wait. It will happily plan a trajectory and then
        abort the execution with

            Action client not connected to action server:
            b_ur_arm_controller/follow_joint_trajectory

        which arrives back here as a bare CONTROL_FAILED, and since planning
        succeeded it looks like the plan was the problem. Worse, it costs
        nothing to produce, so the retry loop burned all six attempts in
        0.3 seconds and gave up on a goal that would have worked a moment
        later. With two arms there is simply more to come up, and arm B is
        the one spawned last.
        """
        for client, what in ((self.control_client, 'arm controller'),
                             (self.gripper_client, 'gripper controller')):
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self._error(f'{self.label}: {what} never appeared')
                return False
        return True

    def go_home(self) -> bool:
        if self.home_joints is None:
            return True
        return self.move_to_joints(f'{self.prefix}home', self.home_joints)

    def _ik_once(self, xyz, quat, seed):
        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.group
        ik.ik_link_name = self.eef_link
        # Collision-aware. Only approach poses are solved here -- the grasp
        # itself is reached by a straight Cartesian descent from one -- so
        # nothing being solved for is deliberately inside an object's
        # envelope, and letting the solver reject its own bad branches is
        # free. Measured: with this off, the solver returned a state with
        # base_link_inertia against upper_arm_link, and OMPL then spent six
        # attempts reporting "Unable to sample any valid states for goal
        # tree", which does not mention collision anywhere.
        ik.avoid_collisions = True
        ik.timeout.sec = 2
        state = RobotState()
        state.joint_state.name = list(self.joints)
        state.joint_state.position = [float(v) for v in seed]
        ik.robot_state = state
        ik.pose_stamped.header.frame_id = self.frame
        ik.pose_stamped.pose = pose_at(xyz, quat)
        request.ik_request = ik

        response = block_on(self.ik_client.call_async(request), timeout_sec=10.0)
        if response is None or response.error_code.val != 1:
            return None
        js = response.solution.joint_state
        return [wrap_to_pi(js.position[js.name.index(j)]) for j in self.joints]

    def state_is_valid(self, joints) -> bool:
        """Report whether a configuration is free of collisions.

        Belt and braces over the solver's own avoid_collisions, and not
        redundant: the two ask different services and the IK one has been
        seen to pass a state that this rejects.
        """
        if not self.validity_client.wait_for_service(timeout_sec=10.0):
            return True
        request = GetStateValidity.Request()
        request.group_name = self.group
        request.robot_state.joint_state.name = list(self.joints)
        request.robot_state.joint_state.position = [float(v) for v in joints]
        response = block_on(self.validity_client.call_async(request),
                            timeout_sec=10.0)
        return True if response is None else response.valid

    def solve_ik_candidates(self, xyz, quat=SIDE_QUAT, seed=None,
                            attempts=IK_ATTEMPTS):
        """Distinct valid IK solutions for a pose, nearest the seed first."""
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self._error('compute_ik not available')
            return []
        base_seed = list(seed or self.home_joints or [0.0] * 6)
        rng = random.Random(0)          # reproducible run to run
        found = []
        for attempt in range(attempts):
            trial = (base_seed if attempt == 0 else
                     [v + rng.uniform(-IK_SEED_JITTER, IK_SEED_JITTER)
                      for v in base_seed])
            solution = self._ik_once(xyz, quat, trial)
            if solution is None or not self.state_is_valid(solution):
                continue
            if any(max(abs(a - b) for a, b in zip(solution, seen)) < 0.02
                   for seen in found):
                continue
            found.append(solution)
        found.sort(key=lambda sol: sum(abs(wrap_to_pi(a - b))
                                       for a, b in zip(sol, base_seed)))
        return found

    def _cartesian_path(self, poses, start_joints=None):
        """Ask move_group to lay out a straight-line path, without moving.

        start_joints says "suppose the arm were HERE". That is the opposite
        of what follow_cartesian does, and deliberately: there, the start
        state is left empty so move_group seeds from the live robot, because
        anything else races the arm settling and gets the trajectory
        rejected. Here nothing is executed, so stating a hypothetical start
        is exactly the point -- it is how an approach configuration can be
        judged before the arm is sent to it.
        """
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            return None
        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame
        request.group_name = self.group
        request.link_name = self.eef_link
        request.max_step = CARTESIAN_STEP
        request.jump_threshold = 0.0
        request.avoid_collisions = False
        request.waypoints = list(poses)
        if start_joints is not None:
            request.start_state.joint_state.name = list(self.joints)
            request.start_state.joint_state.position = [
                float(v) for v in start_joints]
        return block_on(self.cartesian_client.call_async(request),
                        timeout_sec=30.0)

    def cartesian_fraction(self, poses, start_joints=None) -> float:
        """How much of a straight-line path is reachable, without moving."""
        response = self._cartesian_path(poses, start_joints)
        return -1.0 if response is None else response.fraction

    def lowest_link_along(self, poses, start_joints=None):
        """Lowest link origin the arm reaches while running a straight line.

        Answers "and does it take the rest of the arm through the bar?",
        which the fraction on its own does not -- see DESCENT_FLOOR_Z for the
        run this was written for. Returns None when it cannot tell, and the
        caller treats that as "no objection", because refusing to move on a
        failed FK call would strand the sequence on a service hiccup.
        """
        response = self._cartesian_path(poses, start_joints)
        if response is None or response.fraction < MIN_CARTESIAN_FRACTION:
            return None
        points = response.solution.joint_trajectory.points
        names = list(response.solution.joint_trajectory.joint_names)
        if not points or not names:
            return None
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            return None
        links = [self.prefix + ln for ln in DESCENT_CHECK_LINKS]
        step = max(1, len(points) // DESCENT_SAMPLES)
        # Always include the last point: it is the grasp itself, and it is
        # the one waypoint the sampling must not skip.
        sampled = list(points[::step]) + [points[-1]]
        lowest, where = None, None
        for point in sampled:
            request = GetPositionFK.Request()
            request.header.frame_id = self.frame
            request.fk_link_names = links
            request.robot_state.joint_state.name = names
            request.robot_state.joint_state.position = list(point.positions)
            reply = block_on(self.fk_client.call_async(request),
                             timeout_sec=10.0)
            if reply is None or not reply.pose_stamped:
                return None
            for link, stamped in zip(reply.fk_link_names, reply.pose_stamped):
                z = stamped.pose.position.z
                if lowest is None or z < lowest:
                    lowest, where = z, link
        if lowest is not None and lowest < DESCENT_FLOOR_Z:
            self._log().warn(
                f'{self.label}: this descent would put {where} '
                f'{(DESCENT_FLOOR_Z - lowest) * 1000:.1f}mm under the '
                f'counter top')
        return lowest

    def _descent_clears_counter(self, descent, name, index, total,
                                start_joints=None) -> bool:
        """Reject an approach whose descent would go through the bar top.

        Being able to REACH the descent and being able to run it are
        different questions, and until this was added only the first was
        asked. See DESCENT_FLOOR_Z.
        """
        lowest = self.lowest_link_along(descent, start_joints)
        if lowest is None or lowest >= DESCENT_FLOOR_Z:
            return True
        self._log().warn(
            f'{self.label}: "{name}" branch {index + 1} of {total} reaches '
            f'the descent but runs it through the counter; trying the next')
        return False

    def approach_then(self, name: str, approach_xyz, then_xyz,
                      quat=SIDE_QUAT) -> bool:
        """Go to an approach pose FROM WHICH the following move will work.

        The move that follows every approach in this sequence is a straight
        descent onto the thing being picked up, and whether that descent is
        possible depends on which IK branch the approach landed in. Being
        valid and being off the joint limits is not enough -- runs have
        failed here with "Cartesian plan beer descend only reached 0.04 of
        the path" from an approach pose that planned and executed perfectly
        well.

        There is no need to guess, because compute_cartesian_path will answer
        the question for a hypothetical start state. So: enumerate the
        approach configurations, ask each one whether the descent works from
        there, and go to the first that says yes. If none does, fall back to
        the nearest valid one and let the descent report the failure itself,
        which at least fails where the problem is.
        """
        descent = [pose_at(then_xyz, quat)]
        candidates = self.solve_ik_candidates(approach_xyz, quat)
        if not candidates:
            self._log().warn(f'{self.label}: no IK at all for "{name}"; '
                             f'falling back to a pose goal')
            return self._move_to_pose_goal(name, approach_xyz, quat)
        for index, solution in enumerate(candidates):
            if self.cartesian_fraction(descent, solution) < MIN_CARTESIAN_FRACTION:
                continue
            if not self._descent_clears_counter(descent, name, index,
                                                len(candidates), solution):
                continue
            if not self.move_to_joints(name, solution):
                continue
            # Ask again, now from where the arm ACTUALLY is. The prediction
            # was made about a configuration; what arrives is that
            # configuration within the goal tolerance, by whatever path the
            # planner chose and however the controller tracked it. Usually
            # the difference does not matter and occasionally it does -- a
            # descent verified at 1.00 has come back at 0.05 once the arm was
            # standing there. Cheaper to ask than to find out by moving.
            self.wait_until_settled()
            if (self.cartesian_fraction(descent) >= MIN_CARTESIAN_FRACTION
                    and self._descent_clears_counter(descent, name, index,
                                                     len(candidates))):
                if index:
                    self._log().info(
                        f'{self.label}: "{name}" used IK branch {index + 1} '
                        f'of {len(candidates)}; the nearer ones could not '
                        f'reach the descent')
                return True
            self._log().warn(
                f'{self.label}: "{name}" branch {index + 1} of '
                f'{len(candidates)} cannot run the descent from where it '
                f'actually landed; trying the next')
        self._error(
            f'{self.label}: none of {len(candidates)} approach '
            f'configurations for "{name}" can run the descent')
        return False

    def solve_ik(self, xyz, quat=SIDE_QUAT, seed=None, attempts=IK_ATTEMPTS):
        """IK for a tool0 pose: valid, wrapped, and as near the seed as found.

        Three things happen here that a single compute_ik call does not do,
        and each of them was put in after a specific failure.

        WRAPPED, because a solution pressed against a +/-2pi joint limit
        cannot be followed through by the straight-line move that always
        comes next. See wrap_to_pi.

        CHECKED, because the solver returns branches that fold the arm into
        its own base and reports success for them. OMPL then fails to plan to
        that goal and says only "Unable to sample any valid states for goal
        tree", which takes a while to recognise as "your goal is in
        collision".

        NEAREST THE SEED, because among the branches that are fine, the one
        closest to the arm's rest posture is the one least likely to be
        awkward in some way this has not thought of -- and, more practically,
        it makes the answer stable. The solver is randomised, so the same
        target could otherwise give a different posture on every run and a
        sequence that works four times in five for no visible reason.

        The seed is jittered after the first attempt because with a fixed
        seed the solver is deterministic, and one attempt eight times over is
        not a search.
        """
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self._error('compute_ik not available')
            return None
        base_seed = list(seed or self.home_joints or [0.0] * 6)
        rng = random.Random(0)          # reproducible run to run
        best = None
        for attempt in range(attempts):
            trial = (base_seed if attempt == 0 else
                     [v + rng.uniform(-IK_SEED_JITTER, IK_SEED_JITTER)
                      for v in base_seed])
            solution = self._ik_once(xyz, quat, trial)
            if solution is None or not self.state_is_valid(solution):
                continue
            distance = sum(abs(wrap_to_pi(a - b))
                           for a, b in zip(solution, base_seed))
            if best is None or distance < best[0]:
                best = (distance, solution)
        if best is None:
            self._log().warn(f'{self.label}: no usable IK for '
                             f'{tuple(round(v, 4) for v in xyz)} in '
                             f'{attempts} attempts')
            return None
        return best[1]

    def _move_to_pose_goal(self, name: str, xyz, quat=SIDE_QUAT) -> bool:
        """Plan a collision-checked path to a tool0 pose.

        A POSE goal rather than a joint goal, which is the difference between
        this and how bartender_pour transits: that server moves between
        taught joint configurations, which exist because the user recorded
        them on a pendant. The beer and opener stations have no taught points
        -- they are new -- so the goal has to be stated in Cartesian space
        and MoveIt's IK asked to find a configuration. That is also why
        kinematics.yaml needed an entry for b_ur_manipulator: a group with no
        solver silently has no IK, and a pose goal against it fails with
        nothing useful in the message.

        The tolerances are 5mm and 0.02 rad. Tight, because every one of
        these goals is followed by a straight-line Cartesian run-in that
        assumes it started where it was sent, and a loose pose goal pushes
        that error into the run-in where the clearances are millimetres.
        """
        pos = PositionConstraint()
        pos.header.frame_id = self.frame
        pos.link_name = self.eef_link
        pos.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])]
        pos.constraint_region.primitive_poses = [pose_at(xyz, (0.0, 0.0, 0.0, 1.0))]
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header.frame_id = self.frame
        orient.link_name = self.eef_link
        (orient.orientation.x, orient.orientation.y,
         orient.orientation.z, orient.orientation.w) = quat
        orient.absolute_x_axis_tolerance = 0.02
        orient.absolute_y_axis_tolerance = 0.02
        orient.absolute_z_axis_tolerance = 0.02
        orient.weight = 1.0

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest(
            group_name=self.group,
            goal_constraints=[Constraints(position_constraints=[pos],
                                          orientation_constraints=[orient])],
            allowed_planning_time=PLANNING_TIME_S,
            num_planning_attempts=5,
            max_velocity_scaling_factor=JOINT_MOVE_SCALING,
            max_acceleration_scaling_factor=JOINT_MOVE_SCALING,
        )
        goal.planning_options = PlanningOptions(plan_only=False)

        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self._error('move_action server not available')
            return False
        for attempt in range(PLAN_ATTEMPTS):
            self.wait_until_settled()
            handle = block_on(self.move_group_client.send_goal_async(goal),
                              timeout_sec=15.0)
            if handle is None or not handle.accepted:
                continue
            result = block_on(handle.get_result_async(), timeout_sec=60.0)
            if result is not None and result.result.error_code.val == 1:
                return True
            code = result.result.error_code.val if result is not None else '-'
            self._log().warn(
                f'{self.label}: pose goal "{name}" attempt '
                f'{attempt + 1}/{PLAN_ATTEMPTS} failed (error_code {code})')
        self._error(f'{self.label}: pose goal "{name}" failed')
        return False

    def move_cartesian(self, xyz, quat=SIDE_QUAT, label: str = '',
                       may_stall: bool = False) -> bool:
        return self.follow_cartesian([pose_at(xyz, quat)], label, may_stall)

    def follow_cartesian(self, poses, label: str = '',
                         may_stall: bool = False) -> bool:
        """Straight-line move of this arm's tool0 through `poses`.

        may_stall says the move is EXPECTED not to arrive: it is pushing
        against something. The plan still has to come back complete -- a
        short plan means the path itself was unreachable, which is a genuine
        error however it ends -- but a controller that reports the execution
        aborted is then the intended outcome and is logged, not failed on.
        """
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            self._error('compute_cartesian_path not available')
            return False
        self.wait_until_settled()

        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame
        # start_state left empty so move_group seeds from its own current
        # state; seeding from a cached /joint_states races the arm settling
        # and gets the trajectory rejected for deviating from it.
        request.group_name = self.group
        request.link_name = self.eef_link
        request.max_step = CARTESIAN_STEP
        request.jump_threshold = 0.0
        # Collision checking off, as in the pour: these segments are
        # deliberately run within millimetres of the objects being handled,
        # and the straight line itself is what keeps them off everything
        # else. The joint-space transits ARE checked, against the scene
        # published in open_action_server.
        request.avoid_collisions = False
        request.waypoints = list(poses)

        response = block_on(self.cartesian_client.call_async(request),
                            timeout_sec=30.0)
        if response is None:
            self._error(f'{self.label}: Cartesian plan "{label}" timed out')
            return False
        if response.fraction < MIN_CARTESIAN_FRACTION:
            self._error(
                f'{self.label}: Cartesian plan "{label}" only reached '
                f'{response.fraction:.2f} of the path')
            return False

        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self._error('execute_trajectory server not available')
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        handle = block_on(self.execute_client.send_goal_async(goal),
                          timeout_sec=15.0)
        if handle is None or not handle.accepted:
            self._error(f'{self.label}: execution "{label}" rejected')
            return False
        result = block_on(handle.get_result_async(), timeout_sec=60.0)
        ok = result is not None and result.result.error_code.val == 1
        if not ok and may_stall:
            code = result.result.error_code.val if result is not None else '-'
            self._log().info(
                f'{self.label}: "{label}" did not arrive (error_code {code}), '
                f'which is what pushing against something looks like')
            return True
        if not ok:
            self._error(f'{self.label}: execution "{label}" failed')
        return ok

    # -- gripper ----------------------------------------------------------

    def _send_gripper(self, position: float):
        """Send one gripper goal, refusing any that would park on the stop.

        Refused rather than clamped, in line with the rest of the repo: a
        caller asking for 0.0 has asked for something that kills the joint
        for the rest of the run (see GRIPPER_LOWER_LIMIT), and silently
        sending 0.02 instead would hide that from whoever wrote it. This
        guard has no legitimate caller today; it is here so the next one
        finds out at once instead of after a half-hour sim run.
        """
        if not GRIPPER_OPEN_POS <= float(position) <= GRIPPER_UPPER_LIMIT:
            self._error(
                f'{self.label}: refusing gripper command {position:.4f} rad. '
                f'The usable band is {GRIPPER_OPEN_POS:.2f}..'
                f'{GRIPPER_UPPER_LIMIT:.2f}; resting on the lower joint '
                f'limit stops the knuckle responding for the rest of the run.')
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        return block_on(self.gripper_client.send_goal_async(goal),
                        timeout_sec=10.0)

    def wait_for_gripper(self, command=None,
                         timeout_s=GRIPPER_STEP_TIMEOUT_S) -> float:
        """Wait for the knuckle to arrive or to stop, and return where it is.

        Two different endings, and telling them apart is the point.

        ARRIVED: within GRIPPER_ARRIVED of `command`. Returns immediately,
        which is what keeps a thirty-step close down to a few seconds.

        STOPPED: the position has not changed by more than
        GRIPPER_SETTLED_MOVE across a whole GRIPPER_SETTLED_WINDOW_S. This is
        the expensive branch and it has to be, because the thing it is
        distinguishing itself from is a joint creeping slowly towards its
        command. Watch for too short a time and a creep reads as a stop --
        measured, that accepted grips with 0.9mm of bite on a round neck,
        which held until the other arm pushed on them and then let the bottle
        slide 28mm.

        It is also POSITION and not velocity. At the top of an overshoot the
        velocity is exactly zero, so a velocity test reports "settled" at the
        worst possible instant: a pre-close commanded to 0.351 rad was read
        back as 0.508.

        A short lead-in first, because the joint is stationary at the instant
        the goal is sent and every test for "has stopped" is true then.
        """
        time.sleep(0.12)
        deadline = time.time() + timeout_s
        history = []                    # (when, where)
        while time.time() < deadline:
            now, where = time.time(), self.gripper_position()
            if not math.isnan(where):
                if command is not None and abs(where - command) < GRIPPER_ARRIVED:
                    return where
                history.append((now, where))
                window = [w for t, w in history
                          if t >= now - GRIPPER_SETTLED_WINDOW_S]
                if (history[0][0] <= now - GRIPPER_SETTLED_WINDOW_S
                        and max(window) - min(window) < GRIPPER_SETTLED_MOVE):
                    return where
            time.sleep(0.02)
        return self.gripper_position()

    def grasp(self, object_width: float):
        """Close onto something and return the angle the fingers stopped at.

        None if they never stopped -- i.e. nothing was between them.

        CLOSED LOOP, and that is the whole point of it. bartender_pour closes
        to a per-object clamp angle chosen in advance, tuned by hand against
        measured stalls, with a note beside each one saying what happens if
        it is wrong in either direction. That works, but it only works for
        objects somebody has already measured, and it depends on a fixed
        dwell being long enough for the fingers to keep up.

        They are not always. The knuckle lags a moving command by about 30%
        while it is still closing, and two runs here read the joint before it
        had moved at all -- once reporting "clamped at -0.0006 rad" for a
        grasp that had not happened, which then passed the check that exists
        to catch precisely that, because -0.0006 is a long way from the
        command and a long way from the command is what a real grasp looks
        like. A stale reading is indistinguishable from a good grasp if all
        you compare it against is the number you asked for.

        So: step the command up, wait for each step to actually ARRIVE, and
        watch for the step that does not. A step the fingers cannot follow is
        an object. That reading cannot be stale, because staleness shows up
        as "not arrived yet" and is waited out rather than acted on.

        The fast part of the close is skipped: the pads start 85mm apart and
        nothing here is near that wide, so the first move goes straight to
        GRIPPER_PRE_CLOSE_GAP wider than the object, in free air, in one
        command.
        """
        target = max(GRIPPER_OPEN_POS,
                     knuckle_for_gap(object_width + GRIPPER_PRE_CLOSE_GAP))
        command = max(GRIPPER_OPEN_POS, self.gripper_position())
        if math.isnan(command):
            command = GRIPPER_OPEN_POS
        # Where the fingers were before ANY of this, so that "the joint has
        # not moved" can be told from "the joint moved and then stopped".
        # Both fail the same test further down and they are completely
        # different faults. It has to be read here and not after the
        # pre-close, because the pre-close is most of the travel: measured
        # against the pre-close instead, a gripper that had closed 0.345 rad
        # onto the wrong thing still looked like one that had never moved.
        began_at = command
        # The loop counts on the COMMANDED angle, never on the measured one.
        # Stepping the command up from wherever the fingers have got to reads
        # as the obvious thing to do and does not terminate: if something
        # stops them, the measurement stops advancing, the next command is
        # the same command, and the loop sends it forever. That is not
        # hypothetical -- it sat there issuing a gripper goal every 0.35s for
        # five minutes before anything timed out.
        reached = command
        while command < target - 1e-6:
            command = min(target, command + GRIPPER_FAST_STEP)
            handle = self._send_gripper(command)
            if handle is None or not handle.accepted:
                self._error(f'{self.label}: gripper goal rejected')
                return None
            reached = self.wait_for_gripper(command, timeout_s=2.5)
            if command - reached > GRIPPER_STALL_GAP:
                # Stopped short of a width that should still be clear air.
                # Whatever that is, the careful loop below is the part that
                # can tell a grasp from an obstacle, so let it look.
                break
        command = reached
        self._log().info(
            f'{self.label}: pre-closed to {command:.3f} rad '
            f'({gap_for_knuckle(command) * 1000:.1f}mm) for a '
            f'{object_width * 1000:.1f}mm object')

        previous = command
        while command < GRIPPER_FULLY_CLOSED:
            command = min(GRIPPER_FULLY_CLOSED, command + GRIPPER_CLOSE_STEP)
            handle = self._send_gripper(command)
            if handle is None or not handle.accepted:
                self._error(f'{self.label}: gripper goal rejected')
                return None
            reached = self.wait_for_gripper(command)
            held = gap_for_knuckle(reached)
            advance, previous = reached - previous, reached
            # IS THE GRIPPER MOVING AT ALL? Asked first, before anything
            # about widths, because a gripper that never budged answers every
            # other question misleadingly.
            #
            # This check used to sit BELOW the width test, which made it
            # unreachable in precisely the case it was written for: a gripper
            # jammed at its open stop holds the pads 85.0mm apart, wider than
            # any object here, so every step took the `continue` and the loop
            # ran to GRIPPER_FULLY_CLOSED before reporting "closed all the
            # way without meeting anything". Traced on an opener pick that
            # failed with the knuckle reading -0.000 from the pre-close
            # onwards: the fingers had never moved, and the message still
            # described where the opener was.
            if command - reached > GRIPPER_UNRESPONSIVE:
                # ...but WHICH of the two? The command outrunning the joint
                # by this much has two causes that want opposite
                # investigations, and calling both of them "not following"
                # sent one straight to the wrong place: an opener pick
                # stopped with the pads 49.1mm apart on a 24.0mm shaft --
                # the fingers had closed 0.345 rad perfectly well and then
                # met something that was not the opener -- and the message
                # blamed the gripper.
                if reached - began_at < GRIPPER_CLOSE_STEP:
                    self._error(
                        f'{self.label}: gripper is not moving -- commanded '
                        f'{command:.3f} rad and the joint is at '
                        f'{reached:.3f}, {(command - reached):.3f} behind, '
                        f'having left {began_at:.3f} where the close began. '
                        f'The pads are {held * 1000:.1f}mm apart and nothing '
                        f'{object_width * 1000:.1f}mm wide can be stopping '
                        f'them; it is the gripper not following.')
                else:
                    self._error(
                        f'{self.label}: fingers closed from {began_at:.3f} to '
                        f'{reached:.3f} rad and stopped there, {held * 1000:.1f}'
                        f'mm apart, with the command {(command - reached):.3f} '
                        f'past them. The gripper is working -- but nothing '
                        f'{object_width * 1000:.1f}mm wide is that far apart, '
                        f'so they are on something else and the arm is '
                        f'probably not where it should be.')
                return None
            # Nothing counts as contact until the pads are at least as close
            # together as the object is wide. Before that, a joint short of
            # its command is a joint that has not caught up.
            if held > object_width + GRASP_LOOSE:
                continue
            if (command - reached > GRIPPER_STALL_GAP
                    and advance < GRIPPER_CONFIRM_MOVE):
                if held < object_width - GRASP_PENETRATION:
                    self._error(
                        f'{self.label}: fingers stopped at {reached:.4f} rad, '
                        f'which is {held * 1000:.1f}mm apart, but the object '
                        f'is {object_width * 1000:.1f}mm -- so whatever '
                        f'stopped them, it was not that')
                    return None
                # Keep squeezing: the final goal is left ACTIVE and past the
                # stall on purpose. A closing goal that has bottomed out can
                # never report success, and it is that unfinished goal that
                # holds the grip force. Nothing may block on its result.
                self._send_gripper(
                    min(GRIPPER_UPPER_LIMIT, reached + GRIPPER_SQUEEZE))
                # ...and then take the grasp from where the fingers SETTLE
                # under that squeeze, not from where they first touched.
                #
                # The difference is the whole grip. First contact on a round
                # neck is a touch, not a hold: measured, grasps taken there
                # read 1.1mm of bite, survived the lift, and then let the
                # bottle slide 21mm down the pads when the other arm pushed
                # on it -- which the gate caught, correctly, as a bottle that
                # had stopped being held. Left to settle under the squeeze
                # the same close goes on to about 4mm of bite and does not
                # move. So the stall is what the fingers do next, not what
                # they did the instant they arrived.
                settled = self.wait_for_gripper(timeout_s=GRIPPER_SQUEEZE_S)
                if not math.isnan(settled) and settled > reached:
                    reached = settled
                bite = object_width - gap_for_knuckle(reached)
                self._log().info(
                    f'{self.label}: stalled at {reached:.4f} rad, pads '
                    f'{gap_for_knuckle(reached) * 1000:.1f}mm apart '
                    f'({object_width * 1000:.1f}mm expected, '
                    f'{bite * 1000:.1f}mm of bite)')
                return reached
        self._error(
            f'{self.label}: fingers closed all the way to {command:.2f} rad '
            f'(joint at {reached:.3f}, pads '
            f'{gap_for_knuckle(reached) * 1000:.1f}mm apart) without meeting '
            f'anything {object_width * 1000:.1f}mm wide')
        return None

    def command_gripper(self, position: float, clamp: bool = False,
                        ramp: bool = False) -> bool:
        """Drive this arm's gripper, walking the move in small steps.

        Kept for OPENING, which has no object to find and so has nothing to
        close the loop on; `grasp` is what closes onto something. Same
        contract as bartender_pour's: `clamp` leaves the last goal running,
        `ramp` waits for completion and lets things settle. Use ramp for any
        move that starts in contact.
        """
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self._error(f'{self.label}: gripper action not available')
            return False

        start = self.gripper_position()
        if math.isnan(start):
            start = GRIPPER_OPEN_POS
        # Floored, because `start` is only the interpolation origin and a
        # MEASURED one: the knuckle overshoots and can be read at 0.0178
        # mid-brush (see the note at the end of this method). Left alone,
        # the first interpolated step would come out under GRIPPER_OPEN_POS
        # and _send_gripper would refuse it, failing the whole command with
        # "gripper goal rejected" -- a message about the wrong thing.
        start = max(GRIPPER_OPEN_POS, start)
        steps = (max(1, int(math.ceil(abs(position - start) / GRIPPER_CLOSE_STEP)))
                 if (clamp or ramp) else 1)

        handle = None
        for i in range(1, steps + 1):
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
            handle = self._send_gripper(here)
            if handle is None or not handle.accepted:
                self._error(f'{self.label}: gripper goal rejected')
                return False
            if i < steps:
                time.sleep(GRIPPER_STEP_DWELL_S)

        if clamp:
            time.sleep(GRIPPER_SETTLE_S)
            reached = self.gripper_position()
            if abs(position - reached) < GRASP_STALL_MARGIN:
                self._error(
                    f'{self.label}: grasp failed -- fingers reached '
                    f'{reached:.4f} rad against a command of {position:.4f}, '
                    f'so nothing is between them')
                return False
            self._log().info(
                f'{self.label}: clamped at {reached:.4f} rad, '
                f'{position - reached:.4f} short of the command')
            return True

        # Verified by POSITION, not by the action result. An opening goal
        # that is preempted, or whose result is simply slow to come back,
        # reports nothing useful -- and a bare `result is None` return was
        # seen failing a stow with no message at all, after an open that had
        # in fact worked. Where the fingers are is not ambiguous.
        block_on(handle.get_result_async(), timeout_sec=20.0)
        reached = self.wait_for_gripper(position, timeout_s=3.0)
        if ramp:
            time.sleep(GRIPPER_RELEASE_SETTLE_S)
            reached = self.gripper_position()
        if math.isnan(reached) or abs(reached - position) > GRIPPER_STALL_GAP:
            self._error(
                f'{self.label}: gripper was told to go to {position:.3f} rad '
                f'and is at {reached:.3f}')
            return False
        # DID IT COME TO REST ON THE STOP ANYWAY? Nothing commands the stop
        # any more (see GRIPPER_LOWER_LIMIT), but the joint can still
        # OVERSHOOT onto it: the ramp's dwell is shorter than the time a
        # 0.02 step takes, so the command outruns the joint and it arrives
        # at the full 0.5 rad/s. Traced on two separate opens, arm B's
        # knuckle went 0.0178 -> -0.0000 -> back to 0.0200, spending 0.068s
        # of SIMULATED time under 0.015 and exactly one sample at the stop.
        # The two traces agree to the sample. Arm A, running the identical
        # code on the same runs, never went below 0.0200 at all, so this is
        # a timing race and not a property of the command.
        #
        # That brush is survivable -- arm B grasped the opener cleanly
        # later in both of those runs, and every death measured involved
        # 10s or more AT REST on the stop -- but "there is no evidence it
        # matters" is not the same as "it cannot happen".
        #
        # So the one thing that must never go unnoticed is the joint being
        # LEFT there. Said here, at the moment it happens, instead of three
        # minutes later as an unexplained "gripper not following".
        if reached < GRIPPER_LOWER_LIMIT + GRIPPER_LIMIT_MARGIN / 2.0:
            self._error(
                f'{self.label}: gripper has come to rest at {reached:.4f} rad, '
                f'on its lower stop rather than at {position:.3f}. It will '
                f'stop responding to commands from here; restart the '
                f'simulator. See GRIPPER_LOWER_LIMIT in arm.py.')
            return False
        return True

    def still_holding(self, stall: float) -> bool:
        """Report whether the object is still between the fingers.

        Measured against where the fingers ACTUALLY stopped when they took
        hold, not against what they were commanded to. If the object has gone
        the linkage closes past that angle, and it closes a long way past it
        -- the squeeze command is GRIPPER_SQUEEZE beyond the stall, and with
        nothing in the way the fingers go all of it.

        The allowance is deliberately smaller than GRIPPER_SQUEEZE, so an
        object that has been lost cannot be inside it, and comfortably larger
        than the drift seen on a good grip, which is a thousandth of a radian
        over several seconds.
        """
        reached = self.gripper_position()
        if math.isnan(reached):
            return False
        return reached - stall < GRIPPER_SQUEEZE / 2.0
