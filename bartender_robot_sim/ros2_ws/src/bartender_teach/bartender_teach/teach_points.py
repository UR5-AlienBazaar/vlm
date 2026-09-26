"""Interactive teach pendant: drive the arm, name where it is, save it.

    ros2 run bartender_teach teach

Every pose bartender_pour uses is currently a literal in its source -- the
bottles' `approach_joints`, HOME_JOINTS, GLASS_XY. Those were obtained by
running /compute_ik by hand, reading branches off the console, normalising
them, and pasting them in, which is slow and is the kind of thing that is
wrong in a way nobody notices until the arm sweeps somewhere it should not.
This does the same job by driving the robot to the pose and naming it.

What it is NOT: a replacement for the motion strategy in pour_action_server.
It records CONFIGURATIONS; what the pour cycle does between them is still that
file's business.

Safety
------
Cartesian jogs default to avoid_collisions=True, which is the opposite of what
pour_action_server does -- deliberately. That file is allowed collisions off
because its close work is straight lines computed in one shot toward a known
object. Here a person is typing distances, and a typo should be stopped by the
planner rather than by the bottle. `safety off` is available for teaching a
point that really is in contact, and it says so loudly every time it jogs.

Jogs are also bounded (MAX_JOG_MM, MAX_JOG_DEG) and REFUSED when exceeded
rather than clamped. A clamp would turn `jog z 500` -- a typo for 50 -- into a
move that silently does something other than what was typed, and the whole
point of a bounds check here is to catch the typo.

Threading
---------
The executor spins in a background thread and the prompt runs on the main
thread. Same constraint as pour_action_server: nested action calls need a
MultiThreadedExecutor with a ReentrantCallbackGroup, and blocking on a plain
threading.Event rather than spin_until_future_complete, which grabs rclpy's
process-global executor and deadlocks against our own.
"""
import math
import shlex
import sys
import threading
import time
from typing import NamedTuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, PlanningOptions, RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState

from bartender_teach.pipelines import Pipeline, PipelineError, Step
from bartender_teach.point_store import (
    Point, PointStore, PointStoreError, default_points_path, points_path_for,
)
from bartender_teach.robot_control import (
    DASHBOARD_VERBS, MODE_POWER_OFF, MODE_RUNNING, RobotControl,
)
from bartender_teach.tool_frames import (
    TOOL0, TOOLS, get_tool, quat_about, quat_rotate, rotate_about_tcp,
    tcp_from_tool0,
)

# Kept in step with pour_action_server; a point taught here is meant to be
# pasted into (or loaded by) that file, so the group, frame and joint naming
# have to be the ones it plans against.
#
# TWO ARMS. Everything below that names a joint, group, link or frame is a
# property of ONE arm, so it lives in an Arm record and the pendant holds a
# selected one rather than reading module constants. The names come from the
# description: arm A is unprefixed and arm B is under `b_`, and the reason for
# that asymmetry is in bartender_description/urdf/bartender.urdf.xacro.
#
# Note that arm B's PLANNING FRAME is b_base_link, not base_link. The two arms
# do not share an origin -- arm B sits 1.06 along and 0.80 across from arm A,
# turned to face back at it
# -- so a pose read or jogged in the wrong frame lands most of a metre from
# where it was asked for. That is the one mistake this split exists to make
# impossible.
UR_JOINTS = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)


class Arm(NamedTuple):
    """Everything about one arm that the pendant has to name."""

    key: str                # what you type: `arm a`
    label: str              # what gets printed
    prefix: str             # '' or 'b_'
    group: str
    joints: tuple
    eef_link: str
    frame: str
    gripper_joint: str
    gripper_action: str

    @classmethod
    def build(cls, key, label, prefix, group, arm_controller):
        return cls(key, label, prefix, group,
                   tuple(prefix + j for j in UR_JOINTS),
                   prefix + 'tool0', prefix + 'base_link',
                   prefix + 'robotiq_85_left_knuckle_joint',
                   arm_controller)


ARMS = {
    'a': Arm.build('a', 'arm A', '', 'ur_manipulator',
                   'gripper_controller/gripper_cmd'),
    'b': Arm.build('b', 'arm B', 'b_', 'b_ur_manipulator',
                   'b_gripper_controller/gripper_cmd'),
}
DEFAULT_ARM = ARMS['a']

# Arm A's values, kept as module names because they were the public surface of
# this module before there was a second arm -- teach_gui imports them, and so
# does anything that only ever cared about the pouring arm.
ARM_JOINTS = list(DEFAULT_ARM.joints)
ARM_GROUP_NAME = DEFAULT_ARM.group
EEF_LINK = DEFAULT_ARM.eef_link
PLANNING_FRAME = DEFAULT_ARM.frame
GRIPPER_JOINT = DEFAULT_ARM.gripper_joint

JOINT_TOLERANCE = 0.01
PLANNING_TIME_S = 5.0
PLAN_ATTEMPTS = 3
CARTESIAN_STEP = 0.005
MIN_CARTESIAN_FRACTION = 0.95
GRIPPER_MAX_EFFORT = 100.0

# Refused, not clamped -- see the module docstring.
MAX_JOG_MM = 150.0
MAX_JOG_DEG = 45.0

# WHAT "OPEN" MEANS, AND WHY IT IS NOT 0.0.
#
# robotiq_85_left_knuckle_joint is limit=[0.0, 0.8], and a knuckle left at
# rest on the LOWER limit stops responding to gripper commands for the rest
# of the simulator run: goals go on being accepted, the controller stays
# active, the joint never moves again. Measured, 0.000 was dead after a 10s
# dwell while 0.020 survived 300s. The full write-up, and the list of
# explanations it is not, is beside GRIPPER_LOWER_LIMIT in
# bartender_open/arm.py; bartender_pour carries the same pair of constants.
#
# So `open` goes to 0.02 rather than 0.0, which leaves the pads 83.2mm
# apart instead of 85.0. Anything outside the band is refused, not
# clamped, like the jogs.
GRIPPER_LOWER_LIMIT = 0.0
GRIPPER_LIMIT_MARGIN = 0.02
GRIPPER_OPEN_POS = GRIPPER_LOWER_LIMIT + GRIPPER_LIMIT_MARGIN
GRIPPER_UPPER_LIMIT = 0.8

# Wait for the arm to stop before planning anything from where it is. Not
# optional: without it, a jog issued straight after a move plans from a start
# state the arm has already left, and execute_trajectory rejects the result
# with "start point deviates from current robot state more than 0.01". That
# showed up here as every Cartesian jog reporting "execution failed" while the
# tool visibly drifted between two `state` calls. pour_action_server has the
# same wait for the same reason.
#
# Measured velocity rather than a fixed sleep, because the overshoot at the end
# of a transit has been seen to reach 0.11 rad and any constant short enough
# not to waste time was too short to cover it.
ARM_SETTLE_VELOCITY = 0.01    # rad/s, per joint
ARM_SETTLE_TIMEOUT_S = 5.0

AXES = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}


class TeachNode(Node):
    """The ROS half: reads state, and moves the arm when told to."""

    def __init__(self, cache_pose=False):
        super().__init__('bartender_teach')
        cb = ReentrantCallbackGroup()
        self._joint_state = None
        self._have_state = threading.Event()
        self._pose_cache = None
        self._pose_lock = threading.Lock()

        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10,
            callback_group=cb)
        self._move_group = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb)
        self._execute = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory', callback_group=cb)
        # One per arm. Made up front rather than on demand because an
        # ActionClient created inside a command would be discovering its
        # server while the command is already trying to use it.
        self._grippers = {
            arm.key: ActionClient(self, GripperCommand, arm.gripper_action,
                                  callback_group=cb)
            for arm in ARMS.values()
        }
        self._gripper = self._grippers[DEFAULT_ARM.key]
        self._cartesian = self.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=cb)
        self._fk = self.create_client(
            GetPositionFK, 'compute_fk', callback_group=cb)
        # Power, program, speed slider, freedrive. Real robot only; see
        # robot_control.py.
        self.robot = RobotControl(self, cb)

        # The browser front end polls state twice a second, and tool_pose()
        # blocks on a service call. Refreshing it on a timer instead keeps
        # those polls instant, and means a slow or absent move_group shows up
        # as a stale-then-empty pose rather than a page that hangs.
        # Which arm that timer polls. The browser front end drives one arm
        # at a time and the bridge keeps this in step with the selection; a
        # timer that always polled arm A would show arm A's flange while the
        # page said arm B.
        self.poll_arm = DEFAULT_ARM
        if cache_pose:
            self.create_timer(0.25, self._refresh_pose, callback_group=cb)

    def _refresh_pose(self):
        # Never queue: the timer fires every 250ms and an FK call can take
        # seconds while the arm is moving, so overlapping callbacks would pile
        # up worker threads against a service that is already struggling.
        if not self._pose_lock.acquire(blocking=False):
            return
        try:
            arm = self.poll_arm
            self._pose_cache = (arm.key, self.tool_pose(None, arm))
        except Exception:                                   # noqa: BLE001
            self._pose_cache = None
        finally:
            self._pose_lock.release()

    def cached_pose(self, arm=DEFAULT_ARM):
        """Last polled flange pose for `arm`, or a fresh one.

        A cache holding another arm's pose is worse than no cache: it is a
        plausible-looking position most of a metre from the truth. So the
        cached value is only used when it was taken for the arm being asked
        about.
        """
        cached = self._pose_cache
        if cached is not None and cached[0] == arm.key:
            return cached[1]
        return self.tool_pose(None, arm)

    def _on_joint_state(self, msg):
        self._joint_state = msg
        self._have_state.set()

    @staticmethod
    def _block_on(future, timeout_sec):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_sec):
            return None
        try:
            return future.result()
        except Exception:                                   # noqa: BLE001
            return None

    # -- state ------------------------------------------------------------

    def wait_for_state(self, timeout=10.0):
        return self._have_state.wait(timeout)

    def joints(self):
        """Every joint currently published, as name -> position."""
        msg = self._joint_state
        if msg is None:
            return {}
        return dict(zip(msg.name, msg.position))

    def arm_joints(self, arm=DEFAULT_ARM):
        """Just the six joints of `arm`, in its own order.

        Indexed by NAME, never by position in the message. /joint_states
        interleaves both arms and both grippers in registration order, so
        slicing the first six off it happens to work today and would break
        silently the moment a controller loads in a different order -- and
        with two arms it would not even be the right arm.
        """
        allj = self.joints()
        missing = [j for j in arm.joints if j not in allj]
        if missing:
            raise PointStoreError(
                f'/joint_states is not publishing {", ".join(missing)}; '
                f'it has {", ".join(sorted(allj)) or "nothing"}. Is the robot '
                f'up and are the controllers running?')
        return {j: allj[j] for j in arm.joints}

    def gripper_position(self, arm=DEFAULT_ARM):
        return self.joints().get(arm.gripper_joint)

    def wait_until_settled(self, arm=DEFAULT_ARM):
        """Block until every joint of `arm` reports (near) zero velocity."""
        deadline = time.time() + ARM_SETTLE_TIMEOUT_S
        while time.time() < deadline:
            js = self._joint_state
            if js is not None and len(js.velocity) >= len(js.name):
                try:
                    speeds = [abs(js.velocity[js.name.index(j)])
                              for j in arm.joints]
                except ValueError:
                    speeds = []
                if speeds and max(speeds) < ARM_SETTLE_VELOCITY:
                    return
            time.sleep(0.05)
        self.get_logger().warn(
            f'arm still moving after {ARM_SETTLE_TIMEOUT_S:.1f}s; '
            f'planning from here anyway')

    def tool_pose(self, joints=None, arm=DEFAULT_ARM):
        """FK of the arm's flange in its own base frame, or None.

        None is a normal answer -- move_group may not be up, and every command
        here except the Cartesian jogs works without it. Callers show what they
        can rather than failing.
        """
        if not self._fk.wait_for_service(timeout_sec=2.0):
            return None
        joints = joints or self.arm_joints(arm)
        req = GetPositionFK.Request()
        req.header.frame_id = arm.frame
        req.fk_link_names = [arm.eef_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState(
            name=list(joints), position=[float(v) for v in joints.values()])
        resp = self._block_on(self._fk.call_async(req), timeout_sec=5.0)
        if resp is None or not resp.pose_stamped or resp.error_code.val != 1:
            return None
        p = resp.pose_stamped[0].pose
        return ((p.position.x, p.position.y, p.position.z),
                (p.orientation.x, p.orientation.y,
                 p.orientation.z, p.orientation.w))

    # -- motion -----------------------------------------------------------

    def move_to_joints(self, positions, label='', arm=DEFAULT_ARM):
        """Plan and execute a joint-space move. positions is name -> radians."""
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest(
            group_name=arm.group,
            goal_constraints=[Constraints(joint_constraints=[
                JointConstraint(joint_name=n, position=float(v),
                                tolerance_above=JOINT_TOLERANCE,
                                tolerance_below=JOINT_TOLERANCE, weight=1.0)
                for n, v in positions.items()])],
            allowed_planning_time=PLANNING_TIME_S,
            num_planning_attempts=5,
        )
        goal.planning_options = PlanningOptions(plan_only=False)

        if not self._move_group.wait_for_server(timeout_sec=10.0):
            return False, 'move_action server not available'
        # RRTConnect is randomised and regularly returns a path that only
        # fails validation after time-parameterisation; resampling clears it.
        last = 'no result'
        for attempt in range(PLAN_ATTEMPTS):
            # Before EVERY attempt: a failed attempt often leaves the arm
            # part-way along the trajectory it was told to abandon, and
            # retrying instantly means the next goal preempts a move still in
            # progress and is itself invalidated for deviating from a moving
            # start state.
            self.wait_until_settled(arm)
            handle = self._block_on(
                self._move_group.send_goal_async(goal), timeout_sec=15.0)
            if handle is None or not handle.accepted:
                last = 'goal rejected'
                continue
            result = self._block_on(handle.get_result_async(), timeout_sec=90.0)
            if result is not None and result.result.error_code.val == 1:
                return True, ''
            last = (f'error_code {result.result.error_code.val}'
                    if result is not None else 'timed out')
            self.get_logger().warn(
                f'plan{" to " + label if label else ""} attempt '
                f'{attempt + 1}/{PLAN_ATTEMPTS} failed ({last})')
        return False, last

    def move_cartesian(self, pose, avoid_collisions=True, label='',
                       arm=DEFAULT_ARM):
        """Straight line of the arm's flange to a pose in ITS base frame."""
        if not self._cartesian.wait_for_service(timeout_sec=10.0):
            return False, 'compute_cartesian_path service not available'
        self.wait_until_settled(arm)
        req = GetCartesianPath.Request()
        req.header.frame_id = arm.frame
        req.group_name = arm.group
        req.link_name = arm.eef_link
        req.max_step = CARTESIAN_STEP
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.waypoints = [pose]

        resp = self._block_on(self._cartesian.call_async(req), timeout_sec=30.0)
        if resp is None:
            return False, 'Cartesian plan timed out'
        if resp.fraction < MIN_CARTESIAN_FRACTION:
            # Worth reporting the number: a partial fraction with safety on
            # usually means the jog runs into something, which is the check
            # doing its job, and is a different problem from being unreachable.
            return False, (f'only {resp.fraction:.2f} of the path was '
                           f'reachable' + ('' if avoid_collisions else
                                           ' (collision checking is OFF)'))

        if not self._execute.wait_for_server(timeout_sec=10.0):
            return False, 'execute_trajectory server not available'
        g = ExecuteTrajectory.Goal()
        g.trajectory = resp.solution
        handle = self._block_on(self._execute.send_goal_async(g), timeout_sec=15.0)
        if handle is None or not handle.accepted:
            return False, f'execution{" of " + label if label else ""} rejected'
        result = self._block_on(handle.get_result_async(), timeout_sec=90.0)
        if result is None:
            return False, 'execution timed out'
        if result.result.error_code.val != 1:
            # Report the code. "execution failed" on its own sent this round
            # the houses once already.
            return False, (f'execution failed (error_code '
                           f'{result.result.error_code.val})')
        return True, ''

    def command_gripper(self, position, arm=DEFAULT_ARM):
        if not GRIPPER_OPEN_POS <= float(position) <= GRIPPER_UPPER_LIMIT:
            return False, (
                f'gripper position {float(position):.3f} rad is outside '
                f'{GRIPPER_OPEN_POS:.2f}..{GRIPPER_UPPER_LIMIT:.2f}. '
                f'Refusing rather than clamping -- and note the floor is '
                f'not 0: a knuckle parked on its lower limit stops '
                f'responding for the rest of the run.')
        client = self._grippers[arm.key]
        if not client.wait_for_server(timeout_sec=10.0):
            return False, f"{arm.label}'s gripper action server not available"
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        handle = self._block_on(
            client.send_goal_async(goal), timeout_sec=10.0)
        if handle is None or not handle.accepted:
            return False, 'gripper goal rejected'
        # Deliberately NOT waited on. A closing goal that has bottomed out on
        # an object can never satisfy goal_tolerance, so it stays active --
        # and staying active is what holds the squeeze. See the gripper notes
        # in pour_action_server and in controllers.yaml.
        return True, ''


HELP = """\
  arm [a|b]                which arm to drive, or say which is selected
  state                    where the arm is now (joints, tool pose, gripper)
  list                     taught points, both arms
  show NAME                one point in full
  save NAME [note...]      record where the selected arm is now
  resave NAME [note...]    same, overwriting an existing point
  rm NAME                  delete a point
  goto NAME                plan and move there, on the arm it was taught on

  jog j1..j6 DEG           one joint, degrees
  jog x|y|z MM             straight line along a base axis
  jog tx|ty|tz MM          straight line along a flange axis (tz = approach)
  jog rx|ry|rz DEG         rotate about a base axis, holding the
                           selected tool's tip still (see `tool`)

  open | close [POS]       gripper (POS in radians, default 0.5)
  wait [SECONDS]           pause, and record the pause (default 1)

  record NAME [note...]    start building a pipeline out of what you teach
  stop                     finish it
  run NAME [dry]           replay one; `dry` lists the steps without moving
  pipeline                 list them
  pipeline show|rm NAME    one in full, or delete it
  pipeline step KIND VAL   append by hand while recording (goto|grip|wait)
  pipeline drop [N]        remove the last step, or step N
  pipeline export [NAME]   print as a Python literal
  tool [NAME]              list tool centre points, or select one

  robot                    real robot: mode, safety, program, speed
  robot on | off           power on and release brakes / power off
  robot play|pause|stop    the External Control program on the robot
  robot unlock             clear a protective stop
  robot resend             restart the control script (headless mode)
  speed [PCT]              show the speed, or set the UR speed slider
  freedrive [on|off]       move the arm by hand; ROS lets go of it
  safety [on|off]          collision checking for Cartesian jogs
  export [NAME]            print as a pour_action_server source snippet
  file                     which point file is being edited
  help | quit

Everything except `list`, `show` and `goto` acts on the SELECTED arm. Jog and
pose axes are in that arm's own base frame, which for arm B is b_base_link --
the two arms do not share an origin.

The robot, speed and freedrive commands only work on the real robot
(workcell_real or workcell_twin); in the simulation they say so.

While recording, `save`, `goto`, `open` and `close` also append a step, and
`save` with no name auto-names it after the pipeline. Jogs never become
steps: they are how you reach a point, and a relative move does not replay.
"""


class Pendant:
    """The human half: parses commands and prints results."""

    def __init__(self, node, store, arm=DEFAULT_ARM):
        self.node = node
        self.store = store
        # Which arm every command below acts on. One selected arm rather than
        # an argument on each command: teaching is a sequence of jogs against
        # one arm, and an `arm` word on every line is both noise and a thing
        # to forget on the line that matters. `arm` with no argument says
        # which one is live, and the prompt carries it too.
        self.arm = arm
        self.safety = True
        # Which point on the end effector the pendant talks about. tool0 --
        # the flange -- is the default, and with it every transform is the
        # identity, so the pendant behaves exactly as it did before tool
        # frames existed. Selecting a bottle's spout makes ROTATION jogs turn
        # about that spout instead, holding its xyz still.
        self.tool = TOOL0
        # The pipeline currently being recorded, or None. RECORD MODE is the
        # whole feature: while this is set, teaching a point also appends a
        # step that drives to it, so a sequence gets built as a side effect
        # of the teaching you were doing anyway rather than as a second job
        # afterwards with the robot already moved on.
        self.recording = None

    # -- helpers ----------------------------------------------------------

    @staticmethod
    def _number(text, what):
        try:
            return float(text)
        except ValueError:
            raise ValueError(f'{what} must be a number, got {text!r}') from None

    def _pose_msg(self, xyz, quat):
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = (float(v) for v in quat)
        return p

    def _require_pose(self):
        pose = self.node.tool_pose(None, self.arm)
        if pose is None:
            raise ValueError(
                f'cannot read {self.arm.eef_link} through /compute_fk, so '
                f'there is no pose to jog FROM. Is move_group running? Joint '
                f'jogs still work.')
        return pose

    def _refuse_in_freedrive(self):
        """Refuse a motion command the robot cannot carry out right now.

        Freedrive: not only because the move would fail (the trajectory
        controller is off): a goal that did start would move an arm somebody
        is holding.

        Program not running: the driver switches the arm controller off
        whenever the control program on the robot stops, and MoveIt then
        reports only "error_code -4" (CONTROL_FAILED) after planning a path
        it cannot send anywhere. Seen on the real cell as a screen of those
        and no hint why. So it is refused before anything is planned, with
        the reason.
        """
        robot = self.node.robot
        if robot.freedrive:
            raise ValueError('freedrive is on: the arm is being moved by '
                             'hand. `freedrive off` first.')
        st = robot.status()
        if st['real'] and st['program_running'] is False:
            raise ValueError(
                'the control program is not running on the robot, so ROS '
                'cannot move the arm. Headless (headless_mode:=true): '
                '`robot resend`. Otherwise load the External Control '
                'program on the UR pendant and `robot play`.')

    def _report(self, ok, why, did):
        print(f'  {did}' if ok else f'  FAILED: {why}')

    def _record(self, kind, arg, arm=None, note=''):
        """Append a step to the pipeline being recorded, if there is one.

        Called from the ordinary commands, so recording is something they do
        as well as their job rather than a separate mode with its own verbs.
        No-op when not recording, which keeps the call sites free of `if`.

        Written to disk immediately, for the reason `save` is: the sequence
        somebody just drove the robot through is not reproducible by
        re-running anything.
        """
        if self.recording is None:
            return None
        step = self.recording.append(Step(kind, arg, note, arm))
        self.store.save()
        print(f'  + step {len(self.recording)}: {step.describe()}')
        return step

    def _auto_point_name(self):
        """Next free `<pipeline>_NN` name, for `save` with no name given.

        So that recording a sequence is jog, save, jog, save -- naming every
        waypoint is work that mostly produces names nobody reads, and the
        ones that matter can still be given explicitly.

        Skips names already taken rather than overwriting: re-recording a
        pipeline over an old one must not silently redefine the old one's
        points, which are what it still runs on.
        """
        for n in range(1, 1000):
            name = f'{self.recording.name}_{n:02d}'
            if name not in self.store:
                return name
        raise ValueError(
            f'cannot find a free name under {self.recording.name}_NN; '
            f'name this point explicitly')

    # -- commands ---------------------------------------------------------

    def cmd_arm(self, args):
        """Select which arm everything else acts on, or say which is live."""
        if not args:
            print(f'  {self.arm.label} ({self.arm.group}), planning in '
                  f'{self.arm.frame}')
            print('  others: ' + ', '.join(
                f'{a.key} = {a.label}' for a in ARMS.values()
                if a.key != self.arm.key))
            return
        key = args[0].lower().replace('arm', '').strip() or args[0].lower()
        if key not in ARMS:
            raise ValueError(
                f'no arm {args[0]!r}; known: ' + ', '.join(
                    f'{a.key} ({a.label})' for a in ARMS.values()))
        self.arm = ARMS[key]
        # The tool frames are arm A's bottle spouts, and a spout is a property
        # of what is being carried rather than of the arm -- but carrying a
        # selection across an arm change would silently apply one arm's tool
        # offset to the other's flange. Reset and say so.
        if self.tool is not TOOL0:
            print(f'  tool reset to {TOOL0.name} (it was {self.tool.name})')
            self.tool = TOOL0
        print(f'  now driving {self.arm.label} ({self.arm.group}), '
              f'poses in {self.arm.frame}')

    def cmd_state(self, args):
        joints = self.node.arm_joints(self.arm)
        for name, value in joints.items():
            print(f'  {name:<29} {value:+.4f} rad  ({math.degrees(value):+7.2f} deg)')
        pose = self.node.tool_pose(joints, self.arm)
        if pose is None:
            print(f'  {self.arm.eef_link}' + ' ' * 26
                  + '(no /compute_fk -- is move_group up?)')
        else:
            (x, y, z), q = pose
            print(f'  {self.arm.eef_link} in {self.arm.frame:<14}'
                  f'({x:+.4f} {y:+.4f} {z:+.4f})'
                  f'  quat ({q[0]:+.4f} {q[1]:+.4f} {q[2]:+.4f} {q[3]:+.4f})')
        if pose is not None and self.tool is not TOOL0:
            tip, _ = tcp_from_tool0(pose[0], pose[1], self.tool)
            print(f'  {self.tool.name:<29} ({tip[0]:+.4f} {tip[1]:+.4f} '
                  f'{tip[2]:+.4f})   <- tool tip, {self.tool.reach*1000:.0f}mm '
                  f'off the flange')
        grip = self.node.gripper_position(self.arm)
        print(f'  {self.arm.gripper_joint:<29} '
              + ('(not published)' if grip is None else f'{grip:+.4f} rad'))

    def cmd_list(self, args):
        """Every point in the file, both arms, grouped by which arm it is on.

        Deliberately NOT filtered to the selected arm. The question this
        answers is "what does the robot know", and hiding half of it behind a
        mode is how you end up teaching a second point that already exists.
        """
        if not len(self.store):
            print(f'  no points yet in {self.store.path}')
            return
        for arm in ARMS.values():
            names = self.store.names(arm.group)
            if not names:
                continue
            here = ' <- selected' if arm.key == self.arm.key else ''
            print(f'  {arm.label} ({arm.group}){here}')
            for name in names:
                print('    ' + self.store.get(name).describe(
                    list(arm.joints), self.store.group))
        stray = [n for n in self.store.names()
                 if self.store.group_of(self.store.get(n))
                 not in {a.group for a in ARMS.values()}]
        if stray:
            # Not an error: the file is hand-editable and may outlive this
            # robot's group names. Better listed under a heading that says so
            # than quietly absent.
            print('  not on any arm this pendant knows')
            for name in stray:
                print('    ' + self.store.get(name).describe(
                    None, self.store.group))

    def cmd_show(self, args):
        if not args:
            raise ValueError('show needs a point name')
        point = self.store.get(args[0])
        group = self.store.group_of(point)
        arm = next((a for a in ARMS.values() if a.group == group), None)
        print('  ' + point.describe(list(arm.joints) if arm else None,
                                    self.store.group))

    def cmd_save(self, args, overwrite=False):
        if not args and self.recording is None:
            raise ValueError('save needs a name')
        if args:
            name, note = args[0], ' '.join(args[1:])
        else:
            name, note = self._auto_point_name(), ''
        if name in self.store and not overwrite:
            raise ValueError(
                f"'{name}' already exists. Use `resave {name}` to replace it, "
                f'or `show {name}` to see what is there.')

        joints = self.node.arm_joints(self.arm)
        pose = self.node.tool_pose(joints, self.arm)
        point = Point(
            name, joints,
            pose=None if pose is None else {
                'xyz': [round(v, 6) for v in pose[0]],
                'quat_xyzw': [round(v, 6) for v in pose[1]]},
            gripper=self.node.gripper_position(self.arm), note=note,
            tool=self.tool.name,
            # Stamped with the arm it was taught on, always -- including for
            # arm A, whose group is also the file default. Relying on the
            # default would make a point's meaning depend on a header line
            # somebody could change, and the joint names alone would still say
            # which arm it is while nothing enforced that they agreed.
            group=self.arm.group)

        point, changed = point.wrapped()
        for jname, before, after in changed:
            # Never silent: this is the branch problem pour_action_server has
            # a note about, and the person teaching the point should know the
            # number they are about to paste is not the one the arm reported.
            print(f'  wrapped {jname}: {before:+.4f} -> {after:+.4f} '
                  f'(same pose, shorter wrist path)')

        self.store.add(point, overwrite=True)
        # Saved to disk NOW rather than on quit. A teaching session is the one
        # thing here that cannot be reproduced by re-running something.
        self.store.save()
        print(f'  saved {name} -> {self.store.path}')
        # Recorded AFTER the point exists, so a pipeline can never contain a
        # goto to a point that was never written.
        self._record('goto', name)

    def cmd_resave(self, args):
        self.cmd_save(args, overwrite=True)

    def cmd_rm(self, args):
        if not args:
            raise ValueError('rm needs a point name')
        self.store.remove(args[0])
        self.store.save()
        print(f'  removed {args[0]}')

    def cmd_goto(self, args):
        """Drive to a point, on whichever arm it was taught on.

        NOT on the selected arm. A point's joint names say which arm it
        belongs to and there is exactly one right answer; refusing, or worse
        driving arm A to arm B's configuration, would both be worse than just
        going. Says which arm it used when that is not the selected one.
        """
        if not args:
            raise ValueError('goto needs a point name')
        self._refuse_in_freedrive()
        point = self.store.get(args[0])
        arm = self._arm_for(point)
        target = dict(zip(arm.joints, point.joints_in_order(list(arm.joints))))
        on = '' if arm.key == self.arm.key else f' on {arm.label}'
        print(f'  moving to {point.name}{on} ...')
        ok, why = self.node.move_to_joints(target, point.name, arm)
        self._report(ok, why, f'at {point.name}')
        # Only a move that arrived becomes a step. Recording a failed one
        # would write a pipeline whose first run is already known not to
        # work, and the operator has just been told it failed.
        if ok:
            self._record('goto', point.name)

    def _arm_for(self, point):
        """Return the arm a stored point belongs to, by group then joints."""
        group = self.store.group_of(point)
        for arm in ARMS.values():
            if arm.group == group:
                return arm
        # No group match: fall back to the joint names, which cannot lie about
        # which arm they are. A hand-edited file with a stale group header
        # lands here rather than driving the wrong arm.
        for arm in ARMS.values():
            if all(j in point.joints for j in arm.joints):
                return arm
        raise PointStoreError(
            f"point '{point.name}' is in group {group!r} and its joints "
            f'({", ".join(sorted(point.joints))}) do not match any arm this '
            f'pendant knows')

    def cmd_jog(self, args):
        if len(args) != 2:
            raise ValueError('jog needs an axis and an amount, e.g. `jog z 20` '
                             'or `jog j1 -15`')
        axis, amount = args[0].lower(), self._number(args[1], 'jog amount')
        self._refuse_in_freedrive()

        if axis.startswith('j') and axis[1:].isdigit():
            index = int(axis[1:]) - 1
            if not 0 <= index < len(self.arm.joints):
                raise ValueError(
                    f'joint {axis} is not in 1..{len(self.arm.joints)}')
            if abs(amount) > MAX_JOG_DEG:
                raise ValueError(
                    f'{amount:g} deg exceeds MAX_JOG_DEG ({MAX_JOG_DEG:g}). '
                    f'Refusing rather than clamping -- if that is really what '
                    f'you meant, jog there in steps.')
            joints = self.node.arm_joints(self.arm)
            name = self.arm.joints[index]
            joints[name] += math.radians(amount)
            print(f'  {name} {amount:+g} deg ...')
            ok, why = self.node.move_to_joints(joints, name, self.arm)
            self._report(ok, why, f'{name} now {joints[name]:+.4f} rad')
            return

        if axis in AXES or (len(axis) == 2 and axis[0] == 't' and axis[1] in AXES):
            if abs(amount) > MAX_JOG_MM:
                raise ValueError(
                    f'{amount:g} mm exceeds MAX_JOG_MM ({MAX_JOG_MM:g}). '
                    f'Refusing rather than clamping -- did you mean '
                    f'{amount / 10:g}?')
            (x, y, z), quat = self._require_pose()
            unit = AXES[axis[-1]]
            step = amount / 1000.0
            if axis[0] == 't':
                # Tool frame: rotate the axis into base_link by the current
                # tool orientation. tz is the approach direction, which is the
                # one you want for running in at a bottle.
                unit = quat_rotate(quat, unit)
            target = self._pose_msg(
                (x + unit[0] * step, y + unit[1] * step, z + unit[2] * step), quat)
            self._jog_cartesian(target, f'{axis} {amount:+g}mm')
            return

        if len(axis) == 2 and axis[0] == 'r' and axis[1] in AXES:
            if abs(amount) > MAX_JOG_DEG:
                raise ValueError(
                    f'{amount:g} deg exceeds MAX_JOG_DEG ({MAX_JOG_DEG:g}). '
                    f'Refusing rather than clamping.')
            p0, q0 = self._require_pose()
            # About the BASE axis (pre-multiplied), which is what somebody
            # watching the robot from across the room means by "tilt it back";
            # post-multiplying would turn it about the tool's own axes.
            #
            # And about the TOOL TIP, not the flange: with a bottle selected
            # this holds the pour spout still and swings the bottle around it.
            # With tool0 it reduces to the old behaviour exactly.
            p1, q1 = rotate_about_tcp(
                p0, q0, self.tool,
                quat_about(AXES[axis[1]], math.radians(amount)))
            self._jog_cartesian(self._pose_msg(p1, q1),
                                f'{axis} {amount:+g}deg about {self.tool.name}')
            return

        raise ValueError(
            f'unknown jog axis {args[0]!r}. Expected j1..j6, x/y/z, '
            f'tx/ty/tz or rx/ry/rz.')

    def _jog_cartesian(self, pose, label):
        if not self.safety:
            print('  *** safety OFF -- this jog is not collision checked ***')
        print(f'  jog {label} ...')
        ok, why = self.node.move_cartesian(pose, self.safety, label, self.arm)
        self._report(ok, why, f'jogged {label}')

    def cmd_open(self, args):
        ok, why = self.node.command_gripper(GRIPPER_OPEN_POS, self.arm)
        self._report(ok, why, f"{self.arm.label}'s gripper opening")
        if ok:
            self._record('grip', GRIPPER_OPEN_POS, self.arm.key)

    def cmd_close(self, args):
        pos = self._number(args[0], 'gripper position') if args else 0.5
        ok, why = self.node.command_gripper(pos, self.arm)
        self._report(ok, why,
                     f"{self.arm.label}'s gripper closing to {pos:.3f}")
        if ok:
            self._record('grip', pos, self.arm.key)

    def cmd_wait(self, args):
        """Pause, and record the pause when recording.

        Interactively this is nearly useless on its own; it exists so that a
        settle a person puts into a sequence by pausing is a settle the
        replay also performs. The alternative is a pipeline that runs the
        steps back to back and fails on the one grasp that needed a moment.
        """
        seconds = self._number(args[0], 'wait') if args else 1.0
        step = Step('wait', seconds)     # validates the bound before sleeping
        print(f'  waiting {step.arg:g}s ...')
        time.sleep(step.arg)
        self._record('wait', step.arg)

    def cmd_tool(self, args):
        """Select the point on the end effector that rotations turn about."""
        if not args:
            for name in sorted(TOOLS):
                tool = TOOLS[name]
                mark = '*' if tool.name == self.tool.name else ' '
                print(f'  {mark} {name:<16} {tool.reach * 1000:6.1f}mm off '
                      f'the flange   {tool.note}')
            return
        # get_tool raises KeyError listing the alternatives; dispatch() only
        # catches ValueError and PointStoreError, so translate it.
        try:
            self.tool = get_tool(args[0])
        except KeyError as exc:
            raise ValueError(str(exc).strip('"')) from None
        print(f'  tool is now {self.tool.name}'
              + ('' if self.tool is TOOL0 else
                 '; rotation jogs will hold its tip still'))

    def cmd_safety(self, args):
        if args:
            word = args[0].lower()
            if word not in ('on', 'off'):
                raise ValueError('safety takes on or off')
            self.safety = word == 'on'
        print(f'  Cartesian jog collision checking: '
              f'{"ON" if self.safety else "OFF"}')

    def cmd_export(self, args):
        names = args or self.store.names()
        if not names:
            print('  nothing to export')
            return
        for name in names:
            point = self.store.get(name)
            arm = self._arm_for(point)
            values = ', '.join(f'{v:.4f}'
                               for v in point.joints_in_order(list(arm.joints)))
            note = f' -- {point.note}' if point.note else ''
            print(f'  # {name} ({arm.label}){note}')
            print(f'  approach_joints=[{values}],')

    # -- pipelines --------------------------------------------------------

    def _pipeline(self, name):
        try:
            return self.store.pipelines[name]
        except KeyError:
            known = ', '.join(sorted(self.store.pipelines)) or '(none)'
            raise ValueError(
                f"no pipeline named '{name}' in {self.store.path}. "
                f'Known: {known}') from None

    def _step_arm(self, step):
        """Name the arm a gripper step drives.

        Falls back to the selected arm for a hand-written step that does not
        say which, and says so out loud. Refusing would make an obvious
        one-line pipeline unusable for a missing field the reader can see is
        missing; guessing in silence is how the wrong gripper opens while the
        other one is holding a bottle.
        """
        if step.arm is None:
            print(f'     (step does not say which arm; using '
                  f'{self.arm.label})')
            return self.arm
        arm = ARMS.get(step.arm)
        if arm is None:
            raise ValueError(
                f'step names arm {step.arm!r}; known: ' + ', '.join(ARMS))
        return arm

    def cmd_record(self, args):
        """Start recording a pipeline, or report the one in progress."""
        if not args:
            if self.recording is None:
                print('  not recording. `record NAME` starts a pipeline.')
                return
            print(f'  recording {self.recording.name} '
                  f'({len(self.recording)} step(s)); `stop` ends it')
            for index, step in enumerate(self.recording.steps, 1):
                print(f'    {index:>2}. {step.describe()}')
            return
        if args[0].lower() == 'off':
            self.cmd_stop([])
            return
        if self.recording is not None:
            raise ValueError(
                f'already recording {self.recording.name}; `stop` finishes '
                f'it. Two at once would put every step into both.')
        name, note = args[0], ' '.join(args[1:])
        if name in self.store.pipelines:
            raise ValueError(
                f"'{name}' already exists. `pipeline rm {name}` first, or "
                f'record under another name.')
        self.recording = Pipeline(name, note=note)
        # In the file from the start, not on `stop`. Same reasoning as
        # saving a point immediately: a session at the robot is the one
        # thing here that cannot be reproduced by re-running something, and
        # that includes the order the points were taught in.
        self.store.pipelines[name] = self.recording
        self.store.save()
        print(f'  recording {name}.')
        print('  save, goto and the gripper commands now also append a step. '
              'Jogs do not:')
        print('  they are how you reach a point, and a relative move cannot '
              'be replayed.')
        print('  `save` with no name auto-names. `stop` when the sequence is '
              'complete.')

    def cmd_stop(self, args):
        """Finish the pipeline being recorded."""
        if self.recording is None:
            raise ValueError('not recording; `record NAME` starts a pipeline')
        done, self.recording = self.recording, None
        if not len(done):
            # An empty pipeline is clutter rather than data -- nothing was
            # taught into it -- and leaving it behind means the next `record`
            # of that name is refused by something holding no steps.
            self.store.pipelines.pop(done.name, None)
            self.store.save()
            print(f'  {done.name} recorded no steps; discarded')
            return
        self.store.save()
        print(f'  stopped. {done.name}: {len(done)} step(s) '
              f'-> {self.store.path}')
        missing = done.missing_points(self.store)
        if missing:
            print('  WARNING: it names points that no longer exist: '
                  + ', '.join(missing))

    def cmd_run(self, args):
        """Replay a pipeline, stopping at the first step that fails."""
        if not args:
            raise ValueError('run needs a pipeline name')
        dry = len(args) > 1 and args[1].lower() in ('dry', 'plan')
        if self.recording is not None:
            raise ValueError(
                f'stop recording {self.recording.name} first -- running now '
                f'would append the replay to it, step by step.')
        pipeline = self._pipeline(args[0])
        if not len(pipeline):
            print(f'  {pipeline.name} has no steps')
            return
        if not dry:
            self._refuse_in_freedrive()
        missing = pipeline.missing_points(self.store)
        if missing:
            # Checked before anything moves. Finding out at step 9 of 11
            # leaves the arm mid-sequence holding something.
            raise ValueError(
                f"{pipeline.name} names points that do not exist: "
                f"{', '.join(missing)}")
        print(f'  {"planning" if dry else "running"} {pipeline.name}: '
              f'{len(pipeline)} step(s)')
        for index, step in enumerate(pipeline.steps, 1):
            head = f'  {index:>2}/{len(pipeline)}  {step.describe()}'
            if dry:
                print(head + self._dry_detail(step))
                continue
            print(head + ' ...')
            ok, why = self._execute(step)
            if not ok:
                print(f'  STOPPED at step {index} of {len(pipeline)}: {why}')
                return
        print(f'  {"planned" if dry else "finished"} {pipeline.name}')

    def _dry_detail(self, step):
        """Say which arm a step would drive, for a dry run's step line."""
        if step.kind == 'goto':
            try:
                return f'   -> {self._arm_for(self.store.get(step.arg)).label}'
            except (ValueError, PointStoreError):
                return '   -> unknown arm'
        # Nothing for grip or wait: a grip step already carries its arm in
        # describe(), and repeating it just makes the column noisy.
        return ''

    def _execute(self, step):
        """Run one step. Returns (ok, why) like every other motion call."""
        if step.kind == 'wait':
            time.sleep(step.arg)
            return True, ''
        if step.kind == 'goto':
            point = self.store.get(step.arg)
            arm = self._arm_for(point)
            target = dict(zip(arm.joints,
                              point.joints_in_order(list(arm.joints))))
            return self.node.move_to_joints(target, point.name, arm)
        return self.node.command_gripper(step.arg, self._step_arm(step))

    def _recording_or_refuse(self, doing):
        """Return the pipeline being recorded, or explain why there is none."""
        if self.recording is None:
            raise ValueError(
                f'not recording, so there is nothing to {doing}. '
                f'`record NAME` starts a pipeline; to change a finished one, '
                f'edit the YAML.')
        return self.recording

    def _list_pipelines(self, args):
        if not self.store.pipelines:
            print(f'  no pipelines yet in {self.store.path}')
            return
        for name in sorted(self.store.pipelines):
            live = self.recording is not None and self.recording.name == name
            print(('  * ' if live else '    ')
                  + self.store.pipelines[name].describe()
                  + ('   <- recording' if live else ''))

    def _pipeline_show(self, args):
        if not args:
            raise ValueError('pipeline show needs a name')
        pipeline = self._pipeline(args[0])
        print('  ' + pipeline.describe())
        for index, step in enumerate(pipeline.steps, 1):
            print(f'    {index:>2}. {step.describe()}' + self._dry_detail(step))
        missing = pipeline.missing_points(self.store)
        if missing:
            print('  missing points: ' + ', '.join(missing))

    def _pipeline_rm(self, args):
        if not args:
            raise ValueError('pipeline rm needs a name')
        name = args[0]
        self._pipeline(name)
        if self.recording is not None and self.recording.name == name:
            raise ValueError(f'{name} is being recorded; `stop` first')
        del self.store.pipelines[name]
        self.store.save()
        print(f'  removed pipeline {name}')

    def _pipeline_step(self, args):
        """Append a step by hand, for the ones no command produces."""
        self._recording_or_refuse('add a step to')
        if len(args) < 2:
            raise ValueError('pipeline step needs a kind and a value, e.g. '
                             '`pipeline step wait 0.5`')
        kind, raw, note = args[0].lower(), args[1], ' '.join(args[2:])
        if kind == 'goto':
            self.store.get(raw)          # refuse a step to a point that is
            self._record(kind, raw, None, note)   # not there
            return
        # Only grip needs an arm, and it takes the selected one -- the same
        # arm `close` would have driven had it been typed instead.
        self._record(kind, self._number(raw, kind),
                     self.arm.key if kind == 'grip' else None, note)

    def _pipeline_drop(self, args):
        recording = self._recording_or_refuse('drop a step from')
        if not len(recording):
            raise ValueError(f'{recording.name} has no steps yet')
        index = (int(self._number(args[0], 'step number')) if args
                 else len(recording))
        if not 1 <= index <= len(recording):
            raise ValueError(f'no step {index}; {recording.name} has '
                             f'{len(recording)}')
        gone = recording.steps.pop(index - 1)
        self.store.save()
        print(f'  dropped step {index}: {gone.describe()}')

    def _pipeline_export(self, args):
        names = args or sorted(self.store.pipelines)
        if not names:
            print('  nothing to export')
            return
        for name in names:
            self._export_pipeline(self._pipeline(name))

    # Same shape as COMMANDS below, and for the same reason: one table that
    # says what exists, so the error for an unknown word can list the real
    # ones rather than repeating a hand-written list that drifts.
    PIPELINE_SUBCOMMANDS = {
        'show': _pipeline_show, 'rm': _pipeline_rm, 'del': _pipeline_rm,
        'step': _pipeline_step, 'drop': _pipeline_drop,
        'export': _pipeline_export,
    }

    def cmd_pipeline(self, args):
        """List pipelines, or show, remove, extend, trim or export one."""
        if not args:
            self._list_pipelines(args)
            return
        verb, rest = args[0].lower(), args[1:]
        handler = self.PIPELINE_SUBCOMMANDS.get(verb)
        if handler is None:
            raise ValueError(
                f'no pipeline subcommand {verb!r}; try '
                + ', '.join(sorted(self.PIPELINE_SUBCOMMANDS))
                + ', or `pipeline` alone to list')
        handler(self, rest)

    def _export_pipeline(self, pipeline):
        """Print a pipeline as a Python literal a script can run.

        The same shape the steps have in the file, flattened into tuples,
        because the thing somebody wants this for is turning a taught
        sequence into an action server -- which is where all of these
        sequences lived before there was anywhere else to put them.
        """
        note = f'  -- {pipeline.note}' if pipeline.note else ''
        print(f'  # {pipeline.name}{note}')
        print(f'  {pipeline.name.upper()} = [')
        for step in pipeline.steps:
            if step.kind == 'goto':
                body = f"('goto', {step.arg!r}),"
            elif step.kind == 'grip':
                body = f"('grip', {step.arg:.4f}, {step.arm!r}),"
            else:
                body = f"('wait', {step.arg:g}),"
            print(f'      {body}'
                  + (f'  # {step.note}' if step.note else ''))
        print('  ]')

    # -- the real robot ---------------------------------------------------

    def cmd_robot(self, args):
        """Show the real robot's state, or power it / run its program."""
        robot = self.node.robot
        if not args:
            st = robot.status()
            if not st['real']:
                print('  no robot status is being published. This is the '
                      'simulation, or the')
                print('  driver is not connected (workcell_real / '
                      'workcell_twin, then Play on the UR pendant).')
                return
            prog = {True: 'running', False: 'NOT running'}.get(
                st['program_running'], 'unknown')
            speed = ('unknown' if st['speed_scaling'] is None
                     else f"{st['speed_scaling']:.0f}%")
            print(f"  robot mode     {st['robot_mode'] or 'unknown'}")
            print(f"  safety         {st['safety_mode'] or 'unknown'}")
            print(f'  program        {prog}'
                  + ('' if st['program_running'] else
                     '   (ROS cannot move the arm until it is)'))
            print(f'  speed          {speed}')
            print(f"  freedrive      {'ON' if st['freedrive'] else 'off'}")
            return
        verb = args[0].lower()
        if verb in ('on', 'off'):
            if verb == 'off' and robot.freedrive:
                raise ValueError('freedrive is on; `freedrive off` first')
            print('  powering on and releasing the brakes (can take 30s) ...'
                  if verb == 'on' else '  powering off ...')
            ok, why = robot.set_mode(
                MODE_RUNNING if verb == 'on' else MODE_POWER_OFF)
            self._report(ok, why, 'robot is ' + (
                'on, brakes released. `robot play` starts the program.'
                if verb == 'on' else 'powered off'))
            return
        if verb == 'resend':
            ok, why = robot.resend()
            self._report(ok, why, why)
            return
        if verb in DASHBOARD_VERBS:
            ok, why = robot.dashboard(verb)
            self._report(ok, why, why)
            return
        raise ValueError(
            f'no robot command {args[0]!r}; try on, off, resend, '
            + ', '.join(sorted(DASHBOARD_VERBS)) + ', or `robot` alone')

    def cmd_speed(self, args):
        """Show the speed, or set the UR speed slider in percent."""
        robot = self.node.robot
        if not args:
            st = robot.status()
            print('  speed: ' + ('unknown (real robot only)'
                                 if st['speed_scaling'] is None
                                 else f"{st['speed_scaling']:.0f}%"))
            return
        pct = self._number(args[0].rstrip('%'), 'speed')
        # Refused, not clamped, like the jogs: `speed 500` is a typo.
        if not 0 < pct <= 100:
            raise ValueError(f'speed must be above 0 and at most 100 (percent), '
                             f'got {pct:g}')
        ok, why = robot.set_speed(pct / 100.0)
        self._report(ok, why, f'speed slider at {pct:g}%')

    def cmd_freedrive(self, args):
        """Let go of the arm so it can be moved by hand, or take it back."""
        robot = self.node.robot
        if not args:
            print(f"  freedrive is {'ON' if robot.freedrive else 'off'}")
            return
        word = args[0].lower()
        if word not in ('on', 'off'):
            raise ValueError('freedrive takes on or off')
        ok, why = robot.set_freedrive(word == 'on')
        self._report(ok, why, (
            'freedrive ON: hold the arm and move it; `save` still works. '
            '`freedrive off` when done.' if word == 'on' else
            'freedrive off: ROS has the arm again'))

    def release(self):
        """Leave the robot as ROS expects it; called when the pendant exits.

        A pendant that quits with freedrive on would leave the trajectory
        controller off, and every other client would find the arm dead.
        """
        if self.node.robot.freedrive:
            ok, why = self.node.robot.set_freedrive(False)
            print('  freedrive off' if ok else f'  could not end freedrive: {why}')

    def cmd_file(self, args):
        extra = (f', {len(self.store.pipelines)} pipeline(s)'
                 if self.store.pipelines else '')
        print(f'  {self.store.path}  ({len(self.store)} point(s){extra})')

    def cmd_help(self, args):
        print(HELP, end='')

    COMMANDS = {
        'state': cmd_state, 'list': cmd_list, 'ls': cmd_list, 'show': cmd_show,
        'save': cmd_save, 'resave': cmd_resave, 'rm': cmd_rm, 'del': cmd_rm,
        'goto': cmd_goto, 'jog': cmd_jog, 'open': cmd_open, 'close': cmd_close,
        'safety': cmd_safety, 'export': cmd_export, 'file': cmd_file,
        'tool': cmd_tool, 'arm': cmd_arm,
        'record': cmd_record, 'stop': cmd_stop, 'run': cmd_run,
        'wait': cmd_wait,
        'robot': cmd_robot, 'speed': cmd_speed, 'freedrive': cmd_freedrive,
        'pipeline': cmd_pipeline, 'pl': cmd_pipeline,
        'help': cmd_help, '?': cmd_help,
    }

    def dispatch(self, line):
        """Run one command line. Returns False to quit."""
        try:
            words = shlex.split(line)
        except ValueError as exc:
            print(f'  cannot parse that line: {exc}')
            return True
        if not words:
            return True
        verb, args = words[0].lower(), words[1:]
        if verb in ('quit', 'exit', 'q'):
            return False

        handler = self.COMMANDS.get(verb)
        if handler is None:
            print(f'  unknown command {verb!r}. `help` lists them.')
            return True
        try:
            handler(self, args)
        except (ValueError, PointStoreError, PipelineError) as exc:
            # Expected, explained failures: a bad name, an out-of-range jog, a
            # point that does not exist. Print and carry on -- losing the
            # session over a typo would mean re-teaching everything.
            print(f'  {exc}')
        except Exception as exc:                            # noqa: BLE001
            print(f'  unexpected error: {type(exc).__name__}: {exc}')
        return True

    def run(self):
        print(HELP, end='')
        self.cmd_file([])
        self.cmd_arm([])
        while True:
            try:
                # The selected arm is in the prompt, not just behind `arm`.
                # Every jog and every save goes to whichever one this says,
                # and a pendant that makes you remember which is a pendant
                # that will eventually drive the wrong arm into the counter.
                # The pipeline being recorded goes in the prompt for the
                # same reason the arm does: every save and every gripper
                # command is going into it, and a mode you cannot see is a
                # mode you forget you are in.
                rec = ('' if self.recording is None
                       else f' rec:{self.recording.name}')
                line = input(f'teach[{self.arm.key}]{rec}> ')
            except EOFError:
                print()
                return
            except KeyboardInterrupt:
                print('\n  (use `quit` to leave)')
                continue
            if not self.dispatch(line):
                return


def main(args=None):
    path = default_points_path()
    argv = sys.argv[1:]
    if '--file' in argv:
        path = points_path_for(argv[argv.index('--file') + 1])

    try:
        store = PointStore.load(path)
    except PointStoreError as exc:
        # Refuse to start rather than start empty. Starting empty on a file we
        # failed to parse would mean the first `save` rewrites it wholesale and
        # destroys whatever was in there.
        print(f'cannot read the point file:\n  {exc}', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = TeachNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    try:
        if not node.wait_for_state(timeout=10.0):
            print('no /joint_states after 10s -- is the robot up?',
                  file=sys.stderr)
            return 1
        pendant = Pendant(node, store)
        try:
            pendant.run()
        finally:
            pendant.release()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
