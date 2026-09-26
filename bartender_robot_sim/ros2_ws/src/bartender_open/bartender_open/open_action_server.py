"""OpenBottle action server: two arms, one capped beer.

THE SEQUENCE
------------
    arm B  picks the opener off its post
    arm A  picks the beer out of its stand and holds it 50mm clear
    arm B  brings the bell down over the cap until the crown plate is flat
           on it, then keeps going 6mm
                        <- the push. Arm A is what stops the bottle moving.
    here   measures whether that actually happened, and only then releases
           the cap
    arm B  lifts the opener away and the freed cap drops out of the bell
    here   measures where the cap ended up
    both   put what they are holding back, if asked to

WHAT IS MEASURED, AND WHY IT IS MEASURED RATHER THAN ASSUMED
------------------------------------------------------------
The cap comes off because this node publishes on a topic. That is not
negotiable at this fidelity and the reasoning is in
bartender_gazebo/scripts/make_beer_and_opener.py -- prying a 0.25mm crown
skirt is not something DART is going to do for us.

What that makes important is the gate in front of the publish. Three things
are read out of the simulator's own pose stream immediately before it:

  the bell is over the cap      -- within SEAT_OFFSET_MAX laterally, and has
                                   descended at least SEAT_DEPTH_MIN of its
                                   16mm depth
  the bottle has not moved      -- less than BOTTLE_SHIFT_MAX since before
                                   the press began
  arm A is still gripping       -- its knuckle is still stalled short of the
                                   clamp command

Take arm A away and the second fails: the push drives the bottle down into
its stand or knocks it off, it moves, and the cap stays on. That is the
sense in which the two arms are actually cooperating and not just both
running. After the opener is lifted clear, where the cap ended up is
measured too, and THAT is what the action reports success on -- not the fact
that the sequence ran to the end.

WHERE THE POSES COME FROM
-------------------------
/world/bar_world/dynamic_pose/info, which is SceneBroadcaster's report of
every non-static model in the world, bridged to a TFMessage by
bartender_gazebo/launch/sim.launch.py. Ground truth, straight from the
physics engine. There is no perception here and no pretence of it: this is a
simulator and reading the simulator is the honest way to know whether a
thing worked.

ONE-SHOT
--------
Ignition's DetachableJoint cannot re-attach in Fortress. Once the cap is off
it stays off until the world is reloaded, so a second goal on the same beer
is refused up front with a message saying so rather than being run and then
judged by a cap displacement that was already large before it started.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

from bartender_pour_interfaces.action import OpenBottle
from bartender_teach.point_store import (
    PointStore, PointStoreError, default_points_path,
)

from . import layout as L
from .arm import GRIPPER_OPEN_POS, Arm, block_on, pose_at, side_quat

# The topic the DetachableJoint inside models/beer_bottle/model.sdf listens
# on, bridged ROS->Gazebo in sim.launch.py. Named in three places and all
# three have to agree; bartender_gazebo's test_beer_and_opener.py checks the
# model's plugin tag and the bridge against the generator's constant.
# ---------------------------------------------------------------------------
# Taught points.
#
# Both arms' rest poses are literals in layout.py AND may be taught with
# `ros2 run bartender_teach teach`; if the point file names one, it wins.
# Same arrangement, and the same reasoning, as bartender_pour's: the literal
# is what runs on a bare workspace or a broken point file, and the shipped
# point file is seeded with these exact numbers so either path gives the same
# motion until somebody deliberately reteaches one.
#
# arm B's matters more than arm A's. Arm A's home is shared with the pour and
# the SRDF and is unlikely to move; arm B exists only for this action, stands
# across the counter facing back at it, and its home was picked by hand to
# keep it clear of the worktop and of arm A. That is exactly the kind of pose
# somebody will want to nudge after watching it work, and nudging it should
# not mean editing Python.
POINT_SOURCES = {}

_ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def _taught(name, prefix, fallback):
    """Return joints for `name` from the taught-point file, else `fallback`."""
    names = [prefix + j for j in _ARM_JOINTS]
    try:
        point = _POINTS.get(name)
    except PointStoreError:
        POINT_SOURCES[name] = 'built-in (not in the point file)'
        return fallback
    try:
        joints = point.joints_in_order(names)
    except PointStoreError as exc:
        # An incomplete point is a broken point -- and here it could also be a
        # point taught on the OTHER arm, whose joints are all named
        # differently. Either way, say so and keep going on the literal rather
        # than driving a partly-specified pose.
        POINT_SOURCES[name] = f'built-in (taught point unusable: {exc})'
        return fallback
    POINT_SOURCES[name] = f'taught, from {_POINTS.path}'
    return joints


try:
    _POINTS = PointStore.load(default_points_path())
except PointStoreError as _exc:
    _POINTS = PointStore(default_points_path())
    POINT_SOURCES['*'] = f'point file unreadable, using built-ins: {_exc}'

ARM_A_HOME = _taught('home', '', L.ARM_A_HOME)
ARM_B_HOME = _taught('b_home', 'b_', L.ARM_B_HOME)

DETACH_TOPIC = '/beer/cap/detach'
POSE_TOPIC = '/world/bar_world/dynamic_pose/info'

# Gazebo model names, as they appear on POSE_TOPIC.
# How thick the bar top is modelled as in the planning scene: the full
# height of the counter, hanging below its top face. Same box
# bartender_pour's COUNTER_BOX describes, and the same 0.9 the model.sdf
# gives it, so the two agree by construction rather than by luck.
COUNTER_THICKNESS = 0.90

BEER_MODEL = 'beer_bottle'
CAP_MODEL = 'beer_cap'
OPENER_MODEL = 'bottle_opener'

# Turns a cylinder primitive's own axis (its local z) into tool0's +y, which
# is "up" in the side grasp. -90 degrees about x, as (x, y, z, w).
UPRIGHT_IN_TOOL0 = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

# The opener rim starts this far above its seated height, so the last part of
# the descent is a straight line the bell wall can guide.
#
# 60mm rather than the 25mm the bell actually needs, and the extra is for the
# PLANNER, not the mechanism. The pose arm B is planned to is the last one
# before collision checking is switched off, and by then both arms are
# carrying attached objects whose envelopes are coaxial cylinders -- the
# opener directly above the bottle. Anything close enough for the bell to be
# near the cap is, to a cylinder envelope, already a collision. 60mm leaves
# 44mm between the two envelopes, which is enough that the plan is about the
# arms and not about the last centimetre of the approach.
PRE_PRESS_GAP = 0.060
# ...and is lifted this far after the detach, which has to clear the cap plus
# the bell depth with room to spare, and drawn back this far, so the bottle
# has somewhere to tip into.
RETREAT_LIFT = 0.080
RETREAT_BACK = 0.120
# Time for a released cap to fall and stop rattling before it is measured.
CAP_SETTLE_S = 2.0

# How many times to come down on the cap before giving up. A miss is usually
# the last few millimetres: the bell has 6.5mm of capture and the arm lands
# within about 3mm, so the descents that fail do so because the cap drifted
# between being measured and being arrived at, or because the arm went a
# little wide once. Backing off 60mm and coming down on a fresh measurement
# fixes that; doing it a third time would be hoping rather than retrying.
SEAT_ATTEMPTS = 2

# How long the held beer keeps moving after arm A's lift reports done, and
# what "stopped" counts as while polling for it.
#
# A bottle held by friction on its neck and lifted 50mm keeps swinging on
# the pads for real seconds after the arm's own trajectory finishes --
# measured on a traced run, 63mm of drift in x over about 6 real seconds
# following the lift, before it settled to within a couple of millimetres.
# "Re-aim right before every descent" (below) assumed a quick re-read would
# catch a settled bottle; it does not, if the swing is still going when the
# read happens, and the descent it feeds then takes long enough that the
# cap has moved again by the time the bell arrives. That is the leading
# suspect for the run that put a press 114.8mm off centre while the grasps
# on either side of it were clean -- see ROADMAP.md, "the open is
# unreliable".
#
# A fixed sleep was considered and rejected: the swing's size depends on how
# the lift disturbed the grip, which varies, and a guessed duration is
# either wasted time on a quiet lift or too short on a bad one -- exactly
# the failure this exists to close. Polled instead, the same way
# Arm.wait_for_gripper waits out a joint settling: quiet for a whole window,
# not just quiet at one instant.
BEER_SETTLE_MOVE = 0.003
BEER_SETTLE_WINDOW_S = 1.0
BEER_SETTLE_TIMEOUT_S = 15.0

# Both of the streams this node reads are worth only their newest message:
# an old /joint_states is a lie about where the fingers are, and an old model
# pose is a lie about where the bottle is. Depth 1 and best-effort, so a
# backlog is dropped rather than queued and read later as if it were now.
#
# This is not tuning. The first run to get as far as the grasp reported
# "clamped at 0.0000 rad" for a gripper that was in fact closed to 0.75 on
# nothing, and so passed the check that exists to catch exactly that. The
# reading was a second old: dynamic_pose/info arrives at 60Hz carrying ~40
# transforms, and on the default reliable queue it was both filling the
# executor and letting joint_states fall behind it.
FRESH = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# Lift used when taking the opener off its post and the beer out of its well.
PICK_LIFT = 0.050


class OpenActionServer(Node):

    def __init__(self):
        super().__init__('open_action_server')
        cb_group = ReentrantCallbackGroup()
        # Each of the two high-rate subscriptions gets its OWN mutually
        # exclusive group, so neither can take more than one executor thread
        # and neither can be held up behind the other. The clients and the
        # action stay reentrant: they have to be, because the action callback
        # blocks on futures that those clients complete.
        state_group = MutuallyExclusiveCallbackGroup()
        pose_group = MutuallyExclusiveCallbackGroup()

        self.joint_state = None
        self._poses = {}
        self._poses_lock = threading.Lock()

        self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, FRESH,
            callback_group=state_group)
        self.create_subscription(
            TFMessage, POSE_TOPIC, self._on_poses, FRESH,
            callback_group=pose_group)

        self._detach_pub = self.create_publisher(Empty, DETACH_TOPIC, 10)
        self._scene_client = self.create_client(
            ApplyPlanningScene, 'apply_planning_scene', callback_group=cb_group)

        self.arm_a = Arm(self, '', 'arm A', cb_group, ARM_A_HOME)
        self.arm_b = Arm(self, 'b_', 'arm B', cb_group, ARM_B_HOME)
        for _name, _source in sorted(POINT_SOURCES.items()):
            self.get_logger().info(f'point {_name}: {_source}')
        # Where each gripper's fingers actually stopped when it took hold.
        # Set by the grasp, read by the grip checks; there is no commanded
        # clamp angle to compare against because nothing commands one.
        self._beer_stall = None
        self._opener_stall = None

        self._action_server = ActionServer(
            self, OpenBottle, 'open_bottle',
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        self.get_logger().info('open_action_server ready')

    # ---- state ----------------------------------------------------------

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def _on_poses(self, msg):
        """Cache every model pose Gazebo reports.

        The bridge turns Pose_V into a TFMessage, so each entry arrives as a
        transform whose child_frame_id is the entity's name. Link poses come
        through on the same topic and are relative to their model rather than
        to the world, so only names we know to be MODELS are ever looked up
        -- which is why the three constants above exist instead of a
        substring match.
        """
        with self._poses_lock:
            for tf in msg.transforms:
                t = tf.transform.translation
                self._poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def _pose(self, name, timeout_s=5.0):
        """World position of a model, or None if it never turns up."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._poses_lock:
                if name in self._poses:
                    return self._poses[name]
            time.sleep(0.05)
        self.get_logger().error(
            f'no pose for "{name}" on {POSE_TOPIC} after {timeout_s:.0f}s')
        return None

    # ---- planning scene -------------------------------------------------

    @staticmethod
    def _obj(object_id, primitive, xyz):
        """Build a collision object in arm A's base_link.

        Everything goes in base_link, including the objects arm B works on.
        MoveIt resolves collision objects into the model's own planning frame
        whatever frame they were declared in, so there is no reason to state
        any of this twice, and a second copy in b_base_link would be a second
        set of numbers to get wrong.
        """
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        pose.orientation.w = 1.0
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = object_id
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    @staticmethod
    def _cyl(height, radius):
        return SolidPrimitive(type=SolidPrimitive.CYLINDER,
                              dimensions=[height, radius])

    @staticmethod
    def _attached(object_id, link, primitive, xyz, quat, touch_links):
        """Build an object carried by a gripper, as MoveIt sees it.

        WHY ATTACH AT ALL. The pour server never needs this: everything it
        does while holding a bottle is Cartesian and unchecked, so a bottle
        that is really in the gripper while its scene object sits at its
        station costs nothing. Two arms breaks that in both directions.

        Going one way, the stale object is where the gripper now IS. The
        first run to get this far failed six planning attempts with
        INVALID_MOTION_PLAN because arm B was holding the opener and the
        opener was also still a cylinder standing on its post, with arm B's
        fingers inside it -- a start state in collision, which OMPL reports
        as an ordinary planning failure.

        Going the other way, the object is NOT where the gripper is, and the
        other arm plans through the space the carried thing is actually in.

        Attaching fixes both: the object moves with the link, and the arm
        holding it stops colliding with it.
        """
        attached = AttachedCollisionObject()
        attached.link_name = link
        attached.touch_links = list(touch_links)
        obj = CollisionObject()
        obj.header.frame_id = link
        obj.id = object_id
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(v) for v in xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = quat
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        attached.object = obj
        return attached

    def _gripper_links(self, prefix):
        """Every link of one gripper, for touch_links.

        An attached object is in permanent contact with the fingers holding
        it. Without these listed, the scene is in collision from the moment
        the object is attached and nothing plans at all.
        """
        return [prefix + name for name in (
            'tool0', 'wrist_3_link',
            'robotiq_85_base_link',
            'robotiq_85_left_knuckle_link', 'robotiq_85_right_knuckle_link',
            'robotiq_85_left_inner_knuckle_link',
            'robotiq_85_right_inner_knuckle_link',
            'robotiq_85_left_finger_link', 'robotiq_85_right_finger_link',
            'robotiq_85_left_finger_tip_link',
            'robotiq_85_right_finger_tip_link',
        )]

    def _apply(self, collision_objects=(), attached_objects=()) -> bool:
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = list(collision_objects)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = list(attached_objects)
        response = block_on(
            self._scene_client.call_async(ApplyPlanningScene.Request(scene=scene)),
            timeout_sec=10.0)
        if response is None or not response.success:
            self.get_logger().error('planning scene update was refused')
            return False
        return True

    @staticmethod
    def _remove(object_id):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        return obj

    def _take_opener(self) -> bool:
        """Move the opener into arm B's hand, leaving just the post behind.

        The carried envelope is one cylinder around the whole opener rather
        than the bell and shaft separately: it is only ever used to keep the
        arms and the counter clear of it, and the bell is the widest part, so
        one cylinder at the bell's radius is both simpler and conservative.

        Its pose is in b_tool0's frame, where +z is the approach direction
        and +y is up (see side_quat). The opener's centre is
        OPENER_HEIGHT/2 up from the rim and the pads hold it at
        OPENER_GRIP_Z, so the centre sits the difference BELOW the grip
        point, and the grip point is GRIP_AHEAD_OF_TOOL0 along +z. The
        quaternion is -90 degrees about x, which is what puts a cylinder's
        own axis (its local z) onto tool0's +y.
        """
        centre_below_grip = L.OPENER_GRIP_Z - L.OPENER_HEIGHT / 2.0
        return self._apply(
            collision_objects=[
                self._remove('opener_station'),
                self._obj('holster_post',
                          self._cyl(L.HOLSTER_POST_TOP, L.CAP_RADIUS),
                          (*L.station_in_arm('opener', 'a')[:2],
                           L.HOLSTER_POST_TOP / 2.0)),
            ],
            attached_objects=[
                self._attached(
                    'opener', 'b_tool0',
                    self._cyl(L.OPENER_HEIGHT, L.BELL_OUTER_RADIUS),
                    (0.0, -centre_below_grip, L.GRIP_AHEAD_OF_TOOL0),
                    UPRIGHT_IN_TOOL0, self._gripper_links('b_')),
            ])

    def _release_opener(self) -> bool:
        detached = AttachedCollisionObject()
        detached.link_name = 'b_tool0'
        detached.object.id = 'opener'
        detached.object.operation = CollisionObject.REMOVE
        opener = L.station_in_arm('opener', 'a')
        return self._apply(
            collision_objects=[
                self._remove('holster_post'),
                self._obj('opener_station',
                          self._cyl(L.OPENER_REST_RIM_Z + L.OPENER_HEIGHT,
                                    L.BELL_OUTER_RADIUS),
                          (opener[0], opener[1],
                           (L.OPENER_REST_RIM_Z + L.OPENER_HEIGHT) / 2.0)),
            ],
            attached_objects=[detached])

    def _take_beer(self) -> bool:
        """Beer into arm A's hand. Same reasoning as _take_opener.

        The envelope runs the whole bottle including the cap, at the body's
        radius -- the widest part -- so it is a cylinder much fatter than the
        neck arm B has to reach over. That is the conservative direction and
        it is why arm B's last checked pose is 60mm clear rather than 25.
        """
        centre_below_grip = L.BEER_GRASP_HEIGHT - L.cap_top_above_base() / 2.0
        return self._apply(
            collision_objects=[self._remove('beer_bottle')],
            attached_objects=[
                self._attached(
                    'beer', 'tool0',
                    self._cyl(L.cap_top_above_base(), L.BEER_BODY_RADIUS),
                    (0.0, -centre_below_grip, L.GRIP_AHEAD_OF_TOOL0),
                    UPRIGHT_IN_TOOL0, self._gripper_links('')),
            ])

    def _release_beer(self) -> bool:
        detached = AttachedCollisionObject()
        detached.link_name = 'tool0'
        detached.object.id = 'beer'
        detached.object.operation = CollisionObject.REMOVE
        beer = L.station_in_arm('beer', 'a')
        return self._apply(
            collision_objects=[
                self._obj('beer_bottle',
                          self._cyl(L.BEER_KEEPOUT_HEIGHT, L.BEER_KEEPOUT_RADIUS),
                          (beer[0], beer[1], L.BEER_KEEPOUT_HEIGHT / 2.0)),
            ],
            attached_objects=[detached])

    def _publish_obstacles(self) -> bool:
        """Add what this action needs to the scene, without disturbing the rest.

        is_diff, so this adds to whatever is already there rather than
        replacing it.

        THE COUNTER IS PUBLISHED HERE, and it did not use to be. The
        argument for leaving it out was that bartender_pour publishes it at
        the start of every pour goal, and re-sending it from a second node
        would mean two places deciding where the counter is. That was wrong
        twice over. An open goal on a freshly started stack runs with no
        pour behind it, so the counter was simply absent -- and MoveIt plans
        happily through a worktop nobody has told it about.

        It is not hypothetical and it is not subtle. Traced on the
        redesigned bar, arm B reached for the opener through the bar: the
        configuration it picked put the forearm 236mm BELOW the counter top
        and the wrist 41mm below, which left the gripper inside the
        worktop's collision geometry where its fingers could not close at
        all. The goal failed as "fingers closed all the way without meeting
        anything 24.0mm wide", which is an honest report of a gripper jammed
        in a table. The old layout never showed it only because the holster
        happened to sit where the branch above the counter was also the
        nearest one.

        The duplication argument no longer applies either: the counter's
        size and pose are decided in layout.py, bartender_pour restates them
        in arm A's frame from the same source, and test_layout.py checks the
        two agree. Both nodes publish the same box under the same object id,
        so whichever runs first wins and the second is a no-op.

        The rest of what this adds is the three things only this action
        knows about: the beer, its stand and the opener's holster. There
        used to be a fourth, a box for arm B's pedestal; arm B stands on the
        bar top now, so there is no pedestal and the volume under it is
        inside the counter box.

        WHAT IS NOT IN HERE, and should be: the whiskey and the cola. They
        stand on the same counter, 300mm tall, and no part of this action
        tells the planner they exist -- so a path straight through either of
        them is accepted as valid. On the old layout that was a live bug:
        the holster sat 112mm from the whiskey and the carried opener hooked
        it, 71.7mm of it, on the way past. The redesigned bar puts the
        holster 0.90m away on a different row, and the opener's crossing now
        clears the nearest bottle by 250mm, so the bug has no geometry left
        to bite on -- but the hole in the scene is still a hole, and the
        thing keeping the opener off the bottles is still OPENER_TRANSIT_Z
        rather than the planner. Adding them here is the fix. It is not done
        yet for the reason it never was: an obstacle that makes arm B's pick
        unplannable would be a worse bug than the one it closes, and that
        has not been measured on the new layout.

        The beer's object is a keep-out volume rather than the bottle's own
        shape; see BEER_KEEPOUT_RADIUS in layout.py for why, and for why its
        height is what makes an approach from directly above still legal. It
        is swapped for a true-sized attached object the moment the bottle is
        picked up.
        """
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('apply_planning_scene not available')
            return False

        beer = L.station_in_arm('beer', 'a')
        opener = L.station_in_arm('opener', 'a')

        counter = L.to_arm((L.COUNTER_CENTRE[0], L.COUNTER_CENTRE[1],
                            L.COUNTER_Z - COUNTER_THICKNESS / 2.0),
                           L.ARM_A_ORIGIN, L.ARM_A_YAW)

        scene = PlanningScene()
        scene.world.collision_objects = [
            # Same id and same box bartender_pour publishes, from the same
            # numbers. Whichever node gets there first wins.
            self._obj('bar_counter',
                      SolidPrimitive(type=SolidPrimitive.BOX,
                                     dimensions=[L.COUNTER_SIZE[0],
                                                 L.COUNTER_SIZE[1],
                                                 COUNTER_THICKNESS]),
                      counter),
            self._obj('beer_bottle',
                      self._cyl(L.BEER_KEEPOUT_HEIGHT, L.BEER_KEEPOUT_RADIUS),
                      (beer[0], beer[1], L.BEER_KEEPOUT_HEIGHT / 2.0)),
            self._obj('beer_stand',
                      self._cyl(L.STAND_HEIGHT, L.BEER_STAND_RADIUS),
                      (beer[0], beer[1], L.STAND_HEIGHT / 2.0)),
            # Holster plus the opener standing on it, as one cylinder from
            # the counter to the top of the shaft. They are only ever
            # approached together and separating them would buy nothing.
            self._obj('opener_station',
                      self._cyl(L.OPENER_REST_RIM_Z + L.OPENER_HEIGHT,
                                L.BELL_OUTER_RADIUS),
                      (opener[0], opener[1],
                       (L.OPENER_REST_RIM_Z + L.OPENER_HEIGHT) / 2.0)),
        ]
        return self._apply(scene.world.collision_objects)

    # ---- measurement ----------------------------------------------------

    @staticmethod
    def _dist(a, b):
        return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))

    def _cap_displacement(self):
        """How far the cap is from where it would be if it were still on.

        Measured against the BOTTLE'S CURRENT pose, not against the station:
        the bottle may legitimately be anywhere (held in the air, back in its
        stand), and the only thing that means "open" is the cap not being on
        its mouth.
        """
        beer = self._pose(BEER_MODEL)
        cap = self._pose(CAP_MODEL)
        if beer is None or cap is None:
            return None
        mouth = (beer[0], beer[1], beer[2] + L.cap_bottom_above_base())
        return self._dist(cap, mouth)

    def _seating(self):
        """(depth the bell has descended over the cap, lateral offset)."""
        cap = self._pose(CAP_MODEL)
        opener = self._pose(OPENER_MODEL)
        if cap is None or opener is None:
            return None
        cap_top_z = cap[2] + L.CAP_HEIGHT
        return (cap_top_z - opener[2],
                math.hypot(opener[0] - cap[0], opener[1] - cap[1]))

    # ---- the sequence ---------------------------------------------------

    @staticmethod
    def _above(xyz, height=L.APPROACH_Z):
        return (xyz[0], xyz[1], height)

    def _fetch_opener(self, step) -> bool:
        b = self.arm_b
        tool = L.side_grasp_tool0(L.opener_grip_point(L.OPENER_REST_RIM_Z))

        step('reaching for the opener', 0.05)
        # Ramped, not a single command. At this point in a fresh session the
        # gripper has settled shut under gravity, so this is the largest move
        # it makes, and one goal across the whole range was seen stopping
        # two-thirds of the way.
        if not b.command_gripper(GRIPPER_OPEN_POS, ramp=True):
            return False
        if not b.approach_then('opener approach', self._above(tool), tool):
            return False
        if not b.move_cartesian(tool, label='opener descend'):
            return False
        if not b.arrived_at(tool):
            return False

        step('gripping the opener', 0.12)
        self._opener_stall = b.grasp(L.OPENER_SHAFT)
        if self._opener_stall is None:
            return False
        lifted = (tool[0], tool[1], tool[2] + PICK_LIFT)
        if not b.move_cartesian(lifted, label='opener lift'):
            return False
        if not self._take_opener():
            return False
        # UP TO CRUISING HEIGHT BEFORE GOING ANYWHERE. Two separate moves,
        # and the split matters: the first has to be short and vertical to
        # come off the post's lead-in without dragging across it, and the
        # second has to happen before anything swings sideways.
        #
        # PICK_LIFT alone leaves the bell rim 50mm above the counter, and the
        # next move is a joint-space swing across a counter whose whiskey
        # stands 112mm away and 300mm tall -- and which is not in the
        # planning scene at all, so nothing would refuse that path. See
        # OPENER_TRANSIT_Z in layout.py.
        #
        # Vertical and Cartesian rather than folded into the next joint move,
        # because a joint move to a high pose is free to get there by any
        # route it likes, including through the bottle this is climbing to
        # avoid.
        cruise = (tool[0], tool[1], L.OPENER_TRANSIT_Z)
        return b.move_cartesian(cruise, label='opener to cruising height')

    def _pick_up_beer(self, step) -> bool:
        a = self.arm_a
        tool = L.side_grasp_tool0(L.beer_grip_point())

        step('reaching for the beer', 0.20)
        if not a.command_gripper(GRIPPER_OPEN_POS, ramp=True):
            return False
        standing = self._pose(BEER_MODEL)
        if not a.approach_then('beer approach', self._above(tool), tool):
            return False
        # Check the bottle is still standing where it was before closing on
        # it. Without this a bottle knocked over on the way in is discovered
        # as "fingers closed all the way without meeting anything", which is
        # true but says nothing about what went wrong or when.
        moved = self._pose(BEER_MODEL)
        if standing and moved and self._dist(standing, moved) > 0.005:
            self.get_logger().error(
                f'the beer moved {self._dist(standing, moved) * 1000:.0f}mm '
                f'while arm A was reaching for it, so it is no longer where '
                f'the grasp is aimed')
            return False
        if not a.move_cartesian(tool, label='beer descend'):
            return False
        # Confirm it ARRIVED, not merely that the move was accepted. The
        # descent is 215mm and a Cartesian plan is allowed back at 95% of
        # its path, which is 10mm -- and 10mm up the neck of this bottle is
        # a different diameter. Without this the symptom is a grasp that
        # closes straight past the neck and reports finding nothing of the
        # right width, which is true and says nothing about why.
        if not a.arrived_at(tool):
            return False

        step('gripping the beer', 0.30)
        self._beer_stall = a.grasp(L.BEER_WIDTH)
        if self._beer_stall is None:
            return False
        held = L.side_grasp_tool0(L.beer_grip_point(L.HOLD_LIFT))
        if not a.move_cartesian(held, label='beer lift'):
            return False

        # Is it actually off the counter? The one thing a bottle standing in
        # its well cannot fake, and the reason the whole claim of this
        # sequence -- that one arm is holding what the other pushes on --
        # is not taken on trust.
        #
        # Checked HERE, with the bottle lifted and nothing pushing on it,
        # rather than after the press. Measured after the press it reads 20mm
        # low on runs that are working perfectly well, because a 6mm
        # interference between two position-controlled arms deflects the one
        # being pushed on. That deflection is the push doing its job; it is
        # not the grip failing, and a check that cannot tell them apart is
        # not worth having. What guards the press itself is the shift limit
        # and arm A's own grip check.
        standing = self._pose(BEER_MODEL)
        clearance = (standing[2] - L.COUNTER_Z) if standing else 0.0
        if clearance < L.MIN_HELD_CLEARANCE:
            self.get_logger().error(
                f'the beer is only {clearance * 1000:.0f}mm off the counter '
                f'after the lift, so it is not really being held '
                f'(expected about {L.HOLD_LIFT * 1000:.0f}mm)')
            return False
        self.get_logger().info(
            f'arm A has the beer {clearance * 1000:.0f}mm off the counter')
        return self._take_beer()

    def _seated_tool0(self):
        """Where arm B's flange must be for the bell to sit home on the cap.

        Solved from the cap's MEASURED pose, not from the station it nominally
        stands at, and that is the difference between this working and not.

        The bottle does not stay exactly where it was put. Closing on a round
        neck rolls it a few millimetres before the pads stall -- observed 5 to
        8mm -- and the lift then carries that error up with it. Aiming the
        bell at the station instead put it 7.3mm off the cap, which is more
        than the 2.5mm the bore clears it by, so it landed ON the cap rather
        than over it and drove the bottle sideways.

        There is no reason to guess. The cap's pose is on the same topic
        everything else here is measured from, so the opener is aimed at the
        cap that exists rather than the one the layout describes.
        """
        cap = self._pose(CAP_MODEL)
        if cap is None:
            return None
        in_b = L.to_arm(cap, L.ARM_B_ORIGIN, L.ARM_B_YAW)
        rim_z = in_b[2] + L.CAP_HEIGHT - L.BELL_HEIGHT
        return L.side_grasp_tool0((in_b[0], in_b[1], rim_z + L.OPENER_GRIP_Z))

    def _wait_for_beer_to_settle(self):
        """Poll the held beer until it stops moving, and say how long it took.

        See BEER_SETTLE_MOVE for why this polls rather than sleeping a fixed
        time. Returns the settled pose, or the last pose read if it never
        settled within BEER_SETTLE_TIMEOUT_S -- logged, not failed on, since
        aiming at a slow residual creep is still far better than aiming at
        the peak of the swing this exists to wait out.
        """
        start = time.time()
        deadline = start + BEER_SETTLE_TIMEOUT_S
        history = []
        where = self._pose(BEER_MODEL)
        if where is None:
            return None
        while time.time() < deadline:
            now = time.time()
            where = self._pose(BEER_MODEL)
            if where is None:
                return None
            history.append((now, where))
            window = [w for t, w in history if t >= now - BEER_SETTLE_WINDOW_S]
            if history[0][0] <= now - BEER_SETTLE_WINDOW_S:
                xs, ys, zs = zip(*window)
                spread = math.sqrt((max(xs) - min(xs)) ** 2
                                   + (max(ys) - min(ys)) ** 2
                                   + (max(zs) - min(zs)) ** 2)
                if spread < BEER_SETTLE_MOVE:
                    self.get_logger().info(
                        f'beer settled after {now - start:.1f}s')
                    return where
            time.sleep(0.05)
        self.get_logger().warn(
            f'beer had not settled within {BEER_SETTLE_TIMEOUT_S:.0f}s; '
            f'aiming at it anyway')
        return where

    def _press(self, step):
        """Bring the opener down onto the cap and push.

        Returns (how far the bottle moved, how deep the bell seated, how far
        off centre it was), or None if the sequence could not be run at all.
        The caller decides whether those numbers are good enough; this only
        reports them, and retries the descent if the first one misses.
        """
        b = self.arm_b
        self._wait_for_beer_to_settle()
        seated = self._seated_tool0()
        if seated is None:
            return None
        approach = (seated[0], seated[1], seated[2] + PRE_PRESS_GAP)

        step('bringing the opener over the cap', 0.45)
        if not b.approach_then('over the cap', approach, seated):
            return None

        for attempt in range(1, SEAT_ATTEMPTS + 1):
            # Re-aim before EVERY descent, and wait for the swing to be done
            # first. The first measurement was taken before arm B had moved
            # at all, and the bottle does not stand perfectly still in the
            # meantime -- it hangs from the pads and swings as arm A settles,
            # for real seconds rather than an instant. See
            # BEER_SETTLE_MOVE for how much and why a re-read alone is not
            # enough to be sure it is over.
            self._wait_for_beer_to_settle()
            seated = self._seated_tool0() or seated
            # may_stall: `seated` is by construction the pose where the
            # crown plate is TOUCHING the cap, so arriving at it exactly is
            # the boundary case rather than the normal one. A descent that
            # stops early has hit something -- usually the cap's edge or the
            # bottle's shoulder when the aim is off -- and the right answer
            # is to go on and MEASURE the seating, which the gate below
            # already does, not to abandon the attempt with no numbers.
            #
            # Before the arm controllers had a goal tolerance this never
            # arose: the controller called every descent a success. The
            # first run with one gave up as "the press did not run" on an
            # attempt whose own measurement said the bell was 28.6mm off
            # centre -- which is the far more useful thing to report.
            if not b.move_cartesian(seated, label='seat the bell on the cap',
                                    may_stall=True):
                return None

            before = self._pose(BEER_MODEL)
            if before is None:
                return None

            step('pushing', 0.60)
            pressed = (seated[0], seated[1], seated[2] - L.PRESS_TRAVEL)
            # may_stall: there is a bottle in the way, which is the entire
            # point. A press that DID arrive would mean nothing was under it.
            if not b.move_cartesian(pressed, label='push down on the cap',
                                    may_stall=True):
                return None

            after = self._pose(BEER_MODEL)
            if after is None:
                return None
            seating = self._seating()
            if seating is None:
                return None
            depth, offset = seating
            if (depth >= L.SEAT_DEPTH_MIN and offset <= L.SEAT_OFFSET_MAX):
                return self._dist(before, after), depth, offset

            # Not home. Back off and come down again rather than giving up:
            # a miss here is usually the last few millimetres going astray --
            # the cap drifting between the aim and the arrival, or the arm
            # landing a little wide -- and the second descent starts from a
            # fresh measurement taken from directly above it.
            self.get_logger().warn(
                f'attempt {attempt}/{SEAT_ATTEMPTS}: bell seated '
                f'{depth * 1000:.1f}mm of {L.BELL_HEIGHT * 1000:.0f} and '
                f'{offset * 1000:.1f}mm off centre; backing off to try again')
            if attempt == SEAT_ATTEMPTS:
                return self._dist(before, after), depth, offset
            if not b.move_cartesian(
                    (seated[0], seated[1], seated[2] + PRE_PRESS_GAP),
                    label='back off the cap'):
                return None
        return None

    def _shed_cap(self) -> bool:
        """Tip the bottle over far enough that a loose cap cannot stay on it.

        Arm A, not arm B, and the reasoning is in layout.py beside SHED_TILT:
        the opener's bell was tried as a sweep and how much of the cap it
        still has hold of turns out to depend on millimetres. A bottle at 40
        degrees does not need anything to depend on.

        The bottle turns about its own grip point, so the pads do not slide
        along it, and the tilt is interpolated so the planner produces a
        rotation rather than a swing.
        """
        grip = L.beer_grip_point(L.HOLD_LIFT)
        poses = []
        for step_index in range(1, L.SHED_STEPS + 1):
            theta = L.SHED_TILT * step_index / L.SHED_STEPS
            poses.append(pose_at(L.tilted_tool0(grip, theta), side_quat(theta)))
        if not self.arm_a.follow_cartesian(poses, label='tip the bottle'):
            return False
        time.sleep(L.SHED_SETTLE_S)
        # ...and back upright, so the bottle can be put down afterwards.
        back = [pose_at(L.tilted_tool0(grip, L.SHED_TILT * i / L.SHED_STEPS),
                        side_quat(L.SHED_TILT * i / L.SHED_STEPS))
                for i in range(L.SHED_STEPS - 1, -1, -1)]
        return self.arm_a.follow_cartesian(back, label='bottle upright again')

    def _stow(self, step) -> bool:
        """Put the opener back on its post and the beer back in its well."""
        b, a = self.arm_b, self.arm_a

        step('putting the opener back', 0.85)
        down = L.side_grasp_tool0(L.opener_grip_point(L.OPENER_REST_RIM_Z))
        # Carrying the opener, so this crosses at OPENER_TRANSIT_Z and not at
        # the empty-gripper APPROACH_Z -- the return trip passes the same
        # bottles the outbound one does.
        if not b.approach_then('over the holster',
                               self._above(down, L.OPENER_TRANSIT_Z), down):
            return False
        # Straight down onto the post. The post's lead-in is what turns the
        # residual xy error into a funnel rather than an opener parked across
        # the top of it, and it can only do that if the descent is vertical.
        #
        # may_stall: THE RIM LANDS ON THE COUNTER. Like every other
        # setting-down move here this one cannot arrive, and it only looked
        # as though it did while the arm controllers had no goal tolerance
        # and reported success wherever they stopped. Once they had one this
        # came back as `execution "opener onto the post" failed` on a run
        # that had already got the cap off -- "opened, but could not stow".
        if not b.move_cartesian(down, label='opener onto the post',
                                may_stall=True):
            return False
        if not b.command_gripper(GRIPPER_OPEN_POS, ramp=True):
            return False
        if not self._release_opener():
            return False
        # Empty by now -- the opener is back on its post -- so the ordinary
        # transit height is the right one to climb to.
        if not b.move_cartesian(self._above(down), label='opener retreat'):
            return False
        if not b.go_home():
            return False

        step('putting the beer back', 0.93)
        tool = L.side_grasp_tool0(L.beer_grip_point())
        # Via approach_then, exactly as the pick does, and for the same
        # reason: the descent has to be possible from wherever the arm is
        # standing, and after the tilt it is not always. Measured, a straight
        # drop from the hold pose came back at 0.50 of a 50mm path. Going up
        # to the transit height first lets a configuration be chosen that can
        # make the descent, and it is a shorter detour than it sounds.
        #
        # The last part is vertical for the same reason the pick's is: the
        # stand's lead-in only funnels an xy error if the bottle comes
        # straight down onto it.
        if not a.approach_then('above the stand', self._above(tool), tool):
            return False
        # may_stall, for the same reason as the opener above: the bottle's
        # base comes to rest on the counter inside the stand's ring, so the
        # last millimetres of the command are always refused.
        if not a.move_cartesian(tool, label='beer into the stand',
                                may_stall=True):
            return False
        if not a.command_gripper(GRIPPER_OPEN_POS, ramp=True):
            return False
        if not self._release_beer():
            return False
        if not a.move_cartesian(self._above(tool), label='beer retreat'):
            return False
        return a.go_home()

    # ---- action ---------------------------------------------------------

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        bottle = request.bottle_id or 'beer'

        def step(state, progress):
            feedback = OpenBottle.Feedback()
            feedback.state = state
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'[{progress * 100:3.0f}%] {state}')

        if bottle != 'beer':
            return self._fail(goal_handle,
                              f'no capped bottle called "{bottle}"; '
                              f'the scene has one, and it is "beer"')

        self._beer_stall = self._opener_stall = None
        already = self._cap_displacement()
        if already is None:
            return self._fail(
                goal_handle,
                f'no model poses on {POSE_TOPIC} -- is the ros_gz bridge in '
                f'sim.launch.py running?')
        if already > L.CAP_FREE_MIN:
            # Not a failure of this attempt so much as of the premise.
            return self._fail(
                goal_handle,
                f'the beer is already open (cap is {already * 1000:.0f}mm '
                f'off the mouth). Ignition\'s DetachableJoint cannot '
                f're-attach, so restart the sim to run this again.')

        if not self._publish_obstacles():
            return self._fail(goal_handle, 'could not publish the scene')

        # Home both arms before anything else. Not tidiness: after the sim
        # comes up the arms are wherever gravity left them, and IK is seeded
        # from a fixed configuration (see Arm.solve_ik) so that the same
        # target gives the same answer every run. Starting from an unknown
        # pose makes the first goal of a session behave differently from
        # every one after it, which is exactly the kind of difference that
        # gets blamed on the physics.
        step('homing both arms', 0.02)
        for arm in (self.arm_a, self.arm_b):
            if not arm.wait_for_controllers():
                return self._fail(goal_handle,
                                  f'{arm.label} has no controllers')
        if not self.arm_b.go_home():
            return self._fail(
                goal_handle, f'arm B would not go home{self._why(self.arm_b)}')
        if not self.arm_a.go_home():
            return self._fail(
                goal_handle, f'arm A would not go home{self._why(self.arm_a)}')

        if not self._fetch_opener(step):
            return self._fail(
                goal_handle,
                f'arm B could not pick up the opener{self._why(self.arm_b)}')
        if not self._pick_up_beer(step):
            return self._fail(
                goal_handle,
                f'arm A could not pick up the beer{self._why(self.arm_a)}')

        pressed = self._press(step)
        if pressed is None:
            return self._fail(
                goal_handle, f'the press did not run{self._why(self.arm_b)}')
        shift, depth, offset = pressed

        # The gate. Everything up to here was motion; this is the part that
        # decides whether the cap has any business coming off.
        held = self._pose(BEER_MODEL)
        self.get_logger().info(
            f'seated {depth * 1000:.1f}mm of {L.BELL_HEIGHT * 1000:.0f}, '
            f'{offset * 1000:.1f}mm off centre; bottle moved '
            f'{shift * 1000:.1f}mm and is '
            f'{((held[2] - L.COUNTER_Z) * 1000) if held else 0:.0f}mm off '
            f'the counter')

        if not self.arm_a.still_holding(self._beer_stall):
            return self._fail(goal_handle,
                              'arm A is not holding the beer any more',
                              bottle_shift=shift)

        if shift > L.BOTTLE_SHIFT_MAX:
            return self._fail(
                goal_handle,
                f'the beer moved {shift * 1000:.1f}mm while being pushed on '
                f'(limit {L.BOTTLE_SHIFT_MAX * 1000:.0f}mm), so the push went '
                f'into moving it rather than into the cap',
                bottle_shift=shift)
        if depth < L.SEAT_DEPTH_MIN or offset > L.SEAT_OFFSET_MAX:
            return self._fail(
                goal_handle,
                f'the opener is not seated on the cap: {depth * 1000:.1f}mm '
                f'down (needs {L.SEAT_DEPTH_MIN * 1000:.0f}) and '
                f'{offset * 1000:.1f}mm off centre '
                f'(allows {L.SEAT_OFFSET_MAX * 1000:.1f})',
                bottle_shift=shift)

        step('popping the cap', 0.70)
        self._detach_pub.publish(Empty())

        step('lifting the opener clear', 0.76)
        seated_tool = self._seated_tool0()
        if seated_tool is None:
            return self._fail(goal_handle, 'lost sight of the cap',
                              bottle_shift=shift)
        # Up first, then back, as two axis-aligned segments rather than one
        # diagonal. The diagonal is 144mm from a pose that is pressed into a
        # bottle, and a single Cartesian segment over that distance came back
        # at 0.23 of its path. Neither half is difficult on its own.
        lifted = (seated_tool[0], seated_tool[1],
                  seated_tool[2] + RETREAT_LIFT)
        drawn_back = (lifted[0] - RETREAT_BACK, lifted[1], lifted[2])
        for where, label in ((lifted, 'lift the opener off the cap'),
                             (drawn_back, 'draw the opener back')):
            if not self.arm_b.move_cartesian(where, label=label):
                return self._fail(
                    goal_handle,
                    f'arm B could not {label}{self._why(self.arm_b)}',
                    bottle_shift=shift)

        step('tipping the cap off', 0.80)
        if not self._shed_cap():
            return self._fail(
                goal_handle,
                f'arm A could not tip the bottle to shed the '
                f'cap{self._why(self.arm_a)}',
                bottle_shift=shift)
        time.sleep(CAP_SETTLE_S)

        displacement = self._cap_displacement()
        if displacement is None:
            return self._fail(goal_handle, 'lost sight of the cap',
                              bottle_shift=shift)
        if displacement < L.CAP_FREE_MIN:
            return self._fail(
                goal_handle,
                f'the cap is still on the bottle ({displacement * 1000:.0f}mm '
                f'from the mouth); the detach was sent but nothing moved',
                bottle_shift=shift, cap_displacement=displacement)

        if request.stow_after and not self._stow(step):
            return self._fail(
                goal_handle,
                f'opened, but could not stow'
                f'{self._why_any(self.arm_a, self.arm_b)}',
                bottle_shift=shift, cap_displacement=displacement)

        step('open', 1.0)
        goal_handle.succeed()
        result = OpenBottle.Result()
        result.success = True
        result.message = (
            f'cap off: it ended up {displacement * 1000:.0f}mm from the mouth, '
            f'and the bottle moved {shift * 1000:.1f}mm while being pushed on')
        result.cap_displacement_m = float(displacement)
        result.bottle_shift_m = float(shift)
        return result

    @staticmethod
    def _why(arm):
        """Append an arm's own last logged reason, if it left one.

        The caller's own message here is a stage-level summary ("could not
        pick up the opener"); the arm usually worked out something far more
        specific when it happened -- a stalled gripper, a plan that only
        reached 4% of its path -- and logged it. Before Arm.last_error
        existed that reason went nowhere once the caller only checked
        True/False, which is exactly the gap bartender_api's error-taxonomy
        classifier exposed: two very different faults (GRIPPER_NOT_FOLLOWING
        vs GRASP_STOPPED_WIDE) collapsed into the same coarse message here.
        """
        return f' ({arm.last_error})' if arm.last_error else ''

    @staticmethod
    def _why_any(*arms):
        """Like _why, for a stage where more than one arm could be at fault.

        _stow moves both arms in sequence, so a bare "could not stow" does
        not say which one -- showing both candidates' own last reason is
        honest about that ambiguity rather than guessing which arm to blame.
        """
        reasons = [f'{a.label}: {a.last_error}' for a in arms if a.last_error]
        return f' ({"; ".join(reasons)})' if reasons else ''

    def _fail(self, goal_handle, message, bottle_shift=0.0,
              cap_displacement=0.0):
        self.get_logger().error(message)
        goal_handle.abort()
        result = OpenBottle.Result()
        result.success = False
        result.message = message
        result.cap_displacement_m = float(cap_displacement)
        result.bottle_shift_m = float(bottle_shift)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OpenActionServer()
    # Eight, not four. One thread runs the action callback and spends nearly
    # all of its time blocked on a future; the rest have to service two arms'
    # worth of MoveGroup, ExecuteTrajectory, GripperCommand, IK, validity and
    # Cartesian clients, plus the two state subscriptions, and those two must
    # never be the ones that miss out -- everything this action decides, it
    # decides by reading them. Four was enough to make /joint_states a second
    # stale, which is enough to pass a failed grasp as a good one.
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
