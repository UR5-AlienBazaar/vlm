"""Make the Gazebo twin copy the real arm.

    ros2 run bartender_bringup twin_mirror

Reads the REAL robot's /joint_states and streams them into the twin's
position controller (/twin/twin_position_controller/commands), so the
simulated workcell shows the real one: every move, whether it came from
MoveIt, from bartender_teach or from someone jogging on the UR pendant.

One direction only, on purpose. Nothing in the twin can move the real
robot; if the twin and the robot ever disagree, the robot is right and the
twin is what is wrong. workcell_twin.launch.py starts this.

Why streaming positions and not forwarding trajectories: a forwarded
trajectory would only show the moves ROS itself made, and would show them
as planned rather than as executed. A protective stop halfway through a
move would leave the twin at the goal and the robot short of it.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# The order twin_position_controller's `joints` list is in
# (bartender_description/config/workcell_twin_controllers.yaml).
ARM_JOINTS = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)
KNUCKLE = 'robotiq_85_left_knuckle_joint'

# The knuckle's usable band in simulation. A knuckle parked on 0.0 stops
# responding for the rest of the run; see GRIPPER_LOWER_LIMIT in
# bartender_teach/teach_points.py. The real gripper reports 0.0 when open,
# so the twin is held just off it.
KNUCKLE_MIN = 0.02
KNUCKLE_MAX = 0.8

# The real robot is silent for longer than this: say so once, and hold the
# twin where it was rather than guess.
STALE_AFTER_S = 1.0


def twin_command(names, positions, last_knuckle=KNUCKLE_MIN):
    """Return the twin's command for one real /joint_states message.

    Returns (command list, knuckle used), or None when the message does not
    carry all six arm joints. Joints are matched by NAME: /joint_states is in
    whatever order the broadcaster registered them, and a slice that happens
    to work today is the arm moving the wrong joint tomorrow.

    The gripper is optional. The real one may not be publishing (it can be
    mocked or unplugged); the twin then keeps the last knuckle it had.
    """
    state = dict(zip(names, positions))
    if not all(j in state for j in ARM_JOINTS):
        return None
    arm = [float(state[j]) for j in ARM_JOINTS]
    if not all(math.isfinite(v) for v in arm):
        return None
    knuckle = state.get(KNUCKLE, last_knuckle)
    if not math.isfinite(knuckle):
        knuckle = last_knuckle
    knuckle = min(max(float(knuckle), KNUCKLE_MIN), KNUCKLE_MAX)
    return arm + [knuckle], knuckle


class TwinMirror(Node):
    """Copies the real joint states onto the twin at a fixed rate."""

    def __init__(self):
        super().__init__('twin_mirror')
        self.declare_parameter('source_topic', '/joint_states')
        self.declare_parameter(
            'target_topic', '/twin/twin_position_controller/commands')
        # 50 Hz is smooth to watch and far below the 500 Hz the real
        # broadcaster may publish at; the twin needs to look right, not to
        # be a control loop.
        self.declare_parameter('rate_hz', 50.0)

        source = self.get_parameter('source_topic').value
        target = self.get_parameter('target_topic').value
        rate = float(self.get_parameter('rate_hz').value)

        self._latest = None
        self._latest_time = None
        self._knuckle = KNUCKLE_MIN
        self._warned_stale = False
        self._warned_partial = False

        self._pub = self.create_publisher(Float64MultiArray, target, 10)
        self.create_subscription(JointState, source, self._on_state,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'mirroring {source} -> {target} at {rate:g} Hz')

    def _on_state(self, msg):
        self._latest = msg
        self._latest_time = self.get_clock().now()

    def _tick(self):
        if self._latest is None:
            return
        age = (self.get_clock().now() - self._latest_time).nanoseconds * 1e-9
        if age > STALE_AFTER_S:
            if not self._warned_stale:
                self.get_logger().warn(
                    f'no joint states from the real robot for {age:.1f}s; '
                    f'the twin is holding its last pose')
                self._warned_stale = True
            return
        if self._warned_stale:
            self.get_logger().info('real robot joint states are back')
            self._warned_stale = False

        result = twin_command(self._latest.name, self._latest.position,
                              self._knuckle)
        if result is None:
            if not self._warned_partial:
                self.get_logger().warn(
                    'joint states do not include all six arm joints '
                    f'({", ".join(self._latest.name) or "none"}); not '
                    f'mirroring those')
                self._warned_partial = True
            return
        command, self._knuckle = result
        self._pub.publish(Float64MultiArray(data=command))


def main(args=None):
    rclpy.init(args=args)
    node = TwinMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
