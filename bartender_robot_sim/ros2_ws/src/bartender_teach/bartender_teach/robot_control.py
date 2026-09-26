"""The real robot's own controls, for the teach pendant.

Power, brakes, the program on the UR controller, the speed slider and
freedrive: the things you would otherwise have to walk over to the UR
pendant for. None of it exists in simulation. Every call here says so
rather than failing obscurely when it is used against the sim.

What these talk to, all started by workcell_real.launch.py (and so by
workcell_twin.launch.py):

    /ur_robot_state_helper/set_mode           power on/off, brakes, recovery
    /dashboard_client/{play,pause,stop,...}   the program on the controller
    /io_and_status_controller/set_speed_slider
    /io_and_status_controller/{robot_mode,safety_mode,robot_program_running}
    /speed_scaling_state_broadcaster/speed_scaling
    /controller_manager/switch_controller     for freedrive
    /freedrive_mode_controller/enable_freedrive_mode

FREEDRIVE takes the arm away from ROS: the trajectory controller is
switched off and the arm goes limp under your hand, so it is switched in
and out here as one operation, and the pendant refuses motion commands
while it is on. The freedrive controller also drops out by itself if it
stops hearing from us for a second, which is why a timer keeps telling it.
"""
import threading

from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64

# The message and service packages come with ur_robot_driver. Imported
# lazily so the pendant still runs against the simulation on a machine that
# has never had the driver installed.
try:
    from controller_manager_msgs.srv import SwitchController
    from std_srvs.srv import Trigger
    from ur_dashboard_msgs.action import SetMode
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    from ur_msgs.srv import SetSpeedSliderFraction
    HAVE_UR = True
except ImportError:                                         # pragma: no cover
    HAVE_UR = False

ARM_CONTROLLER = 'ur_arm_controller'
FREEDRIVE_CONTROLLER = 'freedrive_mode_controller'

# RobotMode.mode and SafetyMode.mode, as words. The UR pendant uses the
# same ones, so what this prints is what that screen would say.
ROBOT_MODES = {
    -1: 'NO_CONTROLLER', 0: 'DISCONNECTED', 1: 'CONFIRM_SAFETY',
    2: 'BOOTING', 3: 'POWER_OFF', 4: 'POWER_ON', 5: 'IDLE',
    6: 'BACKDRIVE', 7: 'RUNNING', 8: 'UPDATING_FIRMWARE',
}
SAFETY_MODES = {
    1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP', 4: 'RECOVERY',
    5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP',
    7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT',
    10: 'VALIDATE_JOINT_ID', 11: 'UNDEFINED_SAFETY_MODE',
    12: 'AUTOMATIC_MODE_SAFEGUARD_STOP',
    13: 'SYSTEM_THREE_POSITION_ENABLING_STOP',
}
MODE_POWER_OFF = 3
MODE_IDLE = 5
MODE_RUNNING = 7

# `robot VERB`: which dashboard services, in order, and what it means.
# `on` and `off` are not here: they go through the state helper's set_mode
# action, which waits for each transition (booting, idle, brakes) instead of
# firing the next request at a controller that is still in the last one.
DASHBOARD_VERBS = {
    'play': (('play',), 'program playing'),
    'pause': (('pause',), 'program paused'),
    'stop': (('stop',), 'program stopped'),
    # A protective stop leaves a popup on the UR pendant that has to go
    # before the stop can be unlocked.
    'unlock': (('close_safety_popup', 'unlock_protective_stop'),
               'protective stop cleared; `robot play` to carry on'),
}

NOT_REAL = ('is not available. The robot controls need the real robot '
            'connected (workcell_real or workcell_twin, without '
            'use_fake_hardware); the simulation and the dry run have none.')

FREEDRIVE_KEEPALIVE_S = 0.1     # the controller times out after 1s
SET_MODE_TIMEOUT_S = 60.0       # power on + brake release takes ~20s


class RobotControl:
    """The ROS side of the real robot's controls. Owned by TeachNode."""

    def __init__(self, node, callback_group):
        self._node = node
        self._cb = callback_group
        self.robot_mode = None
        self.safety_mode = None
        self.program_running = None
        self.speed_scaling = None
        self.freedrive = False
        self._freedrive_timer = None
        self._lock = threading.Lock()
        if not HAVE_UR:
            return

        # Transient local, like the publisher: these are latched, and the
        # driver only republishes them on a change.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            RobotMode, '/io_and_status_controller/robot_mode',
            lambda m: setattr(self, 'robot_mode', m.mode), latched,
            callback_group=callback_group)
        node.create_subscription(
            SafetyMode, '/io_and_status_controller/safety_mode',
            lambda m: setattr(self, 'safety_mode', m.mode), latched,
            callback_group=callback_group)
        node.create_subscription(
            Bool, '/io_and_status_controller/robot_program_running',
            lambda m: setattr(self, 'program_running', m.data), latched,
            callback_group=callback_group)
        node.create_subscription(
            Float64, '/speed_scaling_state_broadcaster/speed_scaling',
            lambda m: setattr(self, 'speed_scaling', m.data), 10,
            callback_group=callback_group)

        self._set_mode = ActionClient(
            node, SetMode, '/ur_robot_state_helper/set_mode',
            callback_group=callback_group)
        self._dashboard = {
            name: node.create_client(Trigger, f'/dashboard_client/{name}',
                                     callback_group=callback_group)
            for verbs, _ in DASHBOARD_VERBS.values() for name in verbs
        }
        self._resend = node.create_client(
            Trigger, '/io_and_status_controller/resend_robot_program',
            callback_group=callback_group)
        self._speed = node.create_client(
            SetSpeedSliderFraction, '/io_and_status_controller/set_speed_slider',
            callback_group=callback_group)
        self._switch = node.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=callback_group)
        self._freedrive_pub = node.create_publisher(
            Bool, '/freedrive_mode_controller/enable_freedrive_mode', 10)

    # -- helpers ----------------------------------------------------------

    def _call(self, client, request, what, timeout=10.0):
        """Call a service; return (response, '') or (None, why)."""
        if not HAVE_UR:
            return None, (f'{what}: the ur_robot_driver message packages '
                          f'are not installed')
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f'{client.srv_name} {NOT_REAL}'
        resp = self._node._block_on(client.call_async(request), timeout)
        if resp is None:
            return None, f'{what}: no answer from {client.srv_name}'
        return resp, ''

    # -- status -----------------------------------------------------------

    def status(self):
        """Return what the robot is doing, as a dict of words. Never blocks."""
        return {
            'real': HAVE_UR and self.robot_mode is not None,
            'robot_mode': ROBOT_MODES.get(self.robot_mode),
            'safety_mode': SAFETY_MODES.get(self.safety_mode),
            'program_running': self.program_running,
            'speed_scaling': self.speed_scaling,
            'freedrive': self.freedrive,
        }

    # -- power and program ------------------------------------------------

    def set_mode(self, target, play=False):
        """Power the arm to `target` (MODE_*), waiting for it to get there."""
        if not HAVE_UR:
            return False, 'the ur_robot_driver message packages are not installed'
        if not self._set_mode.wait_for_server(timeout_sec=2.0):
            return False, f'/ur_robot_state_helper/set_mode {NOT_REAL}'
        goal = SetMode.Goal()
        goal.target_robot_mode = target
        # Never resume a program that was interrupted: it would carry on
        # with the move that caused the stop. `robot play` is separate.
        goal.stop_program = True
        goal.play_program = play
        handle = self._node._block_on(
            self._set_mode.send_goal_async(goal), timeout_sec=10.0)
        if handle is None or not handle.accepted:
            return False, 'set_mode goal rejected'
        result = self._node._block_on(handle.get_result_async(),
                                      timeout_sec=SET_MODE_TIMEOUT_S)
        if result is None:
            return False, f'robot did not reach {ROBOT_MODES[target]} in time'
        if not result.result.success:
            return False, result.result.message or 'set_mode failed'
        return True, ''

    def dashboard(self, verb):
        """Run one of DASHBOARD_VERBS. Returns (ok, message)."""
        names, done = DASHBOARD_VERBS[verb]
        if not HAVE_UR:
            return False, 'the ur_robot_driver message packages are not installed'
        for name in names:
            resp, why = self._call(self._dashboard[name], Trigger.Request(), name)
            if resp is None:
                return False, why
            # close_safety_popup "fails" when there is no popup, which is
            # not a reason to stop before unlocking.
            if not resp.success and name != 'close_safety_popup':
                return False, f'{name}: {resp.message}'
        return True, done

    def resend(self):
        """Restart ROS's control script on the robot (headless mode).

        In headless mode there is no program on the UR pendant to press
        Play on: the driver sends the script itself when it connects, and
        this sends it again after a stop.
        """
        if not HAVE_UR:
            return False, 'the ur_robot_driver message packages are not installed'
        resp, why = self._call(self._resend, Trigger.Request(),
                               'resend_robot_program')
        if resp is None:
            return False, why
        if not resp.success:
            return False, f'resend_robot_program: {resp.message}'
        return True, 'control script sent; the arm controller comes back by itself'

    def set_speed(self, fraction):
        """Set the UR pendant's speed slider, 0 < fraction <= 1."""
        if not HAVE_UR:
            return False, 'the ur_robot_driver message packages are not installed'
        req = SetSpeedSliderFraction.Request(speed_slider_fraction=float(fraction))
        resp, why = self._call(self._speed, req, 'set_speed_slider')
        if resp is None:
            return False, why
        if not resp.success:
            return False, 'the robot refused the speed (is it connected?)'
        return True, ''

    # -- freedrive --------------------------------------------------------

    def _switch_to(self, activate, deactivate):
        req = SwitchController.Request(
            activate_controllers=[activate], deactivate_controllers=[deactivate],
            strictness=SwitchController.Request.STRICT, activate_asap=True)
        req.timeout.sec = 5
        resp, why = self._call(self._switch, req, 'switch_controller')
        if resp is None:
            return False, why
        if not resp.ok:
            return False, (f'controller_manager would not switch {deactivate} '
                           f'-> {activate}; `ros2 control list_controllers` '
                           f'says what state they are in')
        return True, ''

    def _keepalive(self):
        self._freedrive_pub.publish(Bool(data=True))

    def set_freedrive(self, on):
        """Hand the arm to your hands (on) or back to ROS (off)."""
        if not HAVE_UR:
            return False, 'the ur_robot_driver message packages are not installed'
        with self._lock:
            if on == self.freedrive:
                return True, ''
            if on:
                ok, why = self._switch_to(FREEDRIVE_CONTROLLER, ARM_CONTROLLER)
                if not ok:
                    return False, why
                self.freedrive = True
                self._keepalive()
                self._freedrive_timer = self._node.create_timer(
                    FREEDRIVE_KEEPALIVE_S, self._keepalive,
                    callback_group=self._cb)
                return True, ''
            # Off: stop asking for it BEFORE switching back, so the arm is
            # never in freedrive with the trajectory controller active.
            if self._freedrive_timer is not None:
                self._freedrive_timer.cancel()
                self._node.destroy_timer(self._freedrive_timer)
                self._freedrive_timer = None
            self._freedrive_pub.publish(Bool(data=False))
            ok, why = self._switch_to(ARM_CONTROLLER, FREEDRIVE_CONTROLLER)
            # Marked off either way: we have stopped asking, and the
            # controller drops freedrive by itself a second later. What can
            # still be wrong is the arm controller not being back, and the
            # message says so.
            self.freedrive = False
            return ok, why
