"""HTTP entry point for bartender_api.

    ros2 run bartender_api server [--perception ground_truth|camera|compare]

    GET  /world      what is on the bar, and who can reach it
    GET  /state      joints, tool pose, gripper, per arm
    POST /can        {"verb": "pour"|"open", "args": {...}}  -- "could you?"
    POST /move/point {"arm": "a", "point": "whiskey_approach"}
    POST /move/jog   {"arm": "a", "axis": "z", "amount": 20}
    POST /gripper    {"arm": "b", "position": 0.25}
    GET  /bottles    the bottles that can be picked (grab_<bottle> pipelines)
    POST /pick       {"bottle": "whiskey"}  -- run grab_whiskey
    GET  /drinks     the menu, and which drinks have all their scripts taught
    POST /make       {"drink": "whiskey_cola"}  -- run that drink's scripts
    POST /ask        {"text": "a whiskey coke please"}  -- which call that is (Jev; runs nothing)

Phase B (world/state/can) is read-only, no motion, no new risk. The three
movement routes are Phase C, scoped down to "simple movement" -- see
movement.py's own docstring for exactly what that does and does not cover
(no arbitrary joint or Cartesian targets; goto a taught point, jog
relative to where the arm is, or move the gripper -- the three things
Pendant already does with every bound this project has found a reason
for). `/do`, jobs, pipelines and a global stop are still not built.

Same pattern as bartender_teach/teach_gui.py, and for the same reasons: no
dependencies (http.server is stdlib), works from a laptop on the robot's
network, and degrades gracefully if anything about it misbehaves. The
movement routes reuse teach_gui's own locking pattern (one command at a
time, refused not queued) via movement.MovementBridge.

Binding
-------
127.0.0.1 by default. This process now moves the robot -- binding wide
open is a bad idea outside an isolated robot LAN. --host 0.0.0.0 prints a
warning every time.
"""
import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from bartender_open import layout as L
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from bartender_teach.point_store import (
    PointStore, PointStoreError, points_path_for,
)
from bartender_teach.teach_points import TeachNode

from .drink import jev, label, ocr, vlm
from . import feasibility, intent, menu, movement, perception, state_view, world

# Same topic, same QoS, same reasoning as open_action_server's _on_poses:
# depth 1 and best-effort, because a queued backlog is a lie about where
# something is, not a queue worth keeping.
POSE_TOPIC = '/world/bar_world/dynamic_pose/info'
FRESH = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class PoseCache(Node):
    """Caches the latest pose of every model on POSE_TOPIC, ground truth."""

    def __init__(self):
        super().__init__('bartender_api_poses')
        self._poses = {}
        self._lock = threading.Lock()
        self.create_subscription(TFMessage, POSE_TOPIC, self._on_poses, FRESH)

    def _on_poses(self, msg):
        with self._lock:
            for tf in msg.transforms:
                t = tf.transform.translation
                self._poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def get(self, name):
        with self._lock:
            return self._poses.get(name)

    def has_any(self):
        with self._lock:
            return bool(self._poses)


DEPTH_TOPIC = '/bartender/stand_camera/depth'
CAMERA_INFO_TOPIC = '/bartender/stand_camera/camera_info'
# Same pose and intrinsics as the depth sensor, so its pixels line up with depth.
RGB_TOPIC = '/bartender/stand_camera/rgb'

# ponytail: calibration knobs; a box round a bottle station, tall enough for the
# label, and how long an unreadable label waits before it is read again.
BOTTLE_ROI_R = 0.06
BOTTLE_ROI_H = 0.32
LABEL_RETRY_S = 30.0


class DepthCache(Node):
    """Latest stand-camera depth and CameraInfo, decoded once per arriving image."""

    def __init__(self):
        super().__init__('bartender_api_depth')
        self._image = None
        self._received = None
        self._info = None
        self._decoded = (None, None, None)   # (image, info, (depth, K))
        self._rgb = None
        self._rgb_received = None
        self._lock = threading.Lock()
        self.create_subscription(Image, DEPTH_TOPIC, self._on_image, FRESH)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_info, FRESH)
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, FRESH)

    def _on_image(self, msg):
        with self._lock:
            self._image = msg
            # Arrival time, not header.stamp: sim stamps are sim time, this node is wall time.
            self._received = time.monotonic()

    def _on_info(self, msg):
        with self._lock:
            self._info = msg

    def _on_rgb(self, msg):
        with self._lock:
            self._rgb = msg
            self._rgb_received = time.monotonic()

    def rgb(self):
        """Return (BGR ndarray, None) or (None, why there is none)."""
        with self._lock:
            image, received = self._rgb, self._rgb_received
        if image is None:
            return None, f'no colour image on {RGB_TOPIC}'
        if time.monotonic() - received > perception.MAX_FRAME_AGE_S:
            return None, 'colour image is stale'
        bgr = decode_colour(image.encoding, image.data, image.height,
                            image.width, image.step)
        if bgr is None:
            return None, f'unsupported colour encoding {image.encoding!r}'
        return bgr, None

    def frame(self):
        """Return (perception.Frame, None) or (None, why there is none)."""
        with self._lock:
            image, received, info = self._image, self._received, self._info
            cached_image, cached_info, decoded = self._decoded
        if image is None:
            return None, f'no depth image on {DEPTH_TOPIC}'
        if info is None:
            return None, f'no CameraInfo on {CAMERA_INFO_TOPIC}'
        if (info.width, info.height) != (image.width, image.height):
            return None, 'CameraInfo size does not match the depth image'
        if cached_image is not image or cached_info is not info:
            decoded = (perception.decode_depth(
                image.encoding, image.data, image.height, image.width,
                image.step, image.is_bigendian),
                perception.intrinsics(info.k, info.d))
            with self._lock:
                self._decoded = (image, info, decoded)
        depth, K = decoded
        if depth is None:
            return None, f'unsupported depth encoding {image.encoding!r}'
        if K is None:
            return None, 'depth is not rectified (non-zero distortion)'
        return perception.Frame(depth, K, info.header.frame_id,
                                time.monotonic() - received), None


def decode_colour(encoding, data, height, width, step):
    """Decode rgb8/bgr8 to a BGR array, else None."""
    if encoding not in ('rgb8', 'bgr8'):
        return None
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    bgr = rows[:, :width * 3].reshape(height, width, 3)
    return np.ascontiguousarray(bgr[..., ::-1] if encoding == 'rgb8' else bgr)


def make_label_watcher(depth_cache, calib, inventory):
    """Return see(station, occupied) -> label.Label | None, or None without OCR or Gemini.

    A label is read once per bottle: kept until the station is seen empty,
    and an unknown reading is retried every LABEL_RETRY_S.
    """
    read_text = ocr.make_reader()
    ask = vlm.make_asker(label.PROMPT, label.schema(inventory))
    deciders = (label.fuzzy, *filter(None, [jev.make_jev()]))
    if read_text is None and ask is None:
        return None
    known = {}   # station -> (read at, Label)
    # ponytail: one lock over the OCR and Gemini calls, so a first read holds
    # /world for a few seconds per bottle; read in a background thread if a poller minds.
    lock = threading.Lock()

    def look(station):
        bgr, why = depth_cache.rgb()
        if bgr is None:
            return label.unknown(why)
        frame, why = depth_cache.frame()
        if frame is None:
            return label.unknown(why)
        box, why = perception.station_box(
            frame.K, calib.T_world_optical, L.STATIONS[station], BOTTLE_ROI_R,
            (L.COUNTER_Z, L.COUNTER_Z + BOTTLE_ROI_H), bgr.shape[:2])
        if box is None:
            return label.unknown(why)
        u0, u1, v0, v1 = box
        return label.read_label(bgr[v0:v1, u0:u1], inventory, read_text, ask, deciders)

    def see(station, occupied):
        with lock:
            if occupied is False:
                known.pop(station, None)
                return None
            at, seen = known.get(station, (-float('inf'), None))
            retry = seen is None or (
                seen.type is None
                and time.monotonic() - at >= LABEL_RETRY_S)
            if occupied and retry:
                seen = look(station)
                known[station] = (time.monotonic(), seen)
            return seen
    return see


def make_observer(depth_cache, calib):
    def observe(station):
        frame, why_not = depth_cache.frame()
        if frame is None:
            return perception.unknown(why_not)
        return perception.observe(frame, calib, perception.PROFILES[station],
                                  L.STATIONS[station])
    return observe


def make_handler(pose_cache, teach_node, move_bridge, observe=None,
                 mode='ground_truth', see_label=None, ask_jev=None):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def _send(self, code, payload):
            raw = json.dumps(payload).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            # Every response here reflects live state; a cached copy would
            # be worse than no answer at all.
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def _read_body(self):
            """Return (body, None) or (None, error-payload) -- send nothing."""
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return None, {'error': 'bad request body'}
            # Valid JSON is not necessarily an object -- see teach_gui.py's
            # identical check and the AttributeError it replaced.
            if not isinstance(body, dict):
                return None, {'error': 'body must be a JSON object'}
            return body, None

        def do_GET(self):
            path = self.path.split('?')[0].rstrip('/') or '/'
            if path == '/world':
                self._send(200, world.build(pose_cache.get, observe, mode,
                                            see_label).to_json())
            elif path == '/state':
                self._send(200, state_view.build(teach_node))
            elif path == '/bottles':
                self._send(200, move_bridge.bottles())
            elif path == '/drinks':
                self._send(200, move_bridge.drinks())
            else:
                self._send(404, {'error': 'not found'})

        def do_POST(self):
            path = self.path.split('?')[0].rstrip('/')
            body, error = self._read_body()
            if error is not None:
                self._send(400, error)
                return
            if path == '/can':
                self._do_can(body)
            elif path == '/move/point':
                self._do_move('goto', body, ('point',))
            elif path == '/move/jog':
                self._do_move('jog', body, ('axis', 'amount'))
            elif path == '/gripper':
                self._do_move('gripper', body, ('position',))
            elif path == '/pick':
                result = move_bridge.pick(body.get('bottle'))
                self._send(200 if result['ok'] else 409, result)
            elif path == '/make':
                result = move_bridge.make(body.get('drink'))
                self._send(200 if result['ok'] else 409, result)
            elif path == '/ask':
                self._do_ask(body)
            else:
                self._send(404, {'error': 'not found'})

        def _do_can(self, body):
            verb = body.get('verb', '')
            if not isinstance(verb, str) or not verb:
                self._send(400, {'error': 'no verb'})
                return
            args = body.get('args', {})
            if not isinstance(args, dict):
                self._send(400, {'error': 'args must be a JSON object'})
                return
            self._send(200, feasibility.can(verb, args))

        def _do_ask(self, body):
            if ask_jev is None:
                self._send(503, {'error': 'set JEV_KEY to use /ask'})
                return
            drinks, not_ready, bottles = move_bridge.choices()
            result = intent.route(body.get('text'), drinks, bottles, ask_jev, not_ready)
            self._send(200 if result['ok'] else 422, result)

        def _do_move(self, kind, body, field_names):
            arm = body.get('arm', '')
            if not isinstance(arm, str) or not arm:
                self._send(400, {'error': 'no arm'})
                return
            fields = [body.get(name) for name in field_names]
            if any(f is None for f in fields):
                self._send(400, {'error': f'need {", ".join(field_names)}'})
                return
            result = getattr(move_bridge, kind)(arm, *fields)
            self._send(200 if result['ok'] else 409, result)

        def log_message(self, fmt, *args):
            """Silence per-request logging; a poller would bury real output."""

    return Handler


def _load_calibration(mode, path):
    """Return (Calibration or None, ok); ground_truth mode needs none."""
    if mode == 'ground_truth':
        return None, True
    path = path or os.path.join(get_package_share_directory('bartender_api'),
                                'config', 'stand_camera.yaml')
    try:
        return perception.load_calibration(path), True
    except (OSError, KeyError, ValueError) as exc:
        print(f'cannot load camera calibration {path}: {exc}',
              file=sys.stderr)
        return None, False


def _attach_observer(executor, calib, inventory):
    if calib is None:
        return None, None
    depth_cache = DepthCache()
    executor.add_node(depth_cache)
    return (make_observer(depth_cache, calib),
            make_label_watcher(depth_cache, calib, inventory))


def _wait_for_ground_truth(pose_cache, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not pose_cache.has_any() and time.monotonic() < deadline:
        time.sleep(0.1)
    return pose_cache.has_any()


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='bartender_api', description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1',
                        help='interface to bind (default 127.0.0.1; '
                             '0.0.0.0 exposes this to the network)')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--perception', choices=world.MODES,
                        default='ground_truth',
                        help='where /world bottle poses come from '
                             '(compare is sim only)')
    parser.add_argument('--camera-config', default=None,
                        help='stand camera calibration YAML (default: the '
                             'sim one in this package\'s config/)')
    parser.add_argument('--bottles', default=None,
                        help='bottle inventory YAML that label readings are '
                             'matched against (default: config/bottles.yaml '
                             'in this package)')
    parser.add_argument('--points', default=None,
                        help='point file: a path, or a bare name like '
                             '`workcell` for bartender_teach/config/'
                             'workcell_points.yaml (default: the bar\'s '
                             'taught_points.yaml)')
    parser.add_argument('--menu', default=None,
                        help='drinks menu: a path, or a bare name like '
                             '`workcell` for bartender_teach/config/'
                             'workcell_menu.yaml (default: the --points '
                             'name, when that is a bare name)')
    # ros2 run passes --ros-args through; argparse must not choke on it.
    opts, _ = parser.parse_known_args(sys.argv[1:] if args is None else args)

    calib, ok = _load_calibration(opts.perception, opts.camera_config)
    if not ok:
        return 1
    bottles_path = opts.bottles or os.path.join(
        get_package_share_directory('bartender_api'), 'config', 'bottles.yaml')
    try:
        inventory = label.load_inventory(bottles_path)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f'cannot load bottle inventory {bottles_path}: {exc}',
              file=sys.stderr)
        return 1

    rclpy.init(args=None)
    pose_cache = PoseCache()
    teach_node = TeachNode(cache_pose=True)
    executor = MultiThreadedExecutor()
    executor.add_node(pose_cache)
    executor.add_node(teach_node)
    observe, see_label = _attach_observer(executor, calib, inventory)
    threading.Thread(target=executor.spin, daemon=True).start()

    if opts.perception == 'compare' and not _wait_for_ground_truth(pose_cache):
        print(f'--perception compare needs sim ground truth, and there '
              f'is nothing on {POSE_TOPIC} after 10s', file=sys.stderr)
        executor.shutdown()
        rclpy.try_shutdown()
        return 1

    if not teach_node.wait_for_state(timeout=10.0):
        print('warning: no /joint_states after 10s -- serving anyway, '
              '/state will show both arms disconnected', file=sys.stderr)

    # Same fallback as pour_action_server.py: a point file that is missing
    # or unreadable must not stop the server from starting -- goto just has
    # no points to offer until it is fixed, which /move/point's own refusal
    # already reports per request.
    points_path = points_path_for(opts.points)
    try:
        store = PointStore.load(points_path)
    except PointStoreError as exc:
        print(f'warning: {exc} -- starting with no taught points',
              file=sys.stderr)
        store = PointStore(points_path)
    menu_arg = opts.menu
    if menu_arg is None and opts.points and os.sep not in opts.points \
            and not opts.points.endswith(('.yaml', '.yml')):
        menu_arg = opts.points
    menu_path = menu.menu_path_for(menu_arg)
    move_bridge = movement.MovementBridge(teach_node, store, menu_path)
    ask_jev = jev.make_asker()

    try:
        server = ThreadingHTTPServer(
            (opts.host, opts.port),
            make_handler(pose_cache, teach_node, move_bridge, observe,
                         opts.perception, see_label, ask_jev))
    except OSError as exc:
        print(f'cannot bind {opts.host}:{opts.port}: {exc}', file=sys.stderr)
        executor.shutdown()
        pose_cache.destroy_node()
        teach_node.destroy_node()
        rclpy.try_shutdown()
        return 1

    shown = '127.0.0.1' if opts.host in ('0.0.0.0', '') else opts.host
    print(f'\n  bartender_api:  http://{shown}:{opts.port}')
    print(f'    GET  /world      (bottle poses: {opts.perception})')
    print('    GET  /state')
    print('    POST /can        {"verb": "pour"|"open", "args": {...}}')
    print('    POST /move/point {"arm": "a", "point": "whiskey_approach"}')
    print('    POST /move/jog   {"arm": "a", "axis": "z", "amount": 20}')
    print('    POST /gripper    {"arm": "b", "position": 0.25}')
    print('    GET  /bottles')
    print('    POST /pick       {"bottle": "whiskey"}')
    print('    GET  /drinks')
    print('    POST /make       {"drink": "whiskey_cola"}')
    print('    POST /ask        {"text": "a whiskey coke please"}'
          + ('' if ask_jev else '  -- off: set JEV_KEY'))
    print(f'  points: {points_path}')
    print(f'  menu:   {menu_path or "none (--menu)"}')
    if opts.host not in ('127.0.0.1', 'localhost'):
        print(f'\n  WARNING: bound to {opts.host}. This process moves the '
              f'robot arms (/move/*, /gripper, /pick, /make).\n           Only do this on '
              f'a trusted, isolated robot LAN.')
    print('\n  Ctrl-C to stop.\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopping')
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        pose_cache.destroy_node()
        teach_node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
