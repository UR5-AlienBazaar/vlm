"""Browser front end for the teach pendant.

    ros2 run bartender_teach teach_gui

Then open http://127.0.0.1:8080. Same tool as `teach`, with buttons instead of
typing `jog z -50`.

Every control goes through Pendant.dispatch(), the same entry point the
terminal pendant uses, and the page shows whatever that prints. That is the
whole design: the GUI owns no robot logic at all, so the two front ends cannot
drift apart, a fix to a jog bound or a refusal message reaches both at once,
and the tests over Pendant cover this as well.

Why a web page and not a desktop toolkit: no extra dependencies (http.server
is stdlib, and python3-tk is not installed on every ROS box), it works from a
laptop on the same network as the robot, and it degrades to the terminal
pendant if anything about it misbehaves.

Binding
-------
127.0.0.1 by default, and that default is a safety decision, not laziness:
this page moves a robot arm and has no authentication whatsoever. --host
0.0.0.0 exposes it to the network, which is reasonable on an isolated robot
LAN and a bad idea anywhere else. It prints a warning when you do.
"""
import argparse
import contextlib
import io
import json
import math
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import MultiThreadedExecutor

from bartender_teach.point_store import (
    PointStore, PointStoreError, points_path_for,
)
from bartender_teach.teach_points import ARMS, Pendant, TeachNode
from bartender_teach.tool_frames import TOOLS, tcp_from_tool0
from bartender_teach.web_assets import PAGE


class Bridge:
    """Runs pendant commands on behalf of the page, one at a time."""

    def __init__(self, node, store):
        self.node = node
        self.store = store
        self.pendant = Pendant(node, store)
        self._lock = threading.Lock()
        self.busy = False

    def run(self, line):
        """Dispatch one command line, returning (accepted, printed output).

        Refuses rather than queues when something is already running. The
        buttons are disabled while busy, but a double-click, a stale tab or a
        second browser can still race, and two motion goals interleaved on one
        arm is exactly the failure this tool exists to avoid.
        """
        if not self._lock.acquire(blocking=False):
            return False, '  busy: a command is already running'
        self.busy = True
        try:
            buf = io.StringIO()
            # Pendant prints; capture that verbatim so the page shows exactly
            # what the terminal would. Safe to redirect process-wide here
            # because the lock serialises commands and the terminal pendant is
            # not running in this process.
            with contextlib.redirect_stdout(buf):
                self.pendant.dispatch(line)
            return True, buf.getvalue()
        except Exception as exc:                            # noqa: BLE001
            # dispatch() already swallows the expected failures; anything
            # reaching here is a bug, and it must not kill the server and
            # strand whoever is standing at the robot.
            self.node.get_logger().error(f'command {line!r} raised: {exc}')
            return True, f'  unexpected error: {type(exc).__name__}: {exc}'
        finally:
            self.busy = False
            self._lock.release()

    def state(self):
        """Everything the page renders. Never blocks on the command lock."""
        joints = self.node.joints()
        selected = self.pendant.arm
        # Keep the pose poller on whichever arm the page is showing, or the
        # flange position would be the other arm's.
        self.node.poll_arm = selected
        arm = [{'name': n,
                # "shoulder_pan_joint" is too wide for the panel; the joint
                # order is fixed, so the suffix is unambiguous. The `b_`
                # prefix goes too -- the panel says which arm once, at the
                # top, rather than six times.
                'short': n.replace('_joint', '').replace('b_', '', 1),
                'rad': joints[n],
                'deg': math.degrees(joints[n])}
               for n in selected.joints if n in joints]
        pose = self.node.cached_pose(selected)
        tool = self.pendant.tool
        tip = None if pose is None else tcp_from_tool0(pose[0], pose[1], tool)
        return {
            'connected': bool(arm),
            'busy': self.busy,
            'file': self.store.path,
            'safety': self.pendant.safety,
            'joints': arm,
            'pose': None if pose is None else {
                'xyz': list(pose[0]), 'quat_xyzw': list(pose[1])},
            'tool': tool.name,
            'tools': [{'name': n, 'reach': TOOLS[n].reach,
                       'note': TOOLS[n].note} for n in sorted(TOOLS)],
            # The tip is what the operator is actually aiming, so it is sent
            # even when it equals the flange -- the page then never has to
            # know which case it is in.
            'tip': None if tip is None else list(tip[0]),
            'gripper': joints.get(selected.gripper_joint),
            # The real robot's mode, program and speed; 'real' is False in
            # the simulation and the page hides the panel's status then.
            'robot': self.node.robot.status(),
            'arm': selected.key,
            'arm_label': selected.label,
            'frame': selected.frame,
            'arms': [{'key': a.key, 'label': a.label} for a in ARMS.values()],
            # Every point, both arms, each tagged with the arm it drives. The
            # page needs the tag because `goto` goes to the point's own arm
            # whatever is selected, and a list that did not say which would
            # make that look like a bug.
            'points': [{'name': n,
                        'note': self.store.get(n).note,
                        'arm': self._arm_key(n)}
                       for n in self.store.names()],
            # The pipeline being recorded, or None. The page needs this to
            # show that saving a point is currently doing a second thing as
            # well -- a record mode you cannot see is a record mode you
            # forget you left on.
            'recording': (None if self.pendant.recording is None
                          else self.pendant.recording.name),
            'pipelines': [
                {'name': n,
                 'note': self.store.pipelines[n].note,
                 'steps': [st.describe()
                           for st in self.store.pipelines[n].steps],
                 # Rendered rather than the raw fields, because the page
                 # should not be a second place that decides how a step
                 # reads. describe() is what the terminal prints too.
                 'missing': self.store.pipelines[n].missing_points(self.store)}
                for n in sorted(self.store.pipelines)],
        }

    def _arm_key(self, name):
        group = self.store.group_of(self.store.get(name))
        for a in ARMS.values():
            if a.group == group:
                return a.key
        return '?'


def make_handler(bridge):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def _send(self, code, body, ctype):
            raw = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(raw)))
            # This page reflects live robot state; a cached copy showing an
            # arm that has since moved would be worse than no page.
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            path = self.path.split('?')[0].rstrip('/') or '/'
            if path == '/':
                self._send(200, PAGE, 'text/html; charset=utf-8')
            elif path == '/api/state':
                self._send(200, json.dumps(bridge.state()),
                           'application/json')
            else:
                self._send(404, '{"error":"not found"}', 'application/json')

        def do_POST(self):
            if self.path.split('?')[0].rstrip('/') != '/api/command':
                self._send(404, '{"error":"not found"}', 'application/json')
                return
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                self._send(400, '{"error":"bad request body"}',
                           'application/json')
                return
            # Valid JSON is not necessarily an object: a bare list used to
            # reach .get() and take the connection down with an AttributeError
            # instead of answering 400.
            if not isinstance(body, dict):
                self._send(400, '{"error":"body must be a JSON object"}',
                           'application/json')
                return
            cmd = body.get('cmd', '')
            if not isinstance(cmd, str) or not cmd.strip():
                self._send(400, '{"error":"no command"}', 'application/json')
                return
            accepted, output = bridge.run(cmd)
            self._send(200, json.dumps({'accepted': accepted,
                                        'output': output}),
                       'application/json')

        def log_message(self, fmt, *args):
            """Silence per-request logging.

            The page polls twice a second, which would bury every message the
            pendant actually wants to show in this terminal.
            """

    return Handler


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='teach_gui', description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1',
                        help='interface to bind (default 127.0.0.1; '
                             '0.0.0.0 exposes the robot to the network)')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--file', default=None,
                        help='point file, or a bare name: `workcell` is '
                             'config/workcell_points.yaml (default: the '
                             'shipped taught_points.yaml)')
    # ros2 run passes --ros-args through; argparse must not choke on it.
    opts, _ = parser.parse_known_args(sys.argv[1:] if args is None else args)

    path = points_path_for(opts.file)
    try:
        store = PointStore.load(path)
    except PointStoreError as exc:
        # Refuse to start rather than start empty: an empty store whose first
        # save rewrites the file wholesale would destroy whatever is in there.
        print(f'cannot read the point file:\n  {exc}', file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node = TeachNode(cache_pose=True)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    if not node.wait_for_state(timeout=10.0):
        print('warning: no /joint_states after 10s -- serving anyway, the '
              'page will show it as disconnected', file=sys.stderr)

    bridge = Bridge(node, store)
    try:
        server = ThreadingHTTPServer((opts.host, opts.port),
                                     make_handler(bridge))
    except OSError as exc:
        print(f'cannot bind {opts.host}:{opts.port}: {exc}', file=sys.stderr)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    shown = '127.0.0.1' if opts.host in ('0.0.0.0', '') else opts.host
    print(f'\n  teach pendant:  http://{shown}:{opts.port}')
    print(f'  points file:    {store.path}  ({len(store)} point(s))')
    if opts.host not in ('127.0.0.1', 'localhost'):
        print(f'\n  WARNING: bound to {opts.host}. This page moves the robot '
              f'and has no\n           authentication. Only do this on a '
              f'trusted network.')
    print('\n  Ctrl-C to stop.\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopping')
    finally:
        bridge.pendant.release()
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
