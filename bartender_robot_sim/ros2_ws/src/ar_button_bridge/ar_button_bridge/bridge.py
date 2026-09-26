"""Bridge from the Spectacles AR buttons to a ROS 2 topic.

    ros2 run ar_button_bridge ar_button_bridge

The glasses cannot be a ROS node, so the Lens sends a tiny HTTP request and
this node republishes it:

    POST /button   {"button": 1}     ->  std_msgs/Int32 on /ar/button
    GET  /health                     ->  {"ok": true}

Only the buttons 1, 2 and 3 are accepted; anything else is refused and
nothing is published. A button that repeats inside --min-interval seconds is
ignored, so a jittery pinch cannot send the same command twice.

There is no encryption. Anyone who can reach the port can publish a button,
so run it on a network you trust and set --token (the Lens sends it in the
X-Token header) if the network is shared.

The request handling is kept apart from ROS (see ButtonGate) so it can be
tested without a ROS installation: python3 -m unittest discover -s test
"""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time

TOPIC = '/ar/button'
VALID_BUTTONS = (1, 2, 3)
MAX_BODY_BYTES = 1024


class ButtonGate:
    """Decides whether a request becomes a publish, and says why not if not.

    `publish` is called with the button number (an int) for accepted requests.
    """

    def __init__(self, publish, token=None, min_interval=0.5,
                 clock=time.monotonic):
        self._publish = publish
        self._token = token
        self._min_interval = min_interval
        self._clock = clock
        self._last = {}
        self._lock = threading.Lock()

    def handle(self, body, token_header):
        """Return (http_status, json_dict) for a POST /button."""
        if self._token and not hmac.compare_digest(
                (token_header or '').encode(), self._token.encode()):
            return 401, {'ok': False, 'error': 'bad or missing token'}
        if len(body) > MAX_BODY_BYTES:
            return 413, {'ok': False, 'error': 'body too large'}
        try:
            data = json.loads(body.decode('utf-8'))
        except (UnicodeDecodeError, ValueError):
            return 400, {'ok': False, 'error': 'body is not valid JSON'}
        button = data.get('button') if isinstance(data, dict) else None
        # bool is an int in Python; true must not count as button 1.
        if isinstance(button, bool) or button not in VALID_BUTTONS:
            return 400, {'ok': False,
                         'error': 'button must be one of %s' % (VALID_BUTTONS,)}

        with self._lock:
            now = self._clock()
            last = self._last.get(button)
            if last is not None and now - last < self._min_interval:
                return 429, {'ok': False, 'error': 'too fast, ignored'}
            self._last[button] = now
        self._publish(button)
        return 200, {'ok': True, 'button': button}


def make_handler(gate):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, status, payload):
            out = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(out)))
            self.end_headers()
            self.wfile.write(out)

        def do_GET(self):
            if self.path == '/health':
                self._send(200, {'ok': True})
            else:
                self._send(404, {'ok': False, 'error': 'not found'})

        def do_POST(self):
            if self.path != '/button':
                self._send(404, {'ok': False, 'error': 'not found'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                length = -1
            if length < 0 or length > MAX_BODY_BYTES:
                self._send(413, {'ok': False, 'error': 'body too large'})
                return
            body = self.rfile.read(length)
            status, payload = gate.handle(body, self.headers.get('X-Token'))
            self._send(status, payload)

        def log_message(self, fmt, *args):
            print('[ar_button_bridge] %s %s' % (self.address_string(), fmt % args))

    return Handler


def serve(gate, host, port):
    """Start the HTTP server on a background thread and return it."""
    server = ThreadingHTTPServer((host, port), make_handler(gate))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='0.0.0.0',
                        help='address to listen on (default: all interfaces, '
                             'so the glasses can reach it)')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--token', default=None,
                        help='require this value in the X-Token header')
    parser.add_argument('--min-interval', type=float, default=0.5,
                        help='ignore a repeat of the same button within this '
                             'many seconds')
    parser.add_argument('--topic', default=TOPIC)
    parser.add_argument('--dry-run', action='store_true',
                        help='do not use ROS: just print each accepted button '
                             '(for testing the glasses on a computer without ROS)')
    opts, ros_args = parser.parse_known_args(args)

    if opts.dry_run:
        gate = ButtonGate(lambda b: print('[ar_button_bridge] button %d (dry run, '
                                          'not published)' % b),
                          token=opts.token, min_interval=opts.min_interval)
        server = serve(gate, opts.host, opts.port)
        print('[ar_button_bridge] dry run: listening on http://%s:%d, no ROS'
              % (opts.host, opts.port))
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            server.shutdown()
        return

    # ROS is imported here so ButtonGate can be tested without it.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32

    rclpy.init(args=ros_args)
    node = Node('ar_button_bridge')
    pub = node.create_publisher(Int32, opts.topic, 10)

    def publish(button):
        msg = Int32()
        msg.data = button
        pub.publish(msg)
        node.get_logger().info('button %d -> %s' % (button, opts.topic))

    gate = ButtonGate(publish, token=opts.token, min_interval=opts.min_interval)
    server = serve(gate, opts.host, opts.port)
    node.get_logger().info(
        'listening on http://%s:%d, publishing Int32 on %s'
        % (opts.host, opts.port, opts.topic))
    if opts.host == '0.0.0.0' and not opts.token:
        node.get_logger().warn(
            'listening on every interface with no --token: anyone on this '
            'network can publish a button')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
