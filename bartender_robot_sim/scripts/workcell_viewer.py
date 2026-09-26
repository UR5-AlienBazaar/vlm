#!/usr/bin/env python3
"""Live MJPEG view of the workcell twin's cameras (run inside the sim container).

    python3 scripts/workcell_viewer.py --port 8090    # http://localhost:8090

`view` is the fixed overview camera in workcell_world.sdf; `vlm` is the camera
capture_workcell_scenes.py moves around, and shows nothing until it runs.
"""
import argparse
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

CAMS = {'view': '/workcell/view_camera/image_raw', 'vlm': '/vlm_cam/rgb'}
jpeg = {}
changed = threading.Condition()


def on_image(name):
    def cb(msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        _, buf = cv2.imencode('.jpg', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        with changed:
            jpeg[name] = buf.tobytes()
            changed.notify_all()
    return cb


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        name = self.path.strip('/')
        if name not in CAMS:
            page = ''.join(f'<figure style="display:inline-block"><img src="/{c}" height="400">'
                           f'<figcaption>{c}</figcaption></figure>' for c in CAMS)
            body = ('<html><title>Workcell twin</title>'
                    f'<body style="background:#222;color:#ddd;font-family:sans-serif">{page}</body></html>')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(body.encode())
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with changed:
                    changed.wait(timeout=5)
                    frame = jpeg.get(name)
                if frame:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', type=int, default=8090)
    opts = parser.parse_args()
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                               *(f'{t}@sensor_msgs/msg/Image[ignition.msgs.Image' for t in CAMS.values())],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('workcell_viewer')
    for c, t in CAMS.items():
        node.create_subscription(Image, t, on_image(c), 2)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'http://localhost:{opts.port}', flush=True)
    try:
        ThreadingHTTPServer(('0.0.0.0', opts.port), Handler).serve_forever()
    finally:
        bridge.terminate()


if __name__ == '__main__':
    main()
