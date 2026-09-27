#!/usr/bin/env python3
"""Re-serve the Pi's MJPEG feed smaller, so it fits through the uplink to Brev.

Measured on the 127 labelled cam0 frames: 960 px at quality 70 detects as many
bottles as the original 1280 px frames at a third of the bytes (61 vs 195 KB).

    python training/mjpeg_relay.py --source http://10.42.0.200:8765/cam0_upside --port 8766
    ssh -N -R 8768:localhost:8766 training-center-point
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bottle_vision.camera import mjpeg_frames


def shrink(frame, width: int, quality: int) -> bytes:
    if frame.shape[1] > width:
        frame = cv2.resize(frame, (width, round(frame.shape[0] * width / frame.shape[1])), interpolation=cv2.INTER_AREA)
    return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="http://10.42.0.200:8765/cam0_upside")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--quality", type=int, default=70)
    args = parser.parse_args()
    latest, fresh = [None], threading.Condition()

    def pump():
        for frame in mjpeg_frames(args.source):
            jpeg = shrink(frame, args.width, args.quality)
            with fresh: latest[0] = jpeg; fresh.notify_all()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def do_GET(self):
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            try:
                while True:
                    with fresh: fresh.wait(); jpeg = latest[0]
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    threading.Thread(target=pump, daemon=True).start()
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    import numpy as np
    assert shrink(np.zeros((720, 1280, 3), np.uint8), 960, 70)[:2] == b"\xff\xd8"
    assert cv2.imdecode(np.frombuffer(shrink(np.zeros((720, 1280, 3), np.uint8), 960, 70), np.uint8), 1).shape == (540, 960, 3)
    main()
