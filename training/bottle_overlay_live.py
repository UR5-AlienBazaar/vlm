#!/usr/bin/env python3
"""Serve a live annotated overhead view for the current bottle stations.

This is a camera-layout bootstrap: regions are seeded from a reviewed frame,
then serve the names visible to the operator while RGB crops are collected for
the learned detector. Ballantine's has its own named label.
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from bottle_vision.camera import mjpeg_frames

# Reviewed 1280x720 cam0_upside frame, 2026-09-27. Re-shoot with --box if the
# table layout changes. `label:x0,y0,x1,y1`; boxes contain the whole bottle.
DEFAULT_BOXES = (
    "cola:1070,225,1250,320",
    "7up:690,520,800,655",
    "mirinda:345,495,475,595",
    "ballantines:1020,15,1200,155",
)


def parse_box(value):
    try:
        label, coordinates = value.split(":", 1)
        return label, tuple(map(int, coordinates.split(",")))
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError("--box must be LABEL:X0,Y0,X1,Y1") from error


class Pipeline:
    def __init__(self, source, boxes):
        self.source, self.boxes, self.lock, self.jpeg = source, boxes, threading.Lock(), None
        self.objects = []
        self.frame_count, self.updated_at = 0, None
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        for frame in mjpeg_frames(self.source):
            output, objects = frame.copy(), []
            for label, (x0, y0, x1, y1) in self.boxes:
                colour = {
                    "cola": (0, 0, 255),       # red (BGR)
                    "7up": (0, 220, 0),        # green
                    "mirinda": (0, 140, 255),  # orange
                    "ballantines": (180, 0, 180),  # purple
                }[label]
                shown = "BALLANTINE'S" if label == "ballantines" else label.upper()
                cv2.rectangle(output, (x0, y0), (x1, y1), colour, 3)
                cv2.putText(output, shown, (x0, max(28, y0 - 9)), cv2.FONT_HERSHEY_SIMPLEX, .72, colour, 2)
                objects.append({"label": label, "distractor": False,
                                "bbox": [x0, y0, x1, y1], "confidence": 1.0, "source": "reviewed_layout"})
            cv2.putText(output, "REVIEWED RGB BOTTLE REGIONS", (25, output.shape[0] - 25), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
            ok, encoded = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 82])
            with self.lock:
                if ok: self.jpeg = encoded.tobytes()
                self.objects = objects
                self.frame_count += 1
                self.updated_at = time.time()

    def state(self):
        with self.lock: return list(self.objects), self.frame_count, self.updated_at
    def image(self):
        with self.lock: return self.jpeg


def handler(pipeline):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_GET(self):
            if self.path == "/objects":
                objects, frame_count, updated_at = pipeline.state()
                body = json.dumps({"objects": objects, "frame_count": frame_count, "updated_at": updated_at}).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
            if self.path not in ("/", "/stream"):
                self.send_error(404); return
            if self.path == "/":
                body = b'<img src="/stream" style="max-width:100%">'
                self.send_response(200); self.send_header("Content-Type", "text/html"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.send_header("Cache-Control", "no-store, no-cache, must-revalidate"); self.send_header("Pragma", "no-cache"); self.end_headers()
            try:
                while True:
                    image = pipeline.image()
                    if image: self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError): pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://localhost:8765/cam0_upside")
    parser.add_argument("--box", action="append", type=parse_box, default=[])
    parser.add_argument("--port", type=int, default=8768)
    args = parser.parse_args()
    boxes = args.box or [parse_box(value) for value in DEFAULT_BOXES]
    ThreadingHTTPServer(("127.0.0.1", args.port), handler(Pipeline(args.source, boxes))).serve_forever()


if __name__ == "__main__": main()
