"""MJPEG + /objects HTTP handler shared by the live bottle streams."""
import json
import time
from http.server import BaseHTTPRequestHandler


def make_handler(pipeline):
    """`pipeline` provides image() -> JPEG bytes | None and state() -> list of objects."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def do_GET(self):
            if self.path == "/objects":
                body = json.dumps({"objects": pipeline.state(), "updated_at": time.time()}).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
                return
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache"); self.end_headers()
            try:
                while True:
                    image = pipeline.image()
                    if image:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler
