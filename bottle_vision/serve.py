"""MJPEG + /objects HTTP handler shared by the live bottle streams."""
import json
import time
from http.server import BaseHTTPRequestHandler


def make_handler(pipeline):
    """`pipeline` provides image() -> JPEG bytes | None, state() -> list of objects, and optionally position() -> dict."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def do_GET(self):
            if self.path in ("/objects", "/position"):
                data = {"objects": pipeline.state(), "updated_at": time.time()} if self.path == "/objects" else pipeline.position()
                body = json.dumps(data).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
                return
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache"); self.end_headers()
            sent = None
            try:
                while True:
                    image = pipeline.image()
                    # Resending unchanged frames saturates the SSH tunnel and
                    # the viewer then lags ever further behind real time.
                    if image and image is not sent:
                        sent = image
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler
