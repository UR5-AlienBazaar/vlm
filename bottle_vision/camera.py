"""Open a webcam, video, or the robot's multipart MJPEG feed."""
import threading
import time
import urllib.request

import numpy as np


def _read_jpegs(url: str, timeout: float):
    while True:
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                data = b""
                while True:
                    chunk = response.read(65536)
                    if not chunk:
                        raise ConnectionError("MJPEG stream closed")
                    data += chunk
                    end = data.rfind(b"\xff\xd9")
                    start = data.rfind(b"\xff\xd8", 0, end)
                    if start >= 0 and end >= 0:
                        jpeg, data = data[start:end + 2], data[end + 2:]
                        yield jpeg
        except (OSError, ConnectionError):
            time.sleep(1)


def mjpeg_frames(url: str, timeout: float = 5.0):
    """Yield newest JPEG frames; reconnect after a camera/tunnel interruption."""
    import cv2
    # A consumer slower than the camera would otherwise read an ever-growing
    # backlog from the socket, so a thread drains it and older frames are dropped.
    latest, fresh = [None], threading.Event()

    def drain():
        for jpeg in _read_jpegs(url, timeout):
            latest[0] = jpeg; fresh.set()

    threading.Thread(target=drain, daemon=True).start()
    while True:
        fresh.wait(); fresh.clear()
        image = cv2.imdecode(np.frombuffer(latest[0], np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            yield image


def frames(source: str):
    """Yield BGR frames from an HTTP MJPEG source, camera index, or video path."""
    if source.startswith(("http://", "https://")):
        yield from mjpeg_frames(source)
        return
    import cv2
    capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open camera/video {source!r}")
    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            yield image
    finally:
        capture.release()
