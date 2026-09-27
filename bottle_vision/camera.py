"""Open a webcam, video, or the robot's multipart MJPEG feed."""
import time
import urllib.request

import numpy as np


def mjpeg_frames(url: str, timeout: float = 5.0):
    """Yield newest JPEG frames; reconnect after a camera/tunnel interruption."""
    import cv2
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
                    if start < 0 or end < 0:
                        continue
                    jpeg, data = data[start:end + 2], data[end + 2:]
                    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if image is not None:
                        yield image
        except (OSError, ConnectionError):
            time.sleep(1)


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
