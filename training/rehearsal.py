#!/usr/bin/env python3
"""Rehearse the demo without the robot: time camera -> detector (Brev) -> bartender_api (mock arm).

Every latency is measured on this machine's clock, so Brev's clock never enters one.

    python -m training.rehearsal --camera http://10.42.0.200:8765/cam0_upside \\
        --detector http://localhost:8772 --api http://localhost:8096

--replay DIR serves recorded JPEGs as the camera on --replay-port instead. Forward that
port to Brev (ssh -R) and point a detector at it. The run then also times appearance:
the replay shows black until /objects is empty, then a recorded frame, and waits for
/objects to report it. That round trip covers both tunnel hops and the detector's own work.

/pick moves the arm behind --api, so it runs only when --pick names a bottle.
"""
import argparse
import json
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from bottle_vision.camera import _read_jpegs

POLL_S = 0.02


def summary(seconds):
    s = sorted(seconds)
    if not s:
        return "no samples"
    ms = [s[0], s[len(s) // 2], s[round(.95 * (len(s) - 1))], s[-1]]
    return f"n={len(s):<3} min {ms[0] * 1e3:7.1f}  p50 {ms[1] * 1e3:7.1f}  p95 {ms[2] * 1e3:7.1f}  max {ms[3] * 1e3:7.1f} ms"


def request(url, body=None, timeout=120):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data, {"Content-Type": "application/json"} if data else {})
    start = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            status, text = response.status, response.read().decode(errors="replace")
    except urllib.error.HTTPError as error:
        status, text = error.code, error.read().decode(errors="replace")
    except OSError as error:
        status, text = type(error).__name__, str(error)
    return time.perf_counter() - start, status, text


def timed_gets(url, rounds):
    runs = [request(url, timeout=30) for _ in range(rounds)]
    return [seconds for seconds, _, _ in runs], {status for _, status, _ in runs}, runs[-1][2]


def stream_rate(url, seconds):
    """Time to first JPEG and JPEGs per second on an MJPEG URL."""
    stamps, start = [], time.perf_counter()

    def read():
        for _ in _read_jpegs(url, timeout=5):
            stamps.append(time.perf_counter())
            if stamps[-1] - start > seconds:
                return

    thread = threading.Thread(target=read, daemon=True)
    thread.start(); thread.join(seconds + 10)
    if len(stamps) < 2:
        return f"{len(stamps)} frames in {seconds}s"
    fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    return f"first frame {(stamps[0] - start) * 1e3:.0f} ms, {fps:.1f} fps over {stamps[-1] - stamps[0]:.1f}s"


class Replay:
    """Recorded JPEGs looped as an MJPEG camera, or a black frame while `blank` is set."""

    def __init__(self, folder, fps):
        paths = sorted(p for p in Path(folder).iterdir() if p.suffix.lower() in (".jpg", ".jpeg"))
        if not paths:
            raise SystemExit(f"no .jpg frames in {folder}")
        self.frames, self.fps, self.blank = [p.read_bytes() for p in paths], fps, False
        shape = cv2.imdecode(np.frombuffer(self.frames[0], np.uint8), cv2.IMREAD_COLOR).shape
        self.black = cv2.imencode(".jpg", np.zeros(shape, np.uint8))[1].tobytes()

    def handler(self):
        replay = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_): pass

            def do_GET(self):
                self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
                i = 0
                try:
                    # A real camera resends every frame, and trackers need consecutive frames to confirm a track.
                    while True:
                        jpeg = replay.black if replay.blank else replay.frames[i % len(replay.frames)]
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                        i += 1; time.sleep(1 / replay.fps)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
        return Handler


def seen(objects_url):
    return any(row.get("found", True) for row in json.loads(request(objects_url, timeout=5)[2])["objects"])


def wait_for(objects_url, want, timeout):
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if seen(objects_url) == want:
            return time.perf_counter()
        time.sleep(POLL_S)
    return None


def appearance(replay, objects_url, trials, timeout):
    """Seconds from switching black -> recorded frame until the detector reports an object."""
    samples = []
    for _ in range(trials):
        replay.blank = True
        if wait_for(objects_url, False, timeout) is None:
            print("  detector still reports objects on a black frame; skipping trial")
            continue
        replay.blank, start = False, time.perf_counter()
        found = wait_for(objects_url, True, timeout)
        if found is None:
            print(f"  nothing reported within {timeout}s of showing the frame")
        else:
            samples.append(found - start)
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", help="MJPEG URL of the camera (the Pi bridge)")
    parser.add_argument("--replay", type=Path, help="folder of recorded .jpg frames to serve as the camera")
    parser.add_argument("--replay-port", type=int, default=8775)
    parser.add_argument("--fps", type=float, default=15)
    parser.add_argument("--detector", help="detector base URL as seen from here, e.g. the -L end of the Brev tunnel")
    parser.add_argument("--api", help="bartender_api base URL")
    parser.add_argument("--pick", help="bottle to POST /pick once (moves the arm behind --api)")
    parser.add_argument("--ask", help="text to POST /ask once (sent to OpenRouter by the API)")
    parser.add_argument("--rounds", type=int, default=20)
    parser.add_argument("--trials", type=int, default=5, help="appearance trials with --replay")
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()

    replay = None
    if args.replay:
        replay = Replay(args.replay, args.fps)
        server = ThreadingHTTPServer(("127.0.0.1", args.replay_port), replay.handler())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        args.camera = args.camera or f"http://127.0.0.1:{args.replay_port}/cam0_upside"
        print(f"replaying {len(replay.frames)} frames at http://127.0.0.1:{args.replay_port}/cam0_upside")

    rows = []
    if args.camera:
        rows.append(("camera stream", stream_rate(args.camera, 5)))
    if args.detector:
        base = args.detector.rstrip("/")
        seconds, statuses, body = timed_gets(base + "/objects", args.rounds)
        rows.append((f"detector GET /objects {sorted(statuses)}", summary(seconds)))
        rows.append(("detector annotated stream", stream_rate(base + "/stream", 5)))
        rows.append(("  last /objects", body[:160]))
        if replay:
            rows.append(("detector appearance (black -> seen)", summary(appearance(replay, base + "/objects", args.trials, args.timeout))))
    if args.api:
        base = args.api.rstrip("/")
        for route in ("/state", "/bottles", "/drinks", "/world"):
            seconds, statuses, _ = timed_gets(base + route, args.rounds if route != "/world" else min(args.rounds, 5))
            rows.append((f"api GET {route} {sorted(statuses)}", summary(seconds)))
        for route, body in (("/ask", args.ask and {"text": args.ask}), ("/pick", args.pick and {"bottle": args.pick})):
            if body:
                seconds, status, text = request(base + route, body, timeout=300)
                rows.append((f"api POST {route} [{status}]", f"{seconds * 1e3:.0f} ms  {text[:160]}"))

    width = max(len(name) for name, _ in rows) if rows else 0
    for name, value in rows:
        print(f"{name:<{width}}  {value}")


if __name__ == "__main__":
    main()
