#!/usr/bin/env python3
"""Detect circles in the live camera feed with OpenCV's Hough transform.

Run from the repo root: python -m training.circle_detect_live
Keys: q quits. The slider tunes the accumulator threshold live.

First run (or --recalibrate): drag the search rectangle, then type the table
X Y in mm of its four corners. The rectangle and corner mm are saved to
--calib and circle centres are then reported in mm.
"""
import argparse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading

import cv2
import numpy as np

from bottle_vision import table
from bottle_vision.camera import frames
from bottle_vision.serve import make_handler

WINDOW = "circles"
# Hand-tuned on cam0_upside, 2026-09-27; retune if the camera height or lighting changes.
ACCUMULATOR, MIN_RADIUS, MAX_RADIUS = 15, 47, 54


def detect(frame, param2: int, min_r: int, max_r: int):
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(min_r * 2, 10),
                               param1=100, param2=max(param2, 1), minRadius=min_r, maxRadius=max_r)
    return [] if circles is None else np.round(circles[0]).astype(int)


def corners(x0, y0, w, h):
    return [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]


def draw_corners(frame, roi, corners_mm=None):
    cv2.rectangle(frame, roi[:2], (roi[0] + roi[2], roi[1] + roi[3]), (255, 0, 0), 2)
    for i, (px, py) in enumerate(corners(*roi)):
        label = f"{i + 1}" if corners_mm is None else f"{i + 1}: {corners_mm[i][0]:.0f},{corners_mm[i][1]:.0f} mm"
        cv2.putText(frame, label, (px + 6, py - 8 if i < 2 else py + 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 0, 0), 2)


def ask_xy(prompt):
    while True:
        try:
            x, y = map(float, input(prompt).replace(",", " ").split())
            return x, y
        except ValueError:
            print("  type two numbers, e.g. 350 -120")


def calibrate(frame, corners_mm=None):
    roi = tuple(int(v) for v in cv2.selectROI("drag search area, Enter to confirm", frame))
    cv2.destroyAllWindows()
    if roi[2] == 0 or roi[3] == 0:
        raise SystemExit("empty search rectangle")
    if corners_mm:
        return {"roi": list(roi), "corners_mm": corners_mm}
    view = frame.copy()
    draw_corners(view, roi)
    cv2.imshow("corners: note 1-4, press any key, then type mm in the console", view)
    cv2.waitKey(0)
    # input() must run with no window open, or Windows marks it unresponsive.
    cv2.destroyAllWindows()
    names = ["top-left", "top-right", "bottom-right", "bottom-left"]
    corners_mm = [ask_xy(f"corner {i + 1} ({name}) X Y mm: ") for i, name in enumerate(names)]
    return {"roi": list(roi), "corners_mm": corners_mm}


class Latest:
    """Newest annotated frame and circle list, shared with the HTTP server thread."""
    jpeg, circles = None, []

    def image(self):
        return self.jpeg

    def state(self):
        return self.circles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://10.42.0.200:8765/cam0_upside",
                        help="MJPEG URL, camera index, or video path")
    parser.add_argument("--calib", type=Path, default=Path("calibration/circle_roi.json"))
    parser.add_argument("--recalibrate", action="store_true", help="redo the rectangle and corner mm")
    parser.add_argument("--corners-mm", help='corner X,Y mm in order TL TR BR BL, e.g. "704,569 1130,569 1130,0 704,0"; skips typing')
    parser.add_argument("--headless", action="store_true", help="no window (servers); needs a saved --calib")
    parser.add_argument("--port", type=int, default=8771, help="serves /objects (JSON) and / (MJPEG); 0 disables")
    args = parser.parse_args()

    latest = Latest()
    if args.port:
        server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(latest))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"circles at http://0.0.0.0:{args.port}/objects")

    stream = frames(args.source)
    if args.headless and (args.recalibrate or not args.calib.exists()):
        raise SystemExit(f"--headless needs {args.calib}; calibrate once on a machine with a screen and copy it over")
    if args.recalibrate or not args.calib.exists():
        corners_mm = args.corners_mm and [list(map(float, c.split(","))) for c in args.corners_mm.split()]
        calib = calibrate(next(stream), corners_mm)
        args.calib.parent.mkdir(parents=True, exist_ok=True)
        args.calib.write_text(json.dumps(calib, indent=2))
        print(f"saved {args.calib}")
    else:
        calib = json.loads(args.calib.read_text())
    roi, corners_mm = tuple(calib["roi"]), calib["corners_mm"]
    x0, y0, w, h = roi
    # ponytail: 4 corners fit exactly, so a mistyped corner shows up only as wrong mm; add points via calibrate_table.py if accuracy matters.
    H, _ = table.fit(corners(*roi), corners_mm)

    if not args.headless:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("accumulator", WINDOW, ACCUMULATOR, 150, lambda _: None)

    for frame in stream:
        circles = detect(frame[y0:y0 + h, x0:x0 + w], ACCUMULATOR if args.headless else cv2.getTrackbarPos("accumulator", WINDOW), MIN_RADIUS, MAX_RADIUS)
        draw_corners(frame, roi, corners_mm)
        found = []
        for x, y, r in circles:
            x, y = x + x0, y + y0
            mx, my = table.apply(H, (x, y))
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(frame, f"{mx:.0f},{my:.0f} mm", (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 2)
            found.append({"x_mm": round(float(mx), 1), "y_mm": round(float(my), 1),
                          "px": int(x), "py": int(y), "r_px": int(r)})
        cv2.putText(frame, f"{len(circles)} circles", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        latest.circles, latest.jpeg = found, cv2.imencode(".jpg", frame)[1].tobytes()
        if args.headless:
            continue
        cv2.imshow(WINDOW, frame)
        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
