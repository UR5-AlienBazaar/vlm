#!/usr/bin/env python3
"""Calibrate cam0_upside pixels to robot base-frame X/Y on the table plane.

Robot mode (default): freedrive the tool tip onto the table, mark the spot
with tape, press r to record the robot's X/Y. Repeat for 6-8 spots spread
over the working area. Then move the arm out of view, press f for a fresh
frame, and click the marks in the same order; each click takes the next
recorded point.

Manual mode (no robot): click a spot, type "X Y" in mm into the window,
Enter (Esc cancels). u undoes the last point, s fits and saves, q quits.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import struct

import cv2

from bottle_vision import table
from bottle_vision.camera import mjpeg_frames


def parse_base(text):
    try:
        x, y = map(float, text.replace(",", " ").split())
        return x, y
    except ValueError:
        return None


def robot_xy(host, port=30013):
    """Actual TCP X/Y in base-frame mm from UR's read-only real-time port."""
    with socket.create_connection((host, port), timeout=3) as connection:
        data = b""
        while len(data) < 4 or len(data) < struct.unpack("!i", data[:4])[0]:
            chunk = connection.recv(4096)
            if not chunk:
                raise ConnectionError("robot closed the connection")
            data += chunk
    # Real-time interface: actual_TCP_pose is 6 doubles (m, rad) at byte 444.
    x, y = struct.unpack("!2d", data[444:460])
    return round(x * 1000, 1), round(y * 1000, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://10.42.0.200:8765/cam0_upside")
    parser.add_argument("--image", type=Path, help="calibrate on a saved frame instead of the live camera")
    parser.add_argument("--out", type=Path, default=Path("calibration/cam0_table.json"))
    parser.add_argument("--robot", default="10.42.0.100", help="UR controller for r; empty to disable")
    args = parser.parse_args()
    frame = cv2.imread(str(args.image)) if args.image else next(mjpeg_frames(args.source))
    if frame is None:
        raise SystemExit(f"cannot read {args.image}")
    pixels, base, clicked = [], [], []

    def mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append((x, y))

    cv2.namedWindow("table calibration"); cv2.setMouseCallback("table calibration", mouse)
    pending, typed, note, recorded = None, "", "", []
    # Typing happens inside the window: blocking on input() in a console
    # stops OpenCV's event pump and Windows kills the window as unresponsive.
    while True:
        view = frame.copy()
        for i, ((px, py), (bx, by)) in enumerate(zip(pixels, base), 1):
            cv2.drawMarker(view, (px, py), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(view, f"{i}: {bx:.0f},{by:.0f}", (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
        if pending:
            cv2.drawMarker(view, pending, (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            status = f"point {len(pixels) + 1} at {pending}: type base X Y mm, Enter ok, Esc cancel > {typed}_"
        else:
            status = note or (f"{len(pixels)} paired, {len(recorded)} waiting: click mark {len(pixels) + 1}" if recorded else
                              f"{len(pixels)} points; r record robot; f fresh frame; click to pair/type; u undo; s save; q quit")
        cv2.rectangle(view, (0, 0), (view.shape[1], 42), (0, 0, 0), -1)
        cv2.putText(view, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
        cv2.imshow("table calibration", view); key = cv2.waitKey(30) & 0xff
        if clicked and recorded:
            pixels.append(clicked.pop()); base.append(recorded.pop(0)); clicked.clear(); note = ""
        elif clicked:
            pending, typed, note = clicked.pop(), "", ""; clicked.clear()
        elif pending:
            if key == 27:
                pending = None
            elif key in (13, 10):
                answer = parse_base(typed)
                if answer:
                    pixels.append(pending); base.append(answer); pending = None
                else:
                    typed = ""
            elif key == 8:
                typed = typed[:-1]
            elif chr(key) in "0123456789-+. ,":
                typed += chr(key)
        elif key == ord("u") and pixels:
            pixels.pop(); base.pop(); note = ""
        elif key == ord("q"):
            break
        elif key == ord("r") and args.robot:
            try:
                recorded.append(robot_xy(args.robot)); note = f"recorded {len(recorded)}: {recorded[-1]} mm"
            except (OSError, ConnectionError) as error:
                note = f"robot read failed: {error}"
        elif key == ord("f"):
            frame = next(mjpeg_frames(args.source)); note = "fresh frame"
        elif key == ord("s"):
            # 4 points always fit exactly (0 mm error), so they cannot reveal a bad point.
            if len(pixels) < 6:
                note = f"need at least 6 points to check accuracy, have {len(pixels)}"; continue
            copied = [i for i, ((px, py), (bx, by)) in enumerate(zip(pixels, base), 1) if abs(px - bx) < 1 and abs(py - by) < 1]
            if copied:
                note = f"points {copied} repeat their pixel numbers; u to remove, then press r at a touched spot"; continue
            if len({tuple(b) for b in base}) < len(base):
                note = "two points have the same robot position: move the tip to a new spot before each r"; continue
            try:
                H, errors = table.fit(pixels, base)
            except ValueError as error:
                note = str(error); continue
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps({"homography": H.tolist(), "pixels": pixels, "base_mm": base,
                                            "errors_mm": [round(float(e), 2) for e in errors],
                                            "image_size": list(frame.shape[1::-1]),
                                            "created_utc": datetime.now(timezone.utc).isoformat()}, indent=2))
            cv2.imwrite(str(args.out.with_suffix(".jpg")), view)
            errors_text = " ".join(f"{i}:{e:.0f}" for i, e in enumerate(errors, 1))
            # A point far worse than the rest is a typo or mis-click: u, redo it, s again.
            note = f"saved; error mm {errors_text}; q quit, u undo last"
            print(note, flush=True)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
