#!/usr/bin/env python3
"""Save periodic overhead frames as a new capture session for label_bottle_captures.py.

Move, rotate and occlude bottles while it runs; every saved frame should show a
different arrangement.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2

from bottle_vision.camera import mjpeg_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://localhost:8765/cam0_upside")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--out", type=Path, help="defaults to photos/capture_<UTC time>")
    parser.add_argument("--every", type=float, default=4.0, help="seconds between saved frames")
    parser.add_argument("--count", type=int, default=90)
    args = parser.parse_args()
    root = args.out or Path("photos") / f"capture_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    (root / "images" / args.camera).mkdir(parents=True, exist_ok=True)
    saved, next_save = 0, 0.0
    with (root / "captures.jsonl").open("a", encoding="utf-8") as index:
        for sequence, frame in enumerate(mjpeg_frames(args.source)):
            if time.monotonic() < next_save:
                continue
            now = datetime.now(timezone.utc)
            relative = f"images/{args.camera}/{args.camera}_{now:%Y%m%dT%H%M%S%fZ}_{sequence:06d}.jpg"
            if not cv2.imwrite(str(root / relative), frame):
                raise SystemExit(f"could not write {root / relative}")
            height, width = frame.shape[:2]
            index.write(json.dumps({"image": relative, "camera": args.camera, "capture_timestamp_ns": time.time_ns(),
                                    "saved_at_utc": now.isoformat(), "frame_id": args.camera, "sequence": sequence,
                                    "width": width, "height": height, "annotated": False}) + "\n")
            index.flush(); saved += 1; next_save = time.monotonic() + args.every
            print(f"{saved}/{args.count} {relative}", flush=True)
            if saved >= args.count:
                break


if __name__ == "__main__":
    main()
