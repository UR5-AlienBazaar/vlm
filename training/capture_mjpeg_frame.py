#!/usr/bin/env python3
"""Save one current BGR frame from the overhead MJPEG stream."""
import argparse
from pathlib import Path

import cv2

from bottle_vision.camera import mjpeg_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--source", default="http://localhost:8765/cam0_upside")
    args = parser.parse_args()
    frame = next(mjpeg_frames(args.source))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.out), frame):
        raise SystemExit(f"could not write {args.out}")
    print(frame.shape)


if __name__ == "__main__":
    main()
