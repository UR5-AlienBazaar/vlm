#!/usr/bin/env python3
"""Collect labelled bottle crops from RGB webcam/video/MJPEG input.

Click a tracked bottle, then use c/m/s/u for cola/Mirinda/7UP/unknown.  The
label can be changed at any time; d removes that track's most recently saved
crop.  ByteTrack IDs come from Ultralytics' pretrained COCO bottle detector.
"""
import argparse
from collections import Counter
from datetime import datetime
from pathlib import Path

import cv2

from bottle_vision import DISPLAY_LABELS
from bottle_vision.camera import frames
from bottle_vision.labeling import CropCollector
from bottle_vision.tracker import YOLOByteTracker

KEY_LABELS = {ord("c"): "cola", ord("m"): "mirinda", ord("s"): "7up", ord("u"): None}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://10.42.0.50:8767/stream")
    parser.add_argument("--data", type=Path, default=Path("data/bottles"))
    parser.add_argument("--session", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--weights", default="yolo11n.pt")
    parser.add_argument("--min-interval", type=int, default=15)
    parser.add_argument("--max-per-track", type=int, default=50)
    args = parser.parse_args()
    tracker = YOLOByteTracker(args.weights)
    collector = CropCollector(args.data, args.session, args.min_interval, args.max_per_track)
    selected, labels, saved_counts, frame_number = None, {}, Counter(), 0

    def choose(event, x, y, _flags, _param):
        nonlocal selected
        if event == cv2.EVENT_LBUTTONDOWN:
            selected = next((item.track_id for item in tracks if item.contains(x, y)), selected)

    cv2.namedWindow("bottle labeling")
    cv2.setMouseCallback("bottle labeling", choose)
    for frame in frames(args.source):
        frame_number += 1
        tracks = tracker.update(frame)
        active = {item.track_id for item in tracks}
        for item in tracks:
            label = labels.get(item.track_id)
            if label:
                if collector.save(frame, item.box, item.track_id, label, frame_number):
                    saved_counts[label] += 1
            x0, y0, x1, y1 = item.box
            colour = (0, 255, 0) if item.track_id == selected else (0, 180, 255)
            text = f"ID {item.track_id} {DISPLAY_LABELS.get(label, 'UNLABELED')}"
            cv2.rectangle(frame, (x0, y0), (x1, y1), colour, 2)
            cv2.putText(frame, text, (x0, max(24, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
        summary = " ".join(f"{DISPLAY_LABELS[name]}:{saved_counts[name]}" for name in DISPLAY_LABELS)
        cv2.putText(frame, "click track; c=cola m=mirinda s=7up u=unknown d=delete q=quit", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
        cv2.putText(frame, summary, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
        cv2.imshow("bottle labeling", frame)
        key = cv2.waitKey(1) & 0xff
        if key == ord("q"):
            break
        if key in KEY_LABELS and selected in active:
            labels[selected] = KEY_LABELS[key]
        elif key == ord("d") and selected is not None and collector.delete_last(selected):
            old = labels.get(selected)
            if old:
                saved_counts[old] -= 1
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
