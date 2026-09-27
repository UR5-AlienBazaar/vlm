#!/usr/bin/env python3
"""Human-label captured cam0/cam1 bottle images with tight boxes.

Draw a box with the mouse, then press c/m/s/j/v/b to label it as Cola,
Mirinda, 7UP, Jägermeister, vodka, or Ballantine's. Press n to save this image
and advance, d to remove the most recent box, q to stop. Annotations retain
the source camera and paired capture_group from captures.jsonl.
"""
import argparse
import json
from pathlib import Path

import cv2

KEYS = {ord("c"): "cola", ord("m"): "mirinda", ord("s"): "7up", ord("j"): "liqueur",
        ord("v"): "vodka", ord("b"): "ballantines"}
COLOURS = {"cola": (0, 0, 255), "mirinda": (0, 140, 255), "7up": (0, 220, 0),
           "liqueur": (0, 255, 0), "vodka": (220, 80, 220), "ballantines": (0, 0, 180)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--out", type=Path, help="defaults to <capture_dir>/annotations.jsonl")
    parser.add_argument("--camera", help="only show this camera, e.g. cam0")
    parser.add_argument("--paired", action="store_true", help="only show images with a capture_group")
    args = parser.parse_args(); root = args.capture_dir; out = args.out or root / "annotations.jsonl"
    metadata = {row["image"]: row for row in map(json.loads, (root / "captures.jsonl").read_text().splitlines())}
    images = [key for key, row in metadata.items() if (root / key).exists()
              and (not args.camera or row.get("camera") == args.camera) and (not args.paired or row.get("capture_group"))]
    # The dataset keeps only the last reviewed record per image, so revisiting
    # must start from the existing boxes or saving would erase them.
    existing = {}
    if out.exists():
        for record in map(json.loads, out.read_text(encoding="utf-8").splitlines()):
            if record.get("reviewed"):
                existing[record["image"]] = [(a["label"], tuple(a["bbox"])) for a in record["annotations"]]

    def load(position):
        return list(existing.get(images[position], [])) if position < len(images) else []

    index, drag_start, draft = 0, None, None
    boxes = load(0)

    def mouse(event, x, y, _flags, _param):
        nonlocal drag_start, draft
        if event == cv2.EVENT_LBUTTONDOWN: drag_start, draft = (x, y), None
        elif event == cv2.EVENT_MOUSEMOVE and drag_start: draft = (*drag_start, x, y)
        elif event == cv2.EVENT_LBUTTONUP and drag_start:
            x0, y0 = drag_start; draft = (min(x0,x), min(y0,y), max(x0,x), max(y0,y)); drag_start = None

    cv2.namedWindow("bottle capture labeling"); cv2.setMouseCallback("bottle capture labeling", mouse)
    with out.open("a", encoding="utf-8") as handle:
        while index < len(images):
            relative = images[index]; image = cv2.imread(str(root / relative)); view = image.copy()
            for label, (x0,y0,x1,y1) in boxes:
                cv2.rectangle(view,(x0,y0),(x1,y1),COLOURS[label],2); cv2.putText(view,label,(x0,max(22,y0-5)),cv2.FONT_HERSHEY_SIMPLEX,.65,COLOURS[label],2)
            if draft: cv2.rectangle(view, draft[:2], draft[2:], (255,255,255), 1)
            cv2.putText(view, f"{index+1}/{len(images)} draw; c/m/s/j/v/b label; n save-next; d undo; q quit", (12,30), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
            cv2.imshow("bottle capture labeling", view); key = cv2.waitKey(20) & 0xff
            if key == ord("q"): break
            if key == ord("d") and boxes: boxes.pop()
            elif key in KEYS and draft and (draft[2]-draft[0]) >= 20 and (draft[3]-draft[1]) >= 20:
                boxes.append((KEYS[key], draft)); draft = None
            elif key == ord("n"):
                handle.write(json.dumps({**metadata[relative], "annotations": [{"label": label, "bbox": list(box)} for label, box in boxes], "reviewed": True}) + "\n"); handle.flush()
                index, draft = index + 1, None; boxes = load(index)
    cv2.destroyAllWindows()


if __name__ == "__main__": main()
