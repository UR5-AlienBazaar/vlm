#!/usr/bin/env python3
"""Human-label captured cam0/cam1 bottle images with tight boxes.

Draw a box with the mouse, then press c/m/s/j/v/b/w/g/e/i to label it as Cola,
Mirinda, 7UP, Jägermeister, vodka, Ballantine's, whiskey, gin, beer or wine. Press n to save this image
and advance, d to remove the most recent box, right-click to delete the box
under the cursor, a label key over a box to rename it, q to stop. --seed prefills unreviewed images with draft boxes. Annotations retain
the source camera and paired capture_group from captures.jsonl.
"""
import argparse
import json
from pathlib import Path

import cv2

KEYS = {ord("c"): "cola", ord("m"): "mirinda", ord("s"): "7up", ord("j"): "liqueur",
        ord("v"): "vodka", ord("b"): "ballantines", ord("w"): "whiskey", ord("g"): "gin",
        ord("e"): "beer", ord("i"): "wine"}
COLOURS = {"cola": (0, 0, 255), "mirinda": (0, 140, 255), "7up": (0, 220, 0),
           "liqueur": (0, 255, 0), "vodka": (220, 80, 220), "ballantines": (0, 0, 180),
           "whiskey": (40, 40, 40), "gin": (255, 200, 0), "beer": (0, 120, 0), "wine": (200, 200, 255)}


def seed_boxes(record):
    """Accept annotations.jsonl records and prelabel_*.jsonl VLM answers alike."""
    if "annotations" in record:
        return [(a["label"], tuple(a["bbox"])) for a in record["annotations"] if a["label"] in COLOURS]
    try:
        bottles = json.loads(record.get("answer") or "{}").get("bottles", [])
    except (json.JSONDecodeError, AttributeError):
        return []
    return [(b["name"], tuple(map(int, b["bbox"]))) for b in bottles
            if b.get("visible") and b.get("name") in COLOURS and len(b.get("bbox") or []) == 4]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--out", type=Path, help="defaults to <capture_dir>/annotations.jsonl")
    parser.add_argument("--camera", help="only show this camera, e.g. cam0")
    parser.add_argument("--seed", type=Path, help="unreviewed annotations.jsonl-shaped draft boxes to start from")
    parser.add_argument("--paired", action="store_true", help="only show images with a capture_group")
    args = parser.parse_args(); root = args.capture_dir; out = args.out or root / "annotations.jsonl"
    captures = root / "captures.jsonl"
    if captures.exists():
        metadata = {row["image"]: row for row in map(json.loads, captures.read_text().splitlines())}
    else:  # raw camera dumps (e.g. data/overhead_raw/<day>/<camera>/*.jpg) have no capture log
        metadata = {p.relative_to(root).as_posix(): {"image": p.relative_to(root).as_posix(), "camera": p.parent.name}
                    for p in sorted(root.glob("*/*.jpg"))}
    images = [key for key, row in metadata.items() if (root / key).exists()
              and (not args.camera or row.get("camera") == args.camera) and (not args.paired or row.get("capture_group"))]
    seeds = {}
    if args.seed:
        by_name = {Path(key).name: key for key in images}
        for record in map(json.loads, args.seed.read_text(encoding="utf-8").splitlines()):
            key = record["image"] if record["image"] in metadata else by_name.get(Path(record["image"]).name)
            if key: seeds[key] = seed_boxes(record)
    # The dataset keeps only the last reviewed record per image, so revisiting
    # must start from the existing boxes or saving would erase them.
    existing = {}
    if out.exists():
        for record in map(json.loads, out.read_text(encoding="utf-8").splitlines()):
            if record.get("reviewed"):
                existing[record["image"]] = [(a["label"], tuple(a["bbox"])) for a in record["annotations"]]

    def overlap(a, b):
        w = min(a[2], b[2]) - max(a[0], b[0]); h = min(a[3], b[3]) - max(a[1], b[1])
        inter = max(w, 0) * max(h, 0)
        return inter / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)

    def load(position, previous=()):
        if position >= len(images): return []
        if images[position] in existing: return list(existing[images[position]])
        # Seeds place boxes well but name overhead bottles poorly, and the layout
        # rarely changes between frames, so a seed box inherits the name of the
        # previous frame's reviewed box it overlaps.
        renamed = []
        for label, box in seeds.get(images[position], []):
            match = max(previous, key=lambda p: overlap(p[1], box), default=None)
            renamed.append((match[0] if match and overlap(match[1], box) > 0.5 else label, box))
        return renamed

    index, drag_start, draft, cursor = 0, None, None, (0, 0)
    boxes = load(0)

    def box_under(x, y):
        hits = [b for b in boxes if b[1][0] <= x <= b[1][2] and b[1][1] <= y <= b[1][3]]
        return min(hits, key=lambda b: (b[1][2]-b[1][0]) * (b[1][3]-b[1][1])) if hits else None

    def mouse(event, x, y, _flags, _param):
        nonlocal drag_start, draft, cursor
        cursor = (x, y)
        if event == cv2.EVENT_RBUTTONDOWN and (hit := box_under(x, y)): boxes.remove(hit)
        elif event == cv2.EVENT_LBUTTONDOWN: drag_start, draft = (x, y), None
        elif event == cv2.EVENT_MOUSEMOVE and drag_start: draft = (*drag_start, x, y)
        elif event == cv2.EVENT_LBUTTONUP and drag_start:
            x0, y0 = drag_start; draft = (min(x0,x), min(y0,y), max(x0,x), max(y0,y)); drag_start = None
            if draft[2]-draft[0] < 20 or draft[3]-draft[1] < 20: draft = None

    cv2.namedWindow("bottle capture labeling"); cv2.setMouseCallback("bottle capture labeling", mouse)
    with out.open("a", encoding="utf-8") as handle:
        while index < len(images):
            relative = images[index]; image = cv2.imread(str(root / relative)); view = image.copy()
            for label, (x0,y0,x1,y1) in boxes:
                cv2.rectangle(view,(x0,y0),(x1,y1),COLOURS[label],2); cv2.putText(view,label,(x0,max(22,y0-5)),cv2.FONT_HERSHEY_SIMPLEX,.65,COLOURS[label],2)
            if draft: cv2.rectangle(view, draft[:2], draft[2:], (255,255,255), 1)
            cv2.putText(view, f"{index+1}/{len(images)} draw; c/m/s/j/v/b/w/g/e/i label; n save-next; d undo; right-click delete; q quit", (12,30), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
            cv2.imshow("bottle capture labeling", view); key = cv2.waitKey(20) & 0xff
            if key == ord("q"): break
            if key == ord("d") and boxes: boxes.pop()
            elif key in KEYS and draft and (draft[2]-draft[0]) >= 20 and (draft[3]-draft[1]) >= 20:
                boxes.append((KEYS[key], draft)); draft = None
            elif key in KEYS and not draft and (hit := box_under(*cursor)):
                boxes[boxes.index(hit)] = (KEYS[key], hit[1])
            elif key == ord("n"):
                handle.write(json.dumps({**metadata[relative], "annotations": [{"label": label, "bbox": list(box)} for label, box in boxes], "reviewed": True}) + "\n"); handle.flush()
                index, draft = index + 1, None; boxes = load(index, boxes)
    cv2.destroyAllWindows()


if __name__ == "__main__": main()
