#!/usr/bin/env python3
"""Crop reviewed bottle boxes into session-safe DINO training splits."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

LABELS = ("cola", "mirinda", "7up", "liqueur", "vodka", "ballantines")


def split_for(group):
    bucket = int(hashlib.sha1(group.encode()).hexdigest(), 16) % 100
    return "test" if bucket < 15 else "val" if bucket < 30 else "train"


def unpaired_bursts(records, gap_seconds):
    bursts, previous, name = {}, {}, {}
    for record in sorted((r for r in records if not r.get("capture_group")), key=lambda r: r["capture_timestamp_ns"]):
        camera, stamp = record["camera"], record["capture_timestamp_ns"]
        if camera not in previous or stamp - previous[camera] > gap_seconds * 1e9:
            name[camera] = f"{camera}-burst-{stamp}"
        previous[camera] = stamp; bursts[record["image"]] = name[camera]
    return bursts


def grouped_records(root, burst_gap=20.0):
    """Yield (record, group, split) for the final reviewed record of each image."""
    # Resuming the UI may write an earlier empty record, then a reviewed one.
    records = {}
    for line in (root / "annotations.jsonl").read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        if record.get("reviewed"): records[record["image"]] = record
    bursts = unpaired_bursts(records.values(), burst_gap)
    for image_name, record in records.items():
        # Keep both views of a paired group together. Unpaired frames stay
        # together per burst: near-duplicates never straddle splits, but one
        # long sequence no longer lands wholesale in a single split.
        group = record.get("capture_group") or bursts[image_name]
        yield record, group, split_for(group)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--out", type=Path, default=Path("data/bottle-crops"))
    parser.add_argument("--burst-gap", type=float, default=20.0, help="seconds of silence that start a new unpaired group")
    args = parser.parse_args(); root = args.capture_dir
    import cv2
    counts = Counter()
    for record, group, split in grouped_records(root, args.burst_gap):
        image_name = record["image"]; image = cv2.imread(str(root / image_name))
        if image is None: raise FileNotFoundError(root / image_name)
        for index, annotation in enumerate(record.get("annotations", [])):
            label = annotation["label"]
            if label not in LABELS: continue
            x0, y0, x1, y1 = annotation["bbox"]
            crop = image[max(0,y0):min(image.shape[0],y1), max(0,x0):min(image.shape[1],x1)]
            if crop.size == 0: continue
            output = args.out / split / label / f"{record['camera']}_{group}_{Path(image_name).stem}_{index}.jpg"
            output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(output), crop): raise RuntimeError(f"failed to write {output}")
            counts[(split, label)] += 1
    report = {split: {label: counts[(split,label)] for label in LABELS} for split in ("train","val","test")}
    print(json.dumps(report, indent=2))
    missing = [f"{split}/{label}" for split in ("train","val") for label in LABELS if not report[split][label]]
    if missing: raise SystemExit(f"missing labelled crops in {', '.join(missing)}; adjust the group split")


if __name__ == "__main__": main()
