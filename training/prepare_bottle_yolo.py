#!/usr/bin/env python3
"""Export reviewed bottle boxes as a one-class YOLO dataset with the same session split."""
import argparse
from pathlib import Path
import shutil

try:
    from training.prepare_annotated_bottle_captures import LABELS, grouped_records
except ImportError:
    from prepare_annotated_bottle_captures import LABELS, grouped_records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dirs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, default=Path("data/bottle-yolo"))
    args = parser.parse_args()
    for root in args.capture_dirs:
        for record, _group, split in grouped_records(root):
            # Identity is the classifier's job; the detector only has to find every bottle.
            boxes = [a["bbox"] for a in record["annotations"] if a["label"] in LABELS]
            width, height = record["width"], record["height"]
            stem = f"{root.name}_{Path(record['image']).stem}"
            (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
            (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)
            shutil.copy(root / record["image"], args.out / "images" / split / f"{stem}.jpg")
            lines = [f"0 {(x0 + x1) / 2 / width:.6f} {(y0 + y1) / 2 / height:.6f} {(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}"
                     for x0, y0, x1, y1 in boxes]
            (args.out / "labels" / split / f"{stem}.txt").write_text("\n".join(lines))
    (args.out / "data.yaml").write_text(f"path: {args.out.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: bottle\n")
    for split in ("train", "val", "test"):
        print(split, len(list((args.out / "labels" / split).glob("*.txt"))), "images")


if __name__ == "__main__": main()
