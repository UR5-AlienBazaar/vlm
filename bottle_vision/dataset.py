"""Manifest records, crop quality checks, deduplication and session-safe splits."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import shutil

from . import LABELS


def crop_quality(crop) -> float:
    import cv2
    return float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())


def valid_crop(crop, min_side: int = 64, min_sharpness: float = 25.0) -> bool:
    return crop is not None and min(crop.shape[:2]) >= min_side and crop_quality(crop) >= min_sharpness


def crop_fingerprint(crop) -> str:
    import cv2
    tiny = cv2.resize(crop, (16, 16), interpolation=cv2.INTER_AREA)
    return hashlib.sha256(tiny.tobytes()).hexdigest()


def read_manifest(path: Path) -> list[dict]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    deleted = {row["deleted"] for row in rows if "deleted" in row}
    return [row for row in rows if "image" in row and row["image"] not in deleted]


def session_split(records: list[dict], val_fraction: float = 0.15, test_fraction: float = 0.15) -> dict[str, list[dict]]:
    """Assign whole sessions deterministically, never individual near-duplicate frames."""
    if val_fraction + test_fraction >= 1:
        raise ValueError("validation and test fractions must total less than one")
    groups = defaultdict(list)
    for record in records:
        if record["label"] in LABELS:
            groups[record["session"]].append(record)
    result = {"train": [], "val": [], "test": []}
    for session, items in groups.items():
        bucket = int(hashlib.sha1(session.encode()).hexdigest(), 16) % 10_000 / 10_000
        split = "test" if bucket < test_fraction else "val" if bucket < test_fraction + val_fraction else "train"
        result[split].extend(items)
    return result


def materialize_splits(manifest: Path, output: Path, val_fraction: float = .15, test_fraction: float = .15) -> dict[str, dict[str, int]]:
    records, seen = read_manifest(manifest), set()
    unique = []
    for record in records:
        if record.get("label") in LABELS and record.get("fingerprint") not in seen:
            unique.append(record)
            seen.add(record.get("fingerprint"))
    splits = session_split(unique, val_fraction, test_fraction)
    report = {split: {label: 0 for label in LABELS} for split in splits}
    for split, items in splits.items():
        for index, record in enumerate(items):
            label = record["label"]
            destination = output / split / label / f"{record['session']}_{index:06d}.jpg"
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(record["image"], destination)
            report[split][label] += 1
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report
