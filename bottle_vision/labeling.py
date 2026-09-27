"""Quality-gated crop persistence for an interactively labelled track."""
import json
from pathlib import Path
import time

from . import LABELS
from .dataset import crop_fingerprint, valid_crop


class CropCollector:
    def __init__(self, root: Path, session: str, min_interval: int = 15, max_per_track: int = 50):
        self.root, self.session = root, session
        self.min_interval, self.max_per_track = min_interval, max_per_track
        self.last_frame: dict[int, int] = {}
        self.counts: dict[int, int] = {}
        self.saved: dict[int, list[Path]] = {}
        self.manifest = root / "metadata.jsonl"

    def save(self, frame, box: tuple[int, int, int, int], track_id: int, label: str, frame_number: int) -> bool:
        if label not in LABELS or frame_number - self.last_frame.get(track_id, -self.min_interval) < self.min_interval:
            return False
        if self.counts.get(track_id, 0) >= self.max_per_track:
            return False
        x0, y0, x1, y1 = box
        crop = frame[max(0, y0):max(0, y1), max(0, x0):max(0, x1)]
        if not valid_crop(crop):
            return False
        import cv2
        path = self.root / "raw" / self.session / label / f"track{track_id}_frame{frame_number}.jpg"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), crop):
            raise RuntimeError(f"failed to write {path}")
        record = {"image": str(path.resolve()), "label": label, "track_id": track_id,
                  "session": self.session, "frame": frame_number, "captured_at": time.time(),
                  "fingerprint": crop_fingerprint(crop)}
        with self.manifest.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
        self.last_frame[track_id], self.counts[track_id] = frame_number, self.counts.get(track_id, 0) + 1
        self.saved.setdefault(track_id, []).append(path.resolve())
        return True

    def delete_last(self, track_id: int) -> bool:
        """Remove the newest crop for a track and record that deletion in the manifest."""
        paths = self.saved.get(track_id, [])
        if not paths:
            return False
        path = paths.pop()
        if path.exists():
            path.unlink()
        with self.manifest.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps({"deleted": str(path)}) + "\n")
        return True
