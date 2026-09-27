#!/usr/bin/env python3
"""Classify a single `cam0_upside` bottle crop with frozen DINOv2 embeddings.

This is deliberately an on-demand classifier: it does not locate bottles in a
frame.  Enrol references once, then pass the crop of one bottle to
``classify``.  A low-confidence result is ``label: null``, never a forced
drink guess.

Examples (on the Brev GPU instance)::

    pip install -q "numpy<2" opencv-python-headless torch transformers pillow
    python bottle_classifier.py --ref-dir photos/bottles --image candidate.jpg
    python bottle_classifier.py --ref-dir photos/bottles --ref beer=heineken_01.jpg \
        --ref wine=frontera_01.jpg --image candidate.jpg

``photos/bottles/labels.jsonl`` is used when present.  Its tight focus box is
cropped before embedding, so the checked-in phone photographs remain useful as
references even though they are not themselves bottle-only crops.  New
single-bottle crops need no sidecar entry.
"""
import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re
from typing import Iterable

import numpy as np

# These are planner-facing names, not model class numbers.  Keep them aligned
# with training/vlm_labels.py; `cola` keeps its historic id there.
DRINK_LABELS = frozenset({
    "whiskey", "beer", "vodka", "liqueur", "gin", "wine", "cola", "mirinda", "7up",
})
DISTRACTOR = "distractor"
NAME_ALIASES = {
    "jack_daniels": "whiskey",
    "jagermeister": "liqueur",
    "seven_up": "7up",
    "ballantines": DISTRACTOR,
    "ballentines": DISTRACTOR,
}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)


@dataclass(frozen=True)
class Reference:
    path: Path
    label: str
    box: tuple[int, int, int, int] | None = None


@dataclass(frozen=True)
class Classification:
    label: str | None
    best_score: float
    margin: float
    runner_up: str | None
    distractor_score: float | None


def canonical_label(name: str) -> str:
    """Map reference metadata/file prefixes to a public drink label."""
    name = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    return NAME_ALIASES.get(name, name)


def label_from_filename(path: Path) -> str | None:
    """Infer a label for unannotated reference crops from their filename."""
    stem = re.sub(r"\s*\(\d+\)$", "", path.stem.lower())
    stem = re.sub(r"[^a-z0-9]+", "_", stem).strip("_")
    for name in sorted((*DRINK_LABELS, *NAME_ALIASES), key=len, reverse=True):
        if stem == name or stem.startswith(name + "_"):
            return canonical_label(name)
    if "distractor" in stem:
        return DISTRACTOR
    return None


def references_from_dir(ref_dir: Path) -> list[Reference]:
    """Load labelled references, preferring labels.jsonl over filename guesses."""
    metadata: dict[str, Reference] = {}
    sidecar = ref_dir / "labels.jsonl"
    if sidecar.exists():
        for line in sidecar.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            path = ref_dir / row["image"]
            focus = row.get("focus")
            if focus:
                label = canonical_label(focus["name"])
                box = tuple(int(v) for v in focus["bbox"])
            elif row.get("distractors"):
                label, box = DISTRACTOR, tuple(int(v) for v in row["distractors"][0]["bbox"])
            else:
                continue
            metadata[path.name] = Reference(path, label, box)

    references = []
    for path in sorted(ref_dir.iterdir()):
        if path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        reference = metadata.get(path.name)
        if reference is None:
            label = label_from_filename(path)
            if label is not None:
                reference = Reference(path, label)
        if reference is not None:
            references.append(reference)
    return references


def parse_extra_reference(value: str) -> Reference:
    """Parse ``LABEL=PATH`` for a crop supplied in addition to --ref-dir."""
    try:
        name, raw_path = value.split("=", 1)
    except ValueError as error:
        raise argparse.ArgumentTypeError("--ref must be LABEL=PATH") from error
    label = canonical_label(name)
    if label not in DRINK_LABELS and label != DISTRACTOR:
        raise argparse.ArgumentTypeError(f"unknown reference label {name!r}")
    return Reference(Path(raw_path), label)


class DinoV2Embedder:
    """GPU DINOv2 whole-crop embeddings, normalized for cosine similarity."""
    def __init__(self, model_id: str = "facebook/dinov2-small", device: str | None = None,
                 image_size: int = 518):
        import torch
        from transformers import AutoModel

        self.torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.image_size = image_size
        self.model = AutoModel.from_pretrained(model_id).to(self.device).eval()
        self.dtype = torch.float16 if self.device.startswith("cuda") else torch.float32
        if self.dtype == torch.float16:
            self.model.half()

    def embed_bgr(self, bgr: np.ndarray) -> np.ndarray:
        import cv2

        if bgr is None or bgr.size == 0:
            raise ValueError("cannot embed an empty image crop")
        rgb = cv2.resize(bgr, (self.image_size, self.image_size), interpolation=cv2.INTER_AREA)[:, :, ::-1].copy()
        x = self.torch.from_numpy(rgb).permute(2, 0, 1).float() / 255
        mean = self.torch.tensor(MEAN).view(3, 1, 1)
        std = self.torch.tensor(STD).view(3, 1, 1)
        x = ((x - mean) / std)[None].to(self.device, dtype=self.dtype)
        with self.torch.inference_mode():
            output = self.model(pixel_values=x)
            pooled = output.pooler_output
        vector = pooled[0].float().cpu().numpy()
        return vector / np.linalg.norm(vector)


def read_reference_crop(reference: Reference) -> np.ndarray:
    import cv2

    image = cv2.imread(str(reference.path))
    if image is None:
        raise FileNotFoundError(f"cannot read reference image {reference.path}")
    if reference.box is None:
        return image
    x0, y0, x1, y1 = reference.box
    height, width = image.shape[:2]
    x0, x1 = max(0, x0), min(width, x1)
    y0, y1 = max(0, y0), min(height, y1)
    if x0 >= x1 or y0 >= y1:
        raise ValueError(f"invalid crop {reference.box} for {reference.path}")
    return image[y0:y1, x0:x1]


class BottleClassifier:
    """Class prototypes plus a distractor gate over normalized DINO embeddings."""
    def __init__(self, prototypes: dict[str, np.ndarray], distractor: np.ndarray | None = None):
        if not prototypes:
            raise ValueError("at least one drink reference is required")
        self.prototypes = {label: self._unit(vector) for label, vector in prototypes.items()}
        self.distractor = self._unit(distractor) if distractor is not None else None

    @staticmethod
    def _unit(vector: np.ndarray) -> np.ndarray:
        vector = np.asarray(vector, dtype=np.float32)
        norm = np.linalg.norm(vector)
        if norm == 0:
            raise ValueError("embedding must be non-zero")
        return vector / norm

    @classmethod
    def enrol(cls, embedder: DinoV2Embedder, references: Iterable[Reference]) -> "BottleClassifier":
        vectors: dict[str, list[np.ndarray]] = {}
        for reference in references:
            vectors.setdefault(reference.label, []).append(embedder.embed_bgr(read_reference_crop(reference)))
        prototypes = {label: cls._unit(np.mean(items, axis=0))
                      for label, items in vectors.items() if label != DISTRACTOR}
        distractors = vectors.get(DISTRACTOR, [])
        distractor = cls._unit(np.mean(distractors, axis=0)) if distractors else None
        return cls(prototypes, distractor)

    def classify_embedding(self, embedding: np.ndarray, min_score: float = 0.55,
                           min_margin: float = 0.025, distractor_margin: float = 0.0) -> Classification:
        vector = self._unit(embedding)
        ranked = sorted(((float(vector @ prototype), label) for label, prototype in self.prototypes.items()), reverse=True)
        best_score, best_label = ranked[0]
        second_score, second_label = ranked[1] if len(ranked) > 1 else (-1.0, None)
        margin = best_score - second_score
        distractor_score = float(vector @ self.distractor) if self.distractor is not None else None
        accepted = (best_score >= min_score and margin >= min_margin and
                    (distractor_score is None or best_score - distractor_score >= distractor_margin))
        return Classification(best_label if accepted else None, best_score, margin, second_label, distractor_score)

    def classify(self, embedder: DinoV2Embedder, crop: np.ndarray, **thresholds: float) -> Classification:
        return self.classify_embedding(embedder.embed_bgr(crop), **thresholds)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref-dir", type=Path, action="append", required=True,
                        help="reference crop directory; repeat to combine sets")
    parser.add_argument("--ref", type=parse_extra_reference, action="append", default=[],
                        help="additional whole crop as LABEL=PATH; repeatable")
    parser.add_argument("--image", type=Path, required=True, help="single-bottle crop to classify")
    parser.add_argument("--model", default="facebook/dinov2-small")
    parser.add_argument("--device", help="torch device (defaults to cuda when available)")
    parser.add_argument("--min-score", type=float, default=0.55, help="minimum cosine similarity")
    parser.add_argument("--min-margin", type=float, default=0.025, help="lead over second-best label")
    parser.add_argument("--distractor-margin", type=float, default=0.0,
                        help="required lead over known-distractor prototype")
    args = parser.parse_args()

    references = [reference for ref_dir in args.ref_dir for reference in references_from_dir(ref_dir)] + args.ref
    if not references:
        raise SystemExit(f"no labelled references in {', '.join(map(str, args.ref_dir))}")
    embedder = DinoV2Embedder(args.model, args.device)
    classifier = BottleClassifier.enrol(embedder, references)
    crop = read_reference_crop(Reference(args.image, "query"))
    result = classifier.classify(embedder, crop, min_score=args.min_score,
                                 min_margin=args.min_margin, distractor_margin=args.distractor_margin)
    print(json.dumps(asdict(result)))


if __name__ == "__main__":
    main()
