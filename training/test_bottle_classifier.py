import json
from pathlib import Path

import numpy as np

from bottle_classifier import (BottleClassifier, DISTRACTOR, Reference, label_from_filename,
                               references_from_dir)


def test_classifier_accepts_a_clear_match():
    classifier = BottleClassifier({"beer": np.array([1, 0]), "wine": np.array([0, 1])})

    result = classifier.classify_embedding(np.array([0.99, 0.1]), min_score=0.5, min_margin=0.1)

    assert result.label == "beer"
    assert result.runner_up == "wine"
    assert result.margin > 0.1


def test_classifier_rejects_an_ambiguous_or_known_distractor_crop():
    classifier = BottleClassifier({"beer": np.array([1, 0]), "wine": np.array([0, 1])},
                                  distractor=np.array([0.7, 0.7]))

    ambiguous = classifier.classify_embedding(np.array([0.7, 0.7]), min_score=0.5, min_margin=0.1)
    distractor = classifier.classify_embedding(np.array([1, 0]), min_score=0.5, min_margin=0.1,
                                                distractor_margin=0.4)

    assert ambiguous.label is None
    assert distractor.label is None
    assert distractor.distractor_score is not None


def test_reference_sidecar_takes_precedence_and_preserves_tight_box(tmp_path):
    (tmp_path / "jack_daniels_01.jpg").touch()
    (tmp_path / "ballantines_distractor_01.jpg").touch()
    (tmp_path / "labels.jsonl").write_text(
        json.dumps({"image": "jack_daniels_01.jpg", "focus": {"name": "whiskey", "bbox": [1, 2, 3, 4]}})
        + "\n" + json.dumps({"image": "ballantines_distractor_01.jpg", "focus": None,
                              "distractors": [{"name": "ballantines", "bbox": [5, 6, 7, 8]}]}) + "\n")

    references = references_from_dir(tmp_path)

    assert references == [Reference(tmp_path / "ballantines_distractor_01.jpg", DISTRACTOR, (5, 6, 7, 8)),
                          Reference(tmp_path / "jack_daniels_01.jpg", "whiskey", (1, 2, 3, 4))]


def test_new_drink_prefixes_are_recognised():
    assert label_from_filename(Path("cola_01.jpg")) == "cola"
    assert label_from_filename(Path("mirinda_01.jpg")) == "mirinda"
    assert label_from_filename(Path("7up_01.jpg")) == "7up"
    assert label_from_filename(Path("seven-up (12).jpg")) == "7up"
    assert label_from_filename(Path("ballentines-2 (1).jpg")) == DISTRACTOR
