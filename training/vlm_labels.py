#!/usr/bin/env python3
"""Turn captured Gazebo frames into a chat-format JSONL dataset for the VLM scene-checker.

Raw layout, written by capture_vlm_frames.py:
    <raw>/<scene>/scene.json            {"bottles": ["whiskey", "cola"], "glasses": ["glass"]}
    <raw>/<scene>/<frame>.json          {"in_gripper": "whiskey" | null}
    <raw>/<scene>/<frame>_<cam>_rgb.png
    <raw>/<scene>/<frame>_<cam>_labels.png   single channel, pixel = LABEL_IDS value

Image paths in the JSONL are relative to <raw>, so training needs <raw> as its image root.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

# glass_b is ROADMAP Phase D's second glass at (-0.04, 0.55), for arm B. It is
# here now so the answer format does not change, and force a retrain, when it lands.
LABEL_IDS = {'whiskey': 1, 'cola': 2, 'beer': 3,
             'glass': 10, 'glass_b': 11, 'distractor': 20}
# Every label in 20-29 is a distractor. The workcell capture gives each one its
# own id so two distractors standing together never merge into one box.
DISTRACTOR_IDS = range(20, 30)
# Fewer pixels than this counts as not visible. The smallest full view is a
# bottle seen top-down by the 320x240 overhead camera: 72 deg FOV from 1.1m is
# ~5mm/px, so a 32mm-radius body is a ~13px disc, ~130px. 40px is ~30% of that,
# i.e. most of it hidden. Wrist and stand views are larger, so this only gets
# more lenient there. Estimated from geometry; re-measure on captured frames.
MIN_VISIBLE_PX = 40
# Obstruction = the distractor's box overlaps a target's box grown by this
# fraction, i.e. it stands in front of or right against what the arm reaches for.
OBSTRUCTION_MARGIN = 0.10
VAL_PERCENT = 10

PROMPT = ('Describe the bar scene as JSON with keys bottles (name, visible, confidence, bbox), '
          'glasses (name, visible, confidence, bbox), in_gripper ({value: bottle name or null, '
          'confidence}) and obstruction ({value: true if something blocks a bottle or a glass, '
          'confidence}). bbox is [x0, y0, x1, y1] in 0-1000 image coordinates. confidence is 0-1: '
          'how sure you are of visible or value.')


def bbox(labels, label_id):
    """Box of `label_id` in 0-1000 coordinates, or None if too little of it shows.

    0-1000 is Qwen3-VL's native grounding format; Qwen2.5-VL would want pixels.
    """
    ys, xs = np.nonzero(labels == label_id)
    if len(xs) < MIN_VISIBLE_PX:
        return None
    h, w = labels.shape
    return [round(1000 * xs.min() / w), round(1000 * ys.min() / h),
            round(1000 * (xs.max() + 1) / w), round(1000 * (ys.max() + 1) / h)]


def iou_boxes(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _overlaps(a, b, margin):
    grow_x = (b[2] - b[0]) * margin
    grow_y = (b[3] - b[1]) * margin
    return (a[0] < b[2] + grow_x and a[2] > b[0] - grow_x and
            a[1] < b[3] + grow_y and a[3] > b[1] - grow_y)


def visibility_confidence(px):
    """How clear-cut `visible` is for an object showing `px` pixels.

    0.5 at the MIN_VISIBLE_PX cut-off, where the label is a coin flip, rising to
    1.0 at 0 px or twice the cut-off. Measured against the fixed cut-off rather
    than the object's full size: a half-hidden bottle filling the wrist view is
    unambiguously visible, and GRPO's Brier term should not teach it otherwise.
    """
    return round(0.5 + 0.5 * min(1.0, abs(px - MIN_VISIBLE_PX) / MIN_VISIBLE_PX), 2)


def _objects(labels, names):
    objects = []
    for name in names:
        box = bbox(labels, LABEL_IDS[name])
        px = int(np.count_nonzero(labels == LABEL_IDS[name]))
        objects.append({'name': name, 'visible': box is not None,
                        'confidence': visibility_confidence(px), 'bbox': box})
    return objects


def scene_label(labels, bottles, glasses, in_gripper):
    """The JSON answer the VLM is trained to give for one frame.

    in_gripper comes from the simulator's state, not the image: the model learns
    to see it, but a bottle hidden in the fingers is still labelled as held.
    Both flags are sim state, so their confidence is 1.0.
    """
    objects = {'bottles': _objects(labels, bottles), 'glasses': _objects(labels, glasses)}
    targets = [o['bbox'] for group in objects.values() for o in group if o['bbox']]
    obstruction = any(_overlaps(d, t, OBSTRUCTION_MARGIN)
                      for d in distractor_boxes(labels) for t in targets)
    return {
        **objects,
        'in_gripper': {'value': in_gripper, 'confidence': 1.0},
        'obstruction': {'value': obstruction, 'confidence': 1.0},
    }


def distractor_boxes(labels):
    return [b for b in (bbox(labels, i) for i in DISTRACTOR_IDS if i in labels) if b]


def bucket(scene, label, labels):
    """Coarse difficulty bucket for per-bucket eval: scene kind / distractor / bottles shown."""
    if label['obstruction']['value']:
        distractor = 'near'
    else:
        distractor = 'far' if distractor_boxes(labels) else 'none'
    shown = '+'.join(b['name'] for b in label['bottles'] if b['visible']) or 'no_bottle'
    return f"{scene.get('kind', 'static')}/{distractor}/{shown}"


def chat_record(image_path, label):
    return {
        'images': [str(image_path)],
        'messages': [
            {'role': 'user', 'content': [{'type': 'image'}, {'type': 'text', 'text': PROMPT}]},
            {'role': 'assistant', 'content': [{'type': 'text', 'text': json.dumps(label)}]},
        ],
    }


def is_val(scene_id):
    """Split by scene, stably, so frames of one scene never land in both splits."""
    digest = hashlib.sha1(scene_id.encode(), usedforsecurity=False).hexdigest()
    return int(digest, 16) % 100 < VAL_PERCENT


def export(raw, out, pour_stride=1):
    """Write train.jsonl and val.jsonl. Pour scenes keep every `pour_stride`-th
    frame: at ~1.7 fps consecutive pour frames are near-duplicates."""
    out.mkdir(parents=True, exist_ok=True)
    counts = {'train': 0, 'val': 0}
    with open(out / 'train.jsonl', 'w') as train, open(out / 'val.jsonl', 'w') as val:
        for scene in sorted(p for p in raw.iterdir() if p.is_dir()):
            meta = json.loads((scene / 'scene.json').read_text())
            stride = pour_stride if meta.get('kind') == 'pour' else 1
            split = 'val' if is_val(scene.name) else 'train'
            for labels_png in sorted(scene.glob('*_labels.png')):
                frame, camera = labels_png.name.split('_')[:2]
                if int(frame) % stride:
                    continue
                in_gripper = json.loads((scene / f'{frame}.json').read_text())['in_gripper']
                labels = np.asarray(Image.open(labels_png))
                rgb = labels_png.with_name(labels_png.name.replace('_labels', '_rgb'))
                label = scene_label(labels, meta['bottles'], meta['glasses'], in_gripper)
                record = chat_record(rgb.relative_to(raw).as_posix(), label)
                record.update(bucket=bucket(meta, label, labels), camera=camera)
                (val if split == 'val' else train).write(json.dumps(record) + '\n')
                counts[split] += 1
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('raw', type=Path, help='capture_vlm_frames.py output directory')
    parser.add_argument('out', type=Path, help='where train.jsonl and val.jsonl go')
    parser.add_argument('--pour-stride', type=int, default=20,
                        help='keep every Nth frame of a pour scene')
    opts = parser.parse_args()
    print(export(opts.raw, opts.out, opts.pour_stride))


if __name__ == '__main__':
    main()
