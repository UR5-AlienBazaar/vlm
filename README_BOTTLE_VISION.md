# Bottle vision

This RGB pipeline identifies `cola`, `mirinda`, and `7up` from the overhead
camera. It is separate from robot control and does not turn perception into a
motion command.

## Initial A100 result

Frozen DINOv2-small reference enrolment ran on `training-center-point` (A100)
using `photos/bottles` and `photos/Bottles-up`. Independent reference queries
returned Cola `0.933`, 7UP `0.939`, and Mirinda `0.631`; each cleared the
default `0.55` score floor and `0.025` runner-up margin. This is an embedding
prototype, not a fine-tuned classification-head result.

## Install

```bash
python -m venv .venv
.venv/bin/pip install -r requirements-bottle-vision.txt
```

For Windows, replace `.venv/bin/` with `.venv\Scripts\`. The Brev instance
needs the CUDA-compatible PyTorch wheel; its tested version is `2.7.1+cu126`.

## Collect labelled RGB crops

Run this on a machine that can open a GUI and reach the MJPEG stream:

```bash
python -m training.live_bottle_label --source http://10.42.0.50:8767/stream
```

The pretrained COCO YOLO model detects `bottle` and Ultralytics ByteTrack
assigns persistent IDs. Click a box, then press `c` (Cola), `m` (Mirinda),
`s` (7UP), or `u` (unknown). `d` deletes the most recently saved crop for the
selected track; `q` quits. The collector spaces saves, rejects tiny/blurry
crops, caps each track, and records image path, label, session, track, frame,
timestamp, and content fingerprint in `data/bottles/metadata.jsonl`.

Run a new `--session` after changing the physical arrangement or recording.
At least three independently recorded sessions containing all three labels are
needed for train/validation/test evaluation. Do not call separate frames from
one uninterrupted video separate sessions.

## Prepare and train

```bash
python -m training.prepare_bottle_dataset data/bottles/metadata.jsonl
python -m training.train_bottle_classifier --data data/bottles/split --stage head
python -m training.train_bottle_classifier --data data/bottles/split --stage finetune --epochs 6
```

Preparation deduplicates fingerprints and assigns entire sessions—never
individual frames—to one split. Training first freezes DINOv2-large and trains
the GELU/dropout head; `--stage finetune` unfreezes only its final four blocks
with a lower backbone learning rate. It refuses to run without non-empty,
class-consistent train and validation sets, rather than reporting misleading
metrics. `checkpoints/bottles/best.pt` is the output.

## Live classification

```bash
python -m training.live_bottle_infer checkpoints/bottles/best.pt \
  --source http://10.42.0.50:8767/stream
```

The same ByteTrack IDs are classified per crop. Fifteen predictions per ID are
averaged by default; below `--threshold` it displays `UNKNOWN`, and an ID that
vanishes loses its smoothing state. The detector and tracker are adapters, so
a custom YOLO bottle model can replace `yolo11n.pt` later without changing the
dataset or DINO components.
