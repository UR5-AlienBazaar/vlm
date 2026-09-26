#!/usr/bin/env python3
"""Zero-shot bake-off: off-the-shelf VLMs on the scene-checker val split, to decide whether we need to fine-tune.

    python compare_vlms.py run --model Qwen/Qwen3-VL-8B-Instruct --data data/vlm --image-root data/vlm_raw --out outputs/bakeoff
    python compare_vlms.py report outputs/bakeoff

One `run` per model and process: vLLM does not release GPU memory reliably
when a second LLM is built in the same process. `--prompt train` scores a
fine-tuned checkpoint with the prompt it was trained on, on the same frames.
"""
import argparse
import base64
import json
import time
from pathlib import Path

from vlm_labels import PROMPT
from vlm_reward import R_INVALID, combine, decide, read_scene, scores

# The training prompt names no objects: a fine-tuned model learned them from
# the labels, a zero-shot one needs them spelled out to have a fair chance.
ZERO_SHOT_PROMPT = (
    'You are the camera of a robot bartender. The bar may hold up to three bottles, named exactly: '
    '"whiskey" (square Jack Daniel\'s bottle with a black label), "cola" (cola bottle) and "beer" '
    '(brown beer bottle). The glass is named "glass". Any other object (box, can, carton) is a '
    'distractor, never a bottle or a glass. in_gripper is the bottle held in the robot gripper, or null.\n'
    + PROMPT +
    '\nList all three bottles and the glass, with visible false for ones you cannot see. Answer with '
    'the JSON object only, for example:\n'
    '{"bottles": [{"name": "whiskey", "visible": true, "confidence": 0.9, "bbox": [120, 300, 180, 520]}, '
    '{"name": "cola", "visible": false, "confidence": 0.8, "bbox": null}, '
    '{"name": "beer", "visible": false, "confidence": 0.8, "bbox": null}], '
    '"glasses": [{"name": "glass", "visible": true, "confidence": 0.9, "bbox": [600, 350, 660, 450]}], '
    '"in_gripper": {"value": null, "confidence": 0.9}, "obstruction": {"value": false, "confidence": 0.8}}')


def load_val(data, image_root, limit=None):
    rows = []
    for line in (Path(data) / 'val.jsonl').read_text().splitlines()[:limit]:
        record = json.loads(line)
        rows.append({'image': Path(image_root) / record['images'][0], 'camera': record['camera'],
                     'bucket': record['bucket'],
                     'label': record['messages'][1]['content'][0]['text']})
    return rows


def image_part(path):
    data = base64.b64encode(Path(path).read_bytes()).decode()
    return {'type': 'image_url', 'image_url': {'url': 'data:image/png;base64,' + data}}


def tally(pred, label):
    """(right, total) decisions and (unsafe go, total not-go) for one canonical prediction."""
    truth = decide(label)
    guess = decide(pred, bottles=list(label['bottles'])) if pred else {}
    not_go = [k for k, v in truth.items() if v != 'go']
    return {'decisions': (sum(guess.get(k) == v for k, v in truth.items()), len(truth)),
            # A 'go' where the truth is 'no' or 'unsure' is the error that makes the arm act on a wrong picture.
            'unsafe_go': (sum(guess.get(k) == 'go' for k in not_go), len(not_go))}


def ratio(tallies, key):
    total = sum(t[key][1] for t in tallies)
    return sum(t[key][0] for t in tallies) / total if total else 0.0


def frame_scores(answer, label_text):
    """Reward, per-term scores and decision tallies for one answer."""
    label = read_scene(label_text)
    pred = read_scene(answer)
    s = scores(pred, label) if pred else None
    return {'valid': pred is not None, 'reward': combine(s) if s else R_INVALID, 'scores': s, **tally(pred, label)}


def summarize(frames):
    valid = [f['scores'] for f in frames if f['valid']]

    def mean_score(key):
        return sum(s[key] for s in valid) / len(valid) if valid else 0.0

    return {'frames': len(frames), 'valid_json': len(valid) / len(frames),
            'reward': sum(f['reward'] for f in frames) / len(frames),
            'decision_acc': ratio(frames, 'decisions'), 'unsafe_go': ratio(frames, 'unsafe_go'),
            'field_acc': mean_score('field_acc'), 'iou': mean_score('iou'), 'brier': mean_score('brier')}


def run(opts):
    from vllm import LLM, SamplingParams

    rows = load_val(opts.data, opts.image_root, opts.limit)
    prompt = PROMPT if opts.prompt == 'train' else ZERO_SHOT_PROMPT
    llm = LLM(model=opts.model, max_model_len=8192, limit_mm_per_prompt={'image': 1},
              gpu_memory_utilization=opts.gpu_memory, trust_remote_code=True)
    conversations = [[{'role': 'user', 'content': [
        image_part(r['image']), {'type': 'text', 'text': prompt}]}] for r in rows]
    start = time.time()
    outputs = llm.chat(conversations, SamplingParams(temperature=0.0, max_tokens=768))
    seconds = time.time() - start
    frames = []
    for r, output in zip(rows, outputs):
        answer = output.outputs[0].text
        frames.append({**frame_scores(answer, r['label']), 'camera': r['camera'], 'bucket': r['bucket'],
                       'image': str(r['image']), 'answer': answer})
    report = {'model': opts.model, 'prompt': opts.prompt, 'seconds_per_frame_batched': seconds / len(rows),
              'all': summarize(frames)}
    for key in ('camera', 'bucket'):
        for value in sorted({f[key] for f in frames}):
            report[f'{key}:{value}'] = summarize([f for f in frames if f[key] == value])
    out = Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)
    slug = opts.name or opts.model.replace('/', '__')
    (out / f'{slug}.json').write_text(json.dumps(report, indent=1))
    with open(out / f'{slug}.answers.jsonl', 'w') as f:
        for frame in frames:
            f.write(json.dumps(frame) + '\n')
    print(json.dumps(report['all'], indent=1))


COLUMNS = ('reward', 'decision_acc', 'unsafe_go', 'valid_json', 'field_acc', 'iou', 'brier')


def report(out):
    reports = [json.loads(p.read_text()) for p in Path(out).glob('*.json')]
    reports.sort(key=lambda r: -r['all']['reward'])
    cameras = sorted({k for r in reports for k in r if k.startswith('camera:')})
    head = ['model', 'prompt', *COLUMNS, *(f'{c} dec' for c in cameras), 's/frame']
    lines = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for r in reports:
        cells = [r['model'], r['prompt'], *(f"{r['all'][c]:.3f}" for c in COLUMNS),
                 *(f"{r[c]['decision_acc']:.3f}" if c in r else '' for c in cameras),
                 f"{r['seconds_per_frame_batched']:.2f}"]
        lines.append('| ' + ' | '.join(cells) + ' |')
    print('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('run')
    p.add_argument('--model', required=True)
    p.add_argument('--name', help='output file name; defaults to the model id')
    p.add_argument('--data', required=True, help='vlm_labels.py export directory')
    p.add_argument('--image-root', required=True)
    p.add_argument('--out', default='outputs/bakeoff')
    p.add_argument('--prompt', choices=('zeroshot', 'train'), default='zeroshot')
    p.add_argument('--limit', type=int)
    p.add_argument('--gpu-memory', type=float, default=0.75)
    p = sub.add_parser('report')
    p.add_argument('out')
    opts = parser.parse_args()
    run(opts) if opts.cmd == 'run' else report(opts.out)


if __name__ == '__main__':
    main()
