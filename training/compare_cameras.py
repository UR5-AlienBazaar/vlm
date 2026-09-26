#!/usr/bin/env python3
"""Camera experiment: does the scene-checker decide better from one camera, all cameras fused, or all in one prompt?

    python compare_cameras.py run --model Qwen/Qwen3-VL-8B-Instruct --data data/vlm --image-root data/vlm_raw
    python compare_cameras.py score --bakeoff outputs/bakeoff --data data/vlm --image-root data/vlm_raw

`run` asks a model about each moment with its overhead, stand and wrist images
in one prompt. `score` judges every model's decisions five ways against one
scene truth fused from the three camera labels: each camera alone and fused
(from compare_vlms.py's single-view answers), and the three-image run. Each
model's scores are one experiment in <out>/results.db.
"""
import argparse
import json
from pathlib import Path

from compare_vlms import ZERO_SHOT_PROMPT, image_part, load_val, ratio, tally
from results import connect, finish_experiment, log_metrics, start_experiment
from vlm_reward import read_scene

CAMERAS = ('overhead', 'stand', 'wrist')
WAYS = (*CAMERAS, 'fused', 'three_image')
MULTIVIEW_PROMPT = (
    'The three images show the same moment from the overhead camera, the stand camera and the wrist '
    'camera on the gripper, in that order. Describe the one scene they show together: an object is '
    'visible if any image shows it, and bbox is in the first (overhead) image, or null.\n' + ZERO_SHOT_PROMPT)


def moment(image):
    """'.../s101_00006/0000_overhead_rgb.png' -> '.../s101_00006/0000', shared by the views of one moment."""
    return str(image).rsplit('_', 2)[0]


def load_moments(data, image_root, limit=None):
    moments = {}
    for r in load_val(data, image_root):
        moments.setdefault(moment(r['image']), {})[r['camera']] = r
    return [m for _, m in sorted(moments.items()) if set(m) == set(CAMERAS)][:limit]


def fuse(scenes):
    """One canonical scene from several views, or None if no view gave a usable answer.

    An object any view sees is there, and an obstruction in any view blocks: the
    labels are per view, so this is what the arm can know from all cameras together.
    """
    scenes = [s for s in scenes if s]
    if not scenes:
        return None
    fused = {}
    for group in ('bottles', 'glasses'):
        fused[group] = {}
        for name in {n for s in scenes for n in s[group]}:
            views = [s[group][name] for s in scenes if name in s[group]]
            visible = any(v for v, _, _ in views)
            fused[group][name] = (visible, max(c for v, c, _ in views if v == visible), None)
    fused['in_gripper'] = max((s['in_gripper'] for s in scenes), key=lambda f: f[1])
    blocked = any(s['obstruction'][0] for s in scenes)
    fused['obstruction'] = (blocked, max(c for v, c in (s['obstruction'] for s in scenes) if v == blocked))
    return fused


def run(opts):
    from vllm import LLM, SamplingParams

    moments = load_moments(opts.data, opts.image_root, opts.limit)
    # 16k fits three images even for models that tile each one into many crops.
    llm = LLM(model=opts.model, max_model_len=16384, limit_mm_per_prompt={'image': len(CAMERAS)},
              gpu_memory_utilization=opts.gpu_memory, trust_remote_code=True)
    outputs = llm.chat([[{'role': 'user', 'content': [
        *(image_part(m[c]['image']) for c in CAMERAS), {'type': 'text', 'text': MULTIVIEW_PROMPT}]}]
        for m in moments], SamplingParams(temperature=0.0, max_tokens=768))
    out = Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / f"{opts.model.replace('/', '__')}.multiview.jsonl", 'w') as f:
        for m, output in zip(moments, outputs):
            f.write(json.dumps({'moment': moment(m['overhead']['image']), 'answer': output.outputs[0].text}) + '\n')


def model_tallies(single_answers, multiview_answers, truth):
    """Decision tallies per way of using the cameras, for one model."""
    preds = {}
    for line in single_answers.read_text().splitlines():
        frame = json.loads(line)
        preds.setdefault(moment(frame['image']), {})[frame['camera']] = read_scene(frame['answer'])
    tallies = {w: [] for w in WAYS}
    for key, label in truth.items():
        views = preds.get(key, {})
        for c in CAMERAS:
            tallies[c].append(tally(views.get(c), label))
        tallies['fused'].append(tally(fuse(list(views.values())), label))
    if multiview_answers.exists():
        for line in multiview_answers.read_text().splitlines():
            row = json.loads(line)
            tallies['three_image'].append(tally(read_scene(row['answer']), truth[row['moment']]))
    return tallies


def score(opts):
    truth = {moment(m['overhead']['image']): fuse([read_scene(m[c]['label']) for c in CAMERAS])
             for m in load_moments(opts.data, opts.image_root)}
    out = Path(opts.out)
    con = connect(out / 'results.db')
    head = ['model (decisions right / unsafe go)', *WAYS]
    lines = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for single in sorted(Path(opts.bakeoff).glob('*.answers.jsonl')):
        slug = single.name[:-len('.answers.jsonl')]
        model = slug.replace('__', '/')
        tallies = model_tallies(single, out / f'{slug}.multiview.jsonl', truth)
        metrics = {f'{w}/{name}': ratio(tallies[w], key) for w in WAYS if tallies[w]
                   for name, key in (('decision_acc', 'decisions'), ('unsafe_go', 'unsafe_go'))}
        exp = start_experiment(con, f'cameras {model}', 'eval',
                               {'experiment': 'cameras', 'model': model, 'data': opts.data,
                                'moments': len(truth), 'ways': [w for w in WAYS if tallies[w]]})
        log_metrics(con, exp, 0, metrics)
        finish_experiment(con, exp)
        cells = [f"{metrics[f'{w}/decision_acc']:.3f} / {metrics[f'{w}/unsafe_go']:.3f}"
                 if f'{w}/decision_acc' in metrics else '' for w in WAYS]
        lines.append('| ' + ' | '.join([model, *cells]) + ' |')
    print('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name in ('run', 'score'):
        p = sub.add_parser(name)
        p.add_argument('--data', required=True, help='vlm_labels.py export directory')
        p.add_argument('--image-root', required=True)
        p.add_argument('--out', default='outputs/cameras')
    p = sub.choices['run']
    p.add_argument('--model', required=True)
    p.add_argument('--limit', type=int)
    p.add_argument('--gpu-memory', type=float, default=0.75)
    sub.choices['score'].add_argument('--bakeoff', required=True, help='compare_vlms.py output directory')
    opts = parser.parse_args()
    run(opts) if opts.cmd == 'run' else score(opts)


if __name__ == '__main__':
    main()
