#!/usr/bin/env python3
"""Real-bar photo check: which bottles does each VLM wrongly claim, which does it miss, and is Ballantine's taken for a served drink?

    python real_photos.py run --model Qwen/Qwen3-VL-8B-Instruct --prompt real --photos photos --out outputs/real
    python real_photos.py run --model outputs/sft-r1/merged --name sft-r1 --prompt sim-train ...
    python real_photos.py report outputs/real --photos photos

`real` is prelabel_real's six-drink prompt; `sim` is the whiskey / cola / beer
scene-checker prompt (`sim-train`: the one a fine-tuned model was trained on).
A bottle claimed but not on the bar is the worst error: the robot would go
for a drink it cannot serve.
"""
import argparse
import base64
import io
import json
from collections import Counter
from pathlib import Path

from PIL import Image, ImageOps

from compare_vlms import ZERO_SHOT_PROMPT
from prelabel_real import BOTTLES, REAL_BAR_PROMPT, canonical_label
from vlm_labels import PROMPT, iou_boxes
from vlm_reward import read_scene

VOCAB = {'real': BOTTLES, 'sim': ('whiskey', 'cola', 'beer')}
PROMPTS = {'real': REAL_BAR_PROMPT, 'sim': ZERO_SHOT_PROMPT, 'sim-train': PROMPT}
# Hand-labelled from the six photos (see photos/). Served bottles shown:
# Jack Daniel's (whiskey), Zubrowka (vodka), Jagermeister (liqueur); Heineken,
# Tenjaku and Frontera are in none. Ballantine's is a distractor; its box, in
# 0-1000 coordinates, is only the neck in the last photo. The clear bottle
# with a blue label and a pourer is unidentified and left out.
TRUTH = {
    'PXL_20260926_184125542.RAW-01.jpg': {'present': {'whiskey', 'vodka', 'liqueur'}, 'unsure': set(),
                                          'ballantines': [319, 561, 456, 967]},
    'PXL_20260926_184127833.RAW-01.jpg': {'present': {'whiskey', 'vodka', 'liqueur'}, 'unsure': set(),
                                          'ballantines': [366, 611, 476, 963]},
    'PXL_20260926_184130966.RAW-01.jpg': {'present': {'whiskey', 'vodka', 'liqueur'}, 'unsure': set(),
                                          'ballantines': [406, 606, 506, 1000]},
    'PXL_20260926_184132730.RAW-01.jpg': {'present': {'whiskey', 'vodka', 'liqueur'}, 'unsure': set(),
                                          'ballantines': [479, 618, 573, 1000]},
    'PXL_20260926_184148139.RAW-01.jpg': {'present': {'whiskey', 'vodka', 'liqueur'}, 'unsure': set(),
                                          'ballantines': [298, 392, 378, 648]},
    # Jack Daniel's may be the dark label behind the clear bottle here.
    'PXL_20260926_184149856.RAW-01.jpg': {'present': {'vodka', 'liqueur'}, 'unsure': {'whiskey'},
                                          'ballantines': [423, 379, 465, 490]},
}
BALLANTINES_IOU = 0.5
MAX_SIDE = 1600
# These answer bboxes in pixels of the image they were sent, not 0-1000.
PIXEL_BOX_MODELS = ('Qwen2.5-VL',)


def sent_image(path):
    image = ImageOps.exif_transpose(Image.open(path)).convert('RGB')
    image.thumbnail((MAX_SIDE, MAX_SIDE))
    return image


def photo_part(path):
    image = sent_image(path)
    buf = io.BytesIO()
    image.save(buf, format='JPEG', quality=90)
    return {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode()}}


def claims(answer, vocab):
    """{bottle name: bbox} the answer says are visible, or None if it is unusable."""
    if vocab == 'real':
        label = canonical_label(answer)
        return None if label is None else {b['name']: b['bbox'] for b in label['bottles'] if b['visible']}
    scene = read_scene(answer)
    if scene is None:
        return None
    return {n: box for n, (visible, _, box) in scene['bottles'].items() if visible and n in VOCAB['sim']}


def to_1000(box, size):
    w, h = size
    return [box[0] * 1000 / w, box[1] * 1000 / h, box[2] * 1000 / w, box[3] * 1000 / h]


def score(answer, photo, vocab, pixel_size=None):
    """Bottles wrongly claimed, bottles missed, and served names put on Ballantine's, for one photo.

    `pixel_size` is the (width, height) sent, for a model that answers boxes in pixels.
    """
    truth = TRUTH[photo]
    said = claims(answer, vocab)
    if said is None:
        return None
    if pixel_size:
        said = {n: box and to_1000(box, pixel_size) for n, box in said.items()}
    absent = set(VOCAB[vocab]) - truth['present'] - truth['unsure']
    return {'wrong': sorted(set(said) & absent),
            'missed': sorted((truth['present'] & set(VOCAB[vocab])) - set(said)),
            'ballantines_as': sorted(n for n, box in said.items()
                                     if box and iou_boxes(box, truth['ballantines']) >= BALLANTINES_IOU)}


def run(opts):
    from vllm import LLM, SamplingParams

    photos = [Path(opts.photos) / name for name in TRUTH]
    llm = LLM(model=opts.model, max_model_len=16384, limit_mm_per_prompt={'image': 1},
              gpu_memory_utilization=opts.gpu_memory, trust_remote_code=True)
    outputs = llm.chat([[{'role': 'user', 'content': [photo_part(p), {'type': 'text', 'text': PROMPTS[opts.prompt]}]}]
                        for p in photos], SamplingParams(temperature=0.0, max_tokens=1024))
    out = Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)
    name = opts.name or opts.model.replace('/', '__')
    (out / f"{name}.{opts.prompt.split('-')[0]}.json").write_text(json.dumps(
        {'model': opts.name or opts.model, 'prompt': opts.prompt,
         'rows': [{'photo': p.name, 'answer': o.outputs[0].text} for p, o in zip(photos, outputs)]}, indent=1))


def report(out, photos):
    lines = ['| model | prompt | bottles wrongly claimed | missed | Ballantine\'s taken as | unusable |',
             '|---|---|---|---|---|---|']
    ranked = []
    for path in sorted(Path(out).glob('*.*.json')):
        r = json.loads(path.read_text())
        vocab = path.name.rsplit('.', 2)[1]
        pixels = any(m in r['model'] for m in PIXEL_BOX_MODELS)
        scores = [score(row['answer'], row['photo'], vocab,
                        sent_image(Path(photos) / row['photo']).size if pixels else None)
                  for row in r['rows']]
        valid = [s for s in scores if s]
        wrong = Counter(n for s in valid for n in s['wrong'])
        missed = sum(len(s['missed']) for s in valid)
        taken = Counter(n for s in valid for n in s['ballantines_as'])
        cells = [r['model'].split('/')[-1], r['prompt'],
                 f"{sum(wrong.values())} ({', '.join(f'{n}x{c}' for n, c in wrong.most_common()) or '-'})",
                 str(missed), ', '.join(f'{n}x{c}' for n, c in taken.most_common()) or 'no',
                 str(len(scores) - len(valid))]
        ranked.append((vocab != 'real', sum(wrong.values()) + 10 * (len(scores) - len(valid)), missed, cells))
    for *_, cells in sorted(ranked):
        lines.append('| ' + ' | '.join(cells) + ' |')
    print('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('run')
    p.add_argument('--model', required=True)
    p.add_argument('--name')
    p.add_argument('--photos', required=True)
    p.add_argument('--out', default='outputs/real')
    p.add_argument('--prompt', choices=tuple(PROMPTS), default='real')
    p.add_argument('--gpu-memory', type=float, default=0.6)
    p = sub.add_parser('report')
    p.add_argument('out')
    p.add_argument('--photos', required=True, help='to size pixel boxes (PIXEL_BOX_MODELS)')
    opts = parser.parse_args()
    run(opts) if opts.cmd == 'run' else report(opts.out, opts.photos)


if __name__ == '__main__':
    main()
