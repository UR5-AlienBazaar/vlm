#!/usr/bin/env python3
"""Pre-label real bar photos with a zero-shot VLM, for a human to correct before they become training data.

    python prelabel_real.py --photos data/real_raw --out data/real_prelabel
    # review preview/*.jpg, fix prelabel.jsonl, set "reviewed": true, then mix into train.jsonl

Records use vlm_labels' chat format and training PROMPT, with image paths
relative to --photos, so --photos is the image root for training.
"""
import argparse
import base64
import json
import re
from pathlib import Path

from vlm_labels import PROMPT, chat_record

MODEL = 'Qwen/Qwen3-VL-8B-Instruct'
BOTTLES = ('whiskey', 'vodka', 'liqueur', 'beer', 'gin', 'wine')
GLASS = 'glass'

# Only the pre-labeller sees brand descriptions; the trained model must learn
# the bottles from the labels, as with the sim data.
REAL_BAR_PROMPT = (
    'You are the camera of a robot bartender. The bar has six bottles, named exactly: '
    '"whiskey" (Jack Daniel\'s, square bottle with a black label), '
    '"vodka" (Zubrowka, clear bottle with a bison on the label), '
    '"liqueur" (Jagermeister, dark green bottle with a stag on the label), '
    '"beer" (Heineken, green bottle with a red star), '
    '"gin" (Tenjaku gin) and "wine" (Frontera white wine). '
    'Every cup or drinking glass is named "glass". Any other bottle or object is a distractor, '
    'never one of these names.\n' + PROMPT +
    '\nList all six bottles, with visible false and bbox null for ones you cannot see, and one '
    '"glass" entry per cup. Answer with the JSON object only.')


def _json_object(text):
    match = re.search(r'\{.*\}', text, re.S)
    try:
        return json.loads(match.group(0)) if match else None
    except json.JSONDecodeError:
        return None


def _entry(item, name):
    visible = bool(item.get('visible')) and item.get('bbox') is not None
    return {'name': name, 'visible': visible,
            'confidence': float(item.get('confidence', 0.5)),
            'bbox': [int(v) for v in item['bbox']] if visible else None}


def canonical_label(answer):
    """The model's answer in training-label shape: all six bottles in fixed order, unknown names dropped.

    None if the answer is not a JSON object.
    """
    raw = _json_object(answer)
    if raw is None:
        return None
    seen = {b.get('name'): b for b in raw.get('bottles', []) if b.get('name') in BOTTLES}
    bottles = [_entry(seen.get(name, {}), name) for name in BOTTLES]
    glasses = [_entry(g, GLASS) for g in raw.get('glasses', []) if g.get('bbox') is not None]
    gripper = raw.get('in_gripper') or {}
    obstruction = raw.get('obstruction') or {}
    held = gripper.get('value')
    return {'bottles': bottles, 'glasses': glasses,
            'in_gripper': {'value': held if held in BOTTLES else None,
                           'confidence': float(gripper.get('confidence', 0.5))},
            'obstruction': {'value': bool(obstruction.get('value')),
                            'confidence': float(obstruction.get('confidence', 0.5))}}


def draw_preview(image_path, label, out_path):
    from PIL import Image, ImageDraw

    image = Image.open(image_path).convert('RGB')
    draw = ImageDraw.Draw(image)
    w, h = image.size
    for item in label['bottles'] + label['glasses']:
        if item['bbox']:
            x0, y0, x1, y1 = item['bbox']
            box = [x0 * w / 1000, y0 * h / 1000, x1 * w / 1000, y1 * h / 1000]
            colour = 'cyan' if item['name'] == GLASS else 'red'
            draw.rectangle(box, outline=colour, width=4)
            draw.text((box[0] + 4, box[1] + 4), f"{item['name']} {item['confidence']:.2f}", fill=colour)
    image.save(out_path, quality=85)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--photos', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--model', default=MODEL)
    parser.add_argument('--gpu-memory', type=float, default=0.31)
    opts = parser.parse_args()

    from vllm import LLM, SamplingParams

    photos = sorted(p for p in opts.photos.rglob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    llm = LLM(model=opts.model, max_model_len=8192, limit_mm_per_prompt={'image': 1},
              gpu_memory_utilization=opts.gpu_memory, trust_remote_code=True)
    mime = {'.png': 'image/png'}
    conversations = [[{'role': 'user', 'content': [
        {'type': 'image_url', 'image_url': {'url': f"data:{mime.get(p.suffix.lower(), 'image/jpeg')};base64,"
                                                   + base64.b64encode(p.read_bytes()).decode()}},
        {'type': 'text', 'text': REAL_BAR_PROMPT}]}] for p in photos]
    outputs = llm.chat(conversations, SamplingParams(temperature=0.0, max_tokens=1024))

    preview = opts.out / 'preview'
    preview.mkdir(parents=True, exist_ok=True)
    invalid = 0
    with open(opts.out / 'prelabel.jsonl', 'w') as f:
        for photo, output in zip(photos, outputs):
            label = canonical_label(output.outputs[0].text)
            if label is None:
                invalid += 1
                print(f'invalid answer, label by hand: {photo}')
                continue
            relative = photo.relative_to(opts.photos).as_posix()
            record = chat_record(relative, label)
            record.update(camera='real', bucket='real/prelabel', reviewed=False, prelabel_model=opts.model)
            f.write(json.dumps(record) + '\n')
            draw_preview(photo, label, preview / (relative.replace('/', '__') + '.jpg'))
    print(f'{len(photos) - invalid} pre-labelled, {invalid} invalid -> {opts.out}')


if __name__ == '__main__':
    main()
