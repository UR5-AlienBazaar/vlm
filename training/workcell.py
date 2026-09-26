"""Ground truth and scoring for workcell twin scenes (bartender_robot_sim capture_workcell_scenes.py).

A scene directory holds scene.json, 0000.json (settled object poses, arm) and
0000_cam_rgb.png / 0000_cam_labels.png. The VLM gets the image and an order
("pick whiskey") and answers what it sees plus the next action; this module
says what the right answer is and how close the model came.
"""
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image

from vlm_labels import LABEL_IDS, bbox, iou_boxes

BOTTLES = ('whiskey', 'vodka', 'liqueur', 'beer', 'gin', 'wine')  # the real bar's drinks
# A distractor this close to the straight line from the arm base to a bottle
# is in the gripper's way: its ~5cm radius plus the open fingers' clearance.
BLOCK_RADIUS = 0.07
ACTIONS = ('pick', 'clear_path', 'report_fallen', 'not_found')
FIELDS = ('visible', 'upright', 'blocked', 'iou', 'action')
# Selection score. The action is what the robot acts on, so it dominates.
WEIGHTS = {'action': 0.4, 'visible': 0.2, 'upright': 0.15, 'blocked': 0.15, 'iou': 0.1}


def _segment_distance(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy or 1.0)))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def blocked(bottle_xy, arm_base, objects):
    """Whether a distractor stands on the arm's straight path to the bottle (settled poses)."""
    return any(o.get('on_table') and 'xy' in o and _segment_distance(o['xy'], arm_base, bottle_xy) < BLOCK_RADIUS
               for name, o in objects.items() if name.startswith('distractor'))


def order_for(scene_name):
    """The bottle this scene is asked for; stable per scene and includes absent ones."""
    digest = hashlib.sha1(scene_name.encode(), usedforsecurity=False).hexdigest()
    return BOTTLES[int(digest, 16) % len(BOTTLES)]


def next_action(bottles, order):
    """What the robot should do to serve `order`, judged from what the camera can show.

    A bottle the camera cannot see is 'not_found' whether it is missing or
    hidden: the picture cannot tell those apart, and either way the robot must
    not reach for it.
    """
    b = bottles[order]
    if not b['visible']:
        action = 'not_found'
    elif not b['upright']:
        action = 'report_fallen'
    elif b['blocked']:
        action = 'clear_path'
    else:
        action = 'pick'
    return {'action': action, 'target': order}


def scene_truth(scene_dir):
    """Per-bottle truth, the order, the right next action and eval buckets for one scene."""
    scene_dir = Path(scene_dir)
    meta = json.loads((scene_dir / 'scene.json').read_text())
    frame = json.loads((scene_dir / '0000.json').read_text())
    labels = np.asarray(Image.open(scene_dir / '0000_cam_labels.png'))
    objects = frame['objects']
    arm_base = meta.get('arm_base', (0.35, 0.35))
    bottles = {}
    for name in BOTTLES:
        obj = objects.get(name, {'on_table': False})
        on_table = bool(obj.get('on_table'))
        box = bbox(labels, LABEL_IDS[name])
        bottles[name] = {
            'visible': box is not None,
            'bbox': box,
            'on_table': on_table,
            'upright': bool(obj.get('upright', True)) if on_table else None,
            'blocked': bool(on_table and 'xy' in obj and blocked(obj['xy'], arm_base, objects)),
        }
    order = order_for(scene_dir.name)
    action = next_action(bottles, order)
    target = bottles[order]
    n_distractors = sum(1 for k, o in objects.items() if k.startswith('distractor') and o.get('on_table'))
    buckets = {
        'camera': meta.get('camera', {}).get('family', 'unknown'),
        'arm': frame.get('arm', {}).get('kind', 'unknown'),
        'distractors': str(n_distractors),
        'action': action['action'],
        'hidden': str(target['on_table'] and not target['visible']).lower(),
    }
    return {'scene': scene_dir.name, 'order': order, 'bottles': bottles,
            'next_action': action, 'buckets': buckets}


def gold_answer(truth):
    """The answer a perfect model gives, in the schema the prompt asks for."""
    return {
        'bottles': [{'name': n, 'visible': b['visible'],
                     'upright': b['upright'] if b['visible'] else None,
                     'blocked': b['blocked'] if b['visible'] else None,
                     'bbox': b['bbox']} for n, b in truth['bottles'].items()],
        'next_action': truth['next_action'],
    }


def read_answer(text):
    """The model's JSON answer as {bottles: {name: dict}, action: str}, or None if unusable."""
    match = re.search(r'\{.*\}', text, re.DOTALL)
    if not match:
        return None
    try:
        answer = json.loads(match.group(0))
        bottles = {str(b['name']).lower(): b for b in answer['bottles'] if isinstance(b, dict)}
        action = str(answer['next_action']['action'])
    except (json.JSONDecodeError, KeyError, TypeError, AttributeError):
        return None
    return {'bottles': bottles, 'action': action}


def _box(value):
    if isinstance(value, list) and len(value) == 4 and all(isinstance(v, (int, float)) for v in value):
        return [float(v) for v in value]
    return None


def _mean(xs):
    return sum(xs) / len(xs) if xs else 1.0


def score(text, truth):
    """Per-field scores in 0-1 for one answer; all zero when the JSON is unusable.

    upright, blocked and iou are judged only on bottles that are really visible:
    the picture holds nothing to judge them by otherwise.
    """
    answer = read_answer(text)
    if answer is None:
        return {'valid': 0.0, **{f: 0.0 for f in FIELDS}, 'total': 0.0}
    vis, upright, block, ious = [], [], [], []
    for name, t in truth['bottles'].items():
        p = answer['bottles'].get(name, {})
        p_visible = p.get('visible') is True
        vis.append(p_visible == t['visible'])
        if t['visible']:
            upright.append(p.get('upright') is t['upright'])
            block.append(bool(p.get('blocked')) == t['blocked'])
            box = _box(p.get('bbox'))
            ious.append(iou_boxes(box, t['bbox']) if box and p_visible else 0.0)
    s = {'valid': 1.0, 'visible': _mean(vis), 'upright': _mean(upright), 'blocked': _mean(block),
         'iou': _mean(ious), 'action': float(answer['action'] == truth['next_action']['action'])}
    s['total'] = sum(w * s[k] for k, w in WEIGHTS.items())
    return s
