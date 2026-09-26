"""GRPO reward for the scene-checker: field accuracy, box overlap, robot decisions, calibration.

Pure Python so it can be tested without the training stack. Both the model's
answer and the label are the JSON that vlm_labels.scene_label produces.
"""
import json
import re

# ponytail: hand-picked; retune after the no-decision / no-Brier ablations.
W_FIELD = 1.0
W_IOU = 0.5
W_DECISION = 1.0
W_CAL = 0.5
R_MAX = W_FIELD + W_IOU + W_DECISION
# Below the worst valid answer (everything wrong at confidence 1 scores
# -W_CAL), so broken JSON is never a way to dodge the calibration penalty.
R_INVALID = -(W_CAL + 1.0)
# A gate field reported below this confidence makes the decision 'unsure':
# the planner should take another look rather than act.
SURE = 0.7
VERBS = ('pick', 'pour')
_ABSENT = (False, 1.0, None)


def _confidence(value):
    c = float(value)
    if not 0.0 <= c <= 1.0:
        raise ValueError(f'confidence {c} outside 0-1')
    return c


def _box(value):
    if isinstance(value, list) and len(value) == 4 and all(isinstance(v, (int, float)) for v in value):
        return [float(v) for v in value]
    return None


def _flag(field):
    return field['value'], _confidence(field['confidence'])


def _objects(items):
    return {str(o['name']): (bool(o['visible']), _confidence(o['confidence']), _box(o.get('bbox')))
            for o in items}


def read_scene(text):
    """Canonical form of a scene answer, or None if it is not valid scene JSON.

    Tolerates a ```json fence or chatter around the object, as the serving side does.
    """
    match = re.search(r'\{.*\}', text, re.DOTALL)
    if not match:
        return None
    try:
        scene = json.loads(match.group(0))
        return {'bottles': _objects(scene['bottles']), 'glasses': _objects(scene['glasses']),
                'in_gripper': _flag(scene['in_gripper']), 'obstruction': _flag(scene['obstruction'])}
    except (json.JSONDecodeError, KeyError, TypeError, ValueError, AttributeError):
        return None


def _gate(checks):
    """'no' if any check is surely failed, else 'unsure' if any is unsure, else 'go'."""
    if any(not ok and conf >= SURE for ok, conf in checks):
        return 'no'
    if any(conf < SURE for _, conf in checks):
        return 'unsure'
    return 'go'


def decide(scene, bottles=None):
    """Perception gate per (verb, bottle): 'go', 'no' or 'unsure'.

    A bottle can be picked or poured when it is visible and not already held and
    nothing obstructs the scene; pouring also needs a visible glass. Obstruction
    is one flag for the whole scene, so any obstruction blocks every verb:
    conservative, and all the label can say. `bottles` fixes which bottles are
    decided, so a model that omits one is judged as calling it absent.
    """
    held, held_conf = scene['in_gripper']
    blocked, blocked_conf = scene['obstruction']
    glasses = scene['glasses'].values()
    glass_visible = any(v for v, _, _ in glasses)
    glass_conf = max((c for v, c, _ in glasses if v == glass_visible), default=1.0)
    decisions = {}
    for name in bottles if bottles is not None else scene['bottles']:
        visible, conf, _ = scene['bottles'].get(name, _ABSENT)
        pick = [(visible, conf), (held != name, held_conf), (not blocked, blocked_conf)]
        decisions[('pick', name)] = _gate(pick)
        decisions[('pour', name)] = _gate(pick + [(glass_visible, glass_conf)])
    return decisions


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def scores(pred, label):
    """Per-term scores of a canonical prediction against a canonical label."""
    pairs = []
    ious = []
    for group in ('bottles', 'glasses'):
        for name, (visible, _, box) in label[group].items():
            p_visible, p_conf, p_box = pred[group].get(name, _ABSENT)
            pairs.append((p_conf, p_visible == visible))
            if visible:
                ious.append(iou(p_box, box) if p_box and p_visible else 0.0)
    for flag in ('in_gripper', 'obstruction'):
        value, conf = pred[flag]
        pairs.append((conf, value == label[flag][0]))
    truth = decide(label)
    guess = decide(pred, bottles=list(label['bottles']))
    return {
        'field_acc': sum(ok for _, ok in pairs) / len(pairs),
        'iou': sum(ious) / len(ious) if ious else 1.0,
        'decision': sum(guess[k] == v for k, v in truth.items()) / len(truth) if truth else 1.0,
        'brier': sum((conf - ok) ** 2 for conf, ok in pairs) / len(pairs),
    }


def combine(s):
    return W_FIELD * s['field_acc'] + W_IOU * s['iou'] + W_DECISION * s['decision'] - W_CAL * s['brier']


def scene_reward(text, label_text):
    """Reward for one completion; `label_text` is the label's JSON."""
    pred = read_scene(text)
    if pred is None:
        return R_INVALID
    return combine(scores(pred, read_scene(label_text)))
