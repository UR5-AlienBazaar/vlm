import json

import numpy as np
from PIL import Image

from workcell import WEIGHTS, blocked, gold_answer, next_action, order_for, scene_truth, score


def make_scene(root, name='w900_00000', objects=None, painted=('whiskey', 'vodka'), camera='front'):
    """A synthetic workcell scene in the capture layout: whiskey and vodka painted, beer absent."""
    scene = root / name
    scene.mkdir(parents=True)
    labels = np.zeros((400, 640), np.uint8)
    boxes = {'whiskey': (100, 160, 200, 260), 'vodka': (300, 160, 360, 220), 'beer': (450, 160, 500, 220)}
    ids = {'whiskey': 1, 'vodka': 4, 'beer': 3}
    for bottle in painted:
        y0, x0, y1, x1 = boxes[bottle]
        labels[y0:y1, x0:x1] = ids[bottle]
    Image.fromarray(labels).save(scene / '0000_cam_labels.png')
    Image.fromarray(np.zeros((400, 640, 3), np.uint8)).save(scene / '0000_cam_rgb.png')
    objects = objects or {
        'whiskey': {'on_table': True, 'xy': [1.0, 0.35], 'upright': True, 'tilt_deg': 0.0},
        'vodka': {'on_table': True, 'xy': [0.9, 0.6], 'upright': False, 'tilt_deg': 88.0},
    }
    (scene / '0000.json').write_text(json.dumps({'in_gripper': None, 'objects': objects,
                                                 'arm': {'kind': 'home', 'joints': [0] * 6, 'reached': True}}))
    (scene / 'scene.json').write_text(json.dumps({
        'bottles': ['whiskey', 'vodka', 'liqueur', 'beer', 'gin', 'wine'], 'glasses': [], 'kind': 'workcell',
        'camera': {'family': camera}, 'arm_base': [0.35, 0.35]}))
    return scene


def test_blocked_means_on_the_line_from_arm_to_bottle():
    objects = {'distractor0': {'on_table': True, 'xy': [0.8, 0.36]}}
    assert blocked((1.0, 0.35), (0.35, 0.35), objects)
    objects['distractor0']['xy'] = [0.8, 0.6]
    assert not blocked((1.0, 0.35), (0.35, 0.35), objects)
    objects['distractor0']['xy'] = [1.2, 0.35]  # behind the bottle, not in the way
    assert not blocked((1.0, 0.35), (0.35, 0.35), objects)


def test_next_action_priorities():
    base = {'visible': True, 'upright': True, 'blocked': False}
    assert next_action({'whiskey': base}, 'whiskey')['action'] == 'pick'
    assert next_action({'whiskey': {**base, 'blocked': True}}, 'whiskey')['action'] == 'clear_path'
    assert next_action({'whiskey': {**base, 'upright': False, 'blocked': True}}, 'whiskey')['action'] == 'report_fallen'
    assert next_action({'whiskey': {**base, 'visible': False}}, 'whiskey')['action'] == 'not_found'


def test_scene_truth_reads_the_capture_layout(tmp_path):
    truth = scene_truth(make_scene(tmp_path))
    b = truth['bottles']
    assert b['whiskey']['visible'] and b['whiskey']['upright'] and b['whiskey']['bbox'] == [250, 250, 406, 500]
    assert b['vodka']['visible'] and b['vodka']['upright'] is False
    assert not b['gin']['visible'] and b['gin']['on_table'] is False
    assert not b['beer']['visible'] and b['beer']['upright'] is None
    assert truth['order'] == order_for('w900_00000')
    assert truth['buckets']['camera'] == 'front'


def test_gold_answer_scores_full_marks_and_garbage_scores_zero(tmp_path):
    truth = scene_truth(make_scene(tmp_path))
    perfect = score(json.dumps(gold_answer(truth)), truth)
    assert perfect['valid'] == 1.0 and abs(perfect['total'] - sum(WEIGHTS.values())) < 1e-9
    assert score('I see some bottles', truth)['total'] == 0.0


def test_a_bare_string_action_is_read(tmp_path):
    truth = scene_truth(make_scene(tmp_path))
    answer = gold_answer(truth)
    answer['next_action'] = truth['next_action']['action']
    assert score(json.dumps(answer), truth)['action'] == 1.0


def test_wrong_action_and_missed_fall_cost_points(tmp_path):
    truth = scene_truth(make_scene(tmp_path))
    answer = gold_answer(truth)
    answer['next_action'] = {'action': 'dance', 'target': truth['order']}
    answer['bottles'][1]['upright'] = True
    s = score(json.dumps(answer), truth)
    assert s['action'] == 0.0 and s['upright'] == 0.5 and s['total'] < sum(WEIGHTS.values()) - 0.4
