import copy
import json

from vlm_reward import R_INVALID, R_MAX, decide, read_scene, scene_reward


def _label():
    return {
        'bottles': [
            {'name': 'whiskey', 'visible': True, 'confidence': 1.0, 'bbox': [100, 100, 200, 600]},
            {'name': 'cola', 'visible': False, 'confidence': 1.0, 'bbox': None},
        ],
        'glasses': [{'name': 'glass', 'visible': True, 'confidence': 1.0, 'bbox': [700, 500, 800, 900]}],
        'in_gripper': {'value': None, 'confidence': 1.0},
        'obstruction': {'value': False, 'confidence': 1.0},
    }


def _reward(answer, label=None):
    return scene_reward(json.dumps(answer), json.dumps(label or _label()))


def test_perfect_answer_gets_the_maximum():
    assert abs(_reward(_label()) - R_MAX) < 1e-9


def test_fenced_answer_is_still_read():
    assert abs(scene_reward('```json\n' + json.dumps(_label()) + '\n```', json.dumps(_label())) - R_MAX) < 1e-9


def test_invalid_answers_get_the_floor():
    label = json.dumps(_label())
    assert scene_reward('the bar has a bottle', label) == R_INVALID
    missing = _label()
    del missing['obstruction']
    assert _reward(missing) == R_INVALID
    scalar = _label()
    scalar['in_gripper'] = None
    assert _reward(scalar) == R_INVALID


def test_worst_valid_answer_beats_invalid_json():
    worst = _label()
    for obj in worst['bottles'] + worst['glasses']:
        obj['visible'] = not obj['visible']
        obj['bbox'] = None
    worst['in_gripper']['value'] = 'whiskey'
    worst['obstruction']['value'] = True
    assert _reward(worst) > R_INVALID


def test_missed_obstruction_costs_more_than_a_ten_percent_box_shift():
    label = _label()
    label['obstruction']['value'] = True
    missed = copy.deepcopy(label)
    missed['obstruction']['value'] = False
    shifted = copy.deepcopy(label)
    shifted['bottles'][0]['bbox'] = [110, 150, 210, 650]
    assert _reward(missed, label) < _reward(shifted, label)


def test_confidently_wrong_scores_below_unconfidently_wrong():
    confident = _label()
    confident['bottles'][1].update(visible=True, bbox=[300, 100, 400, 600])
    unsure = copy.deepcopy(confident)
    unsure['bottles'][1]['confidence'] = 0.5
    assert _reward(confident) < _reward(unsure)


def test_decide_gates_on_visibility_grip_glass_and_obstruction():
    scene = read_scene(json.dumps(_label()))
    assert decide(scene) == {('pick', 'whiskey'): 'go', ('pour', 'whiskey'): 'go',
                             ('pick', 'cola'): 'no', ('pour', 'cola'): 'no'}
    held = _label()
    held['in_gripper']['value'] = 'whiskey'
    assert decide(read_scene(json.dumps(held)))[('pick', 'whiskey')] == 'no'
    blocked = _label()
    blocked['obstruction']['value'] = True
    assert set(decide(read_scene(json.dumps(blocked))).values()) == {'no'}
    no_glass = _label()
    no_glass['glasses'][0].update(visible=False, bbox=None)
    decisions = decide(read_scene(json.dumps(no_glass)))
    assert decisions[('pick', 'whiskey')] == 'go' and decisions[('pour', 'whiskey')] == 'no'


def test_sliver_makes_the_decision_unsure():
    sliver = _label()
    sliver['bottles'][0].update(visible=False, confidence=0.51, bbox=None)
    assert decide(read_scene(json.dumps(sliver)))[('pick', 'whiskey')] == 'unsure'


def test_omitted_bottle_is_judged_as_absent():
    omitted = _label()
    omitted['bottles'] = omitted['bottles'][1:]
    assert _reward(omitted) < R_MAX
