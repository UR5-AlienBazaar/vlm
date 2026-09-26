import json

from compare_vlms import frame_scores, summarize
from test_vlm_reward import _label


def test_blind_go_on_an_obstructed_scene_is_unsafe():
    label = _label()
    label['obstruction'] = {'value': True, 'confidence': 1.0}
    blind = _label()
    frame = frame_scores(json.dumps(blind), json.dumps(label))
    assert frame['unsafe_go'][0] > 0
    assert summarize([frame])['unsafe_go'] > 0


def test_invalid_answer_counts_against_decisions():
    frame = frame_scores('I see some bottles.', json.dumps(_label()))
    summary = summarize([frame])
    assert summary['valid_json'] == 0 and summary['decision_acc'] == 0


def test_perfect_answer_is_never_unsafe():
    summary = summarize([frame_scores(json.dumps(_label()), json.dumps(_label()))])
    assert summary['decision_acc'] == 1 and summary['unsafe_go'] == 0
