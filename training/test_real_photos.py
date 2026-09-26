import json

from real_photos import TRUTH, score

PHOTO = 'PXL_20260926_184125542.RAW-01.jpg'
UNSURE = 'PXL_20260926_184149856.RAW-01.jpg'


def _real(**bottles):
    return json.dumps({'bottles': [{'name': n, 'visible': True, 'confidence': 0.9, 'bbox': box}
                                   for n, box in bottles.items()],
                       'glasses': [], 'in_gripper': {'value': None, 'confidence': 0.9},
                       'obstruction': {'value': False, 'confidence': 0.9}})


def test_claiming_an_absent_drink_is_wrong_and_unclaimed_present_ones_are_missed():
    s = score(_real(whiskey=[0, 0, 10, 10], beer=[20, 20, 30, 30]), PHOTO, 'real')
    assert s['wrong'] == ['beer']
    assert s['missed'] == ['liqueur', 'vodka']


def test_served_name_on_ballantines_box_is_caught():
    s = score(_real(whiskey=TRUTH[PHOTO]['ballantines']), PHOTO, 'real')
    assert s['ballantines_as'] == ['whiskey'] and s['wrong'] == []


def test_unsure_bottle_is_neither_wrong_nor_missed():
    assert score(_real(vodka=[0, 0, 5, 5], liqueur=[5, 5, 9, 9]), UNSURE, 'real')['missed'] == []
    assert score(_real(whiskey=[0, 0, 5, 5]), UNSURE, 'real')['wrong'] == []


def test_sim_prompt_cola_is_never_on_the_real_bar():
    answer = json.dumps({'bottles': [{'name': 'whiskey', 'visible': True, 'confidence': 0.9, 'bbox': [0, 0, 5, 5]},
                                     {'name': 'cola', 'visible': True, 'confidence': 0.9, 'bbox': [6, 6, 9, 9]}],
                         'glasses': [], 'in_gripper': {'value': None, 'confidence': 0.9},
                         'obstruction': {'value': False, 'confidence': 0.9}})
    assert score(answer, PHOTO, 'sim') == {'wrong': ['cola'], 'missed': [], 'ballantines_as': []}


def test_unusable_answer_scores_none():
    assert score('I see bottles', PHOTO, 'real') is None


def test_pixel_boxes_are_scaled_to_1000_before_the_ballantines_check():
    x0, y0, x1, y1 = TRUTH[PHOTO]['ballantines']
    size = (1600, 1204)
    pixels = [x0 * 1.6, y0 * 1.204, x1 * 1.6, y1 * 1.204]
    assert score(_real(whiskey=pixels), PHOTO, 'real')['ballantines_as'] == []
    assert score(_real(whiskey=pixels), PHOTO, 'real', size)['ballantines_as'] == ['whiskey']
