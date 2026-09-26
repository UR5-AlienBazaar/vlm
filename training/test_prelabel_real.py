import json

from prelabel_real import BOTTLES, canonical_label


def test_answer_becomes_all_six_bottles_with_unknowns_dropped():
    answer = '```json\n' + json.dumps({
        'bottles': [{'name': 'liqueur', 'visible': True, 'confidence': 0.8, 'bbox': [1, 2, 3, 4]},
                    {'name': "Ballantine's", 'visible': True, 'confidence': 0.9, 'bbox': [5, 6, 7, 8]},
                    {'name': 'vodka', 'visible': True, 'confidence': 0.7, 'bbox': None}],
        'glasses': [{'name': 'glass', 'visible': True, 'confidence': 0.9, 'bbox': [10, 20, 30, 40]},
                    {'name': 'glass', 'visible': False, 'confidence': 0.9, 'bbox': None}],
        'in_gripper': {'value': 'cola', 'confidence': 0.6},
        'obstruction': {'value': True, 'confidence': 0.7}}) + '\n```'

    label = canonical_label(answer)

    assert [b['name'] for b in label['bottles']] == list(BOTTLES)
    by_name = {b['name']: b for b in label['bottles']}
    assert by_name['liqueur'] == {'name': 'liqueur', 'visible': True, 'confidence': 0.8, 'bbox': [1, 2, 3, 4]}
    assert not by_name['vodka']['visible'] and by_name['vodka']['bbox'] is None
    assert not by_name['whiskey']['visible']
    assert [g['bbox'] for g in label['glasses']] == [[10, 20, 30, 40]]
    assert label['in_gripper']['value'] is None
    assert label['obstruction']['value'] is True


def test_non_json_answer_is_none():
    assert canonical_label('I see some bottles.') is None
