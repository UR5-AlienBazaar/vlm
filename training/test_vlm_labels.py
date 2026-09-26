import json

import numpy as np
from PIL import Image

from vlm_labels import LABEL_IDS, MIN_VISIBLE_PX, bbox, export, is_val, scene_label


def _scene(h=100, w=200):
    labels = np.zeros((h, w), np.uint8)
    labels[10:60, 20:40] = LABEL_IDS['whiskey']
    labels[50:90, 150:170] = LABEL_IDS['glass']
    return labels


def test_bbox_is_normalised_to_1000():
    assert bbox(_scene(), LABEL_IDS['whiskey']) == [100, 100, 200, 600]


def test_sliver_is_not_visible():
    labels = _scene()
    labels[10:60, 20:40] = LABEL_IDS['distractor']
    labels[10, 20:20 + MIN_VISIBLE_PX - 1] = LABEL_IDS['whiskey']
    label = scene_label(labels, ['whiskey'], [], None)
    assert label['bottles'] == [{'name': 'whiskey', 'visible': False, 'bbox': None}]


def test_second_glass_is_another_entry_not_a_new_key():
    labels = _scene()
    labels[50:90, 100:120] = LABEL_IDS['glass_b']
    label = scene_label(labels, ['whiskey'], ['glass', 'glass_b'], None)
    assert [(g['name'], g['visible']) for g in label['glasses']] == [('glass', True), ('glass_b', True)]


def test_distractor_against_a_target_is_an_obstruction():
    labels = _scene()
    assert not scene_label(labels, ['whiskey'], ['glass'], None)['obstruction']
    labels[10:60, 41:60] = LABEL_IDS['distractor']
    assert scene_label(labels, ['whiskey'], ['glass'], None)['obstruction']


def test_distractor_far_away_is_not_an_obstruction():
    labels = _scene()
    labels[70:95, 60:90] = LABEL_IDS['distractor']
    assert not scene_label(labels, ['whiskey'], ['glass'], None)['obstruction']


def test_export_keeps_each_scene_in_one_split(tmp_path):
    raw = tmp_path / 'raw'
    for i in range(40):
        scene = raw / f's{i:03d}'
        scene.mkdir(parents=True)
        (scene / 'scene.json').write_text(json.dumps({'bottles': ['whiskey'], 'glasses': ['glass']}))
        for frame in ('0000', '0001'):
            (scene / f'{frame}.json').write_text(json.dumps({'in_gripper': 'whiskey'}))
            Image.fromarray(_scene()).save(scene / f'{frame}_overhead_labels.png')

    counts = export(raw, tmp_path / 'out')

    assert counts['train'] + counts['val'] == 80
    val_scenes = {r['images'][0].split('/')[0]
                  for r in map(json.loads, (tmp_path / 'out' / 'val.jsonl').read_text().splitlines())}
    assert val_scenes == {f's{i:03d}' for i in range(40) if is_val(f's{i:03d}')}
    first = json.loads((tmp_path / 'out' / 'train.jsonl').read_text().splitlines()[0])
    answer = json.loads(first['messages'][1]['content'][0]['text'])
    assert answer['in_gripper'] == 'whiskey'
    assert answer['glasses'][0]['name'] == 'glass'
    assert first['images'][0].endswith('_overhead_rgb.png')


def test_world_labels_match_label_ids():
    import os
    import xml.etree.ElementTree as ET
    from pathlib import Path

    import pytest
    # The world lives in bartender_robot_sim; point BAR_WORLD_SDF at a checkout of it.
    world = Path(os.environ.get('BAR_WORLD_SDF', Path(__file__).parents[2] / 'bartender_robot_sim'
                                / 'ros2_ws/src/bartender_gazebo/worlds/bar_world.sdf'))
    if not world.exists():
        pytest.skip(f'{world} not found; set BAR_WORLD_SDF')
    labels = {inc.findtext('name'): int(inc.findtext('plugin/label'))
              for inc in ET.parse(world).iter('include') if inc.find('plugin/label') is not None}
    assert labels == {'jack_daniels_bottle': LABEL_IDS['whiskey'], 'cola_bottle': LABEL_IDS['cola'],
                      'beer_bottle': LABEL_IDS['beer'], 'serving_glass': LABEL_IDS['glass']}
