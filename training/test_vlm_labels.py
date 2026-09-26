import json

import numpy as np
from PIL import Image

from vlm_labels import (ARM_A_BASE, LABEL_IDS, MIN_VISIBLE_PX, STANDS, bbox, blocked, bucket, export,
                        is_val, scene_label, visibility_confidence)


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
    label = scene_label(labels, ['whiskey'], [], None, False)
    assert label['bottles'] == [{'name': 'whiskey', 'visible': False,
                                 'confidence': visibility_confidence(MIN_VISIBLE_PX - 1), 'bbox': None}]
    assert label['bottles'][0]['confidence'] < 1


def test_second_glass_is_another_entry_not_a_new_key():
    labels = _scene()
    labels[50:90, 100:120] = LABEL_IDS['glass_b']
    label = scene_label(labels, ['whiskey'], ['glass', 'glass_b'], None, False)
    assert [(g['name'], g['visible']) for g in label['glasses']] == [('glass', True), ('glass_b', True)]


def _midway(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)


def test_distractor_on_the_reach_to_a_shown_bottle_blocks():
    params = {'shown': ['whiskey'], 'glass': (0.2, -0.55),
              'distractor': _midway(ARM_A_BASE, STANDS['whiskey'])}
    assert blocked(params)


def test_distractor_on_the_reach_to_a_hidden_bottle_does_not_block():
    params = {'shown': ['cola'], 'glass': (0.2, -0.55),
              'distractor': (STANDS['whiskey'][0] - 0.01, STANDS['whiskey'][1] - 0.04)}
    assert not blocked(params)


def test_distractor_beside_the_reaches_does_not_block():
    assert not blocked({'shown': ['whiskey', 'cola'], 'glass': (0.2, -0.55), 'distractor': (0.0, 0.2)})
    assert not blocked({'shown': ['whiskey'], 'glass': (0.2, -0.55), 'distractor': None})
    assert not blocked({})


def test_obstruction_is_sim_state_whatever_the_picture_shows():
    label = scene_label(_scene(), ['whiskey'], ['glass'], None, True)
    assert label['obstruction'] == {'value': True, 'confidence': 1.0}


def test_confidence_is_lowest_at_the_visibility_cutoff():
    assert visibility_confidence(0) == 1.0
    assert visibility_confidence(2 * MIN_VISIBLE_PX) == 1.0
    assert visibility_confidence(MIN_VISIBLE_PX) == 0.5


def test_partly_hidden_bottle_is_visible_but_less_certain():
    labels = _scene()
    labels[10:60, 20:40] = LABEL_IDS['distractor']
    labels[10:13, 20:40] = LABEL_IDS['whiskey']
    whiskey = scene_label(labels, ['whiskey'], [], None, False)['bottles'][0]
    assert whiskey['visible'] and whiskey['confidence'] < 1


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
    assert answer['in_gripper'] == {'value': 'whiskey', 'confidence': 1.0}
    assert first['camera'] == 'overhead'
    assert first['bucket'] == 'static/none/whiskey'
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


def test_workcell_world_labels_match_label_ids():
    import xml.etree.ElementTree as ET
    from pathlib import Path

    from vlm_labels import DISTRACTOR_IDS

    world = Path(__file__).parents[1] / 'bartender_robot_sim/ros2_ws/src/bartender_gazebo/worlds/workcell_world.sdf'
    labels = {inc.findtext('name'): int(inc.findtext('plugin/label'))
              for inc in ET.parse(world).iter('include') if inc.find('plugin/label') is not None}
    assert labels == {'jack_daniels_bottle': LABEL_IDS['whiskey'], 'cola_bottle': LABEL_IDS['cola'],
                      'beer_bottle': LABEL_IDS['beer'], 'zubrowka_bottle': LABEL_IDS['vodka'],
                      'jagermeister_bottle': LABEL_IDS['liqueur'], 'tenjaku_bottle': LABEL_IDS['gin'],
                      'frontera_bottle': LABEL_IDS['wine'], 'ballantines_bottle': LABEL_IDS['ballantines']}
    assert LABEL_IDS['ballantines'] in DISTRACTOR_IDS

def test_pour_stride_thins_pour_frames_only(tmp_path):
    raw = tmp_path / 'raw'
    for name, kind in (('pour', 'pour'), ('still', 'static')):
        scene = raw / name
        scene.mkdir(parents=True)
        (scene / 'scene.json').write_text(json.dumps({'bottles': ['whiskey'], 'glasses': [], 'kind': kind}))
        for i in range(4):
            (scene / f'{i:04d}.json').write_text(json.dumps({'in_gripper': None}))
            Image.fromarray(_scene()).save(scene / f'{i:04d}_overhead_labels.png')

    counts = export(raw, tmp_path / 'out', pour_stride=2)

    assert counts['train'] + counts['val'] == 2 + 4


def test_any_label_in_20_to_29_is_a_distractor():
    labels = _scene()
    labels[10:60, 41:60] = 23
    label = scene_label(labels, ['whiskey'], ['glass'], None, False)
    assert bucket({}, label, labels) == 'static/far/whiskey'
