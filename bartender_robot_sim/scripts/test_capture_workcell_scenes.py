import math
import random

from capture_workcell_scenes import (ARM_BASE, BEER_CAP_HEIGHT, FOREARM, LABELS, MIN_GAP, MODELS, OBJECTS,
                                     PLACE_X, PLACE_Y,
                                     REACH_FUDGE, SCENE_PARAMS, SHOULDER_Z, UPPER_ARM, WRIST_DROP, WRIST_LATERAL, arm_ik,
                                     cap_xyz, held_sdf, look_at, sample_appearance, sample_arm,
                                     sample_held, sample_layout)


def test_look_at_points_down_at_the_target():
    roll, pitch, yaw = look_at((1.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    assert roll == 0.0
    assert abs(pitch - math.pi / 4) < 1e-9  # positive pitch tips Gazebo's +x down
    assert abs(abs(yaw) - math.pi) < 1e-9


def test_cap_follows_a_fallen_beer():
    assert cap_xyz((0.5, 0.3, 0.75), (0.0, 0.0, 1.0)) == (0.5, 0.3, 0.75 + BEER_CAP_HEIGHT)
    x, y, z = cap_xyz((0.5, 0.3, 0.8), (math.pi / 2, 0.0, 0.0))
    assert abs(x - 0.5) < 1e-9 and abs(y - (0.3 - BEER_CAP_HEIGHT)) < 1e-9 and abs(z - 0.8) < 1e-9


def test_bottles_land_on_the_table_apart():
    rng = random.Random(0)
    for _ in range(500):
        layout = sample_layout(rng, SCENE_PARAMS)
        bottles = [layout[b]['xy'] for b in OBJECTS if b in layout]
        for x, y in bottles:
            assert PLACE_X[0] <= x <= PLACE_X[1] and PLACE_Y[0] <= y <= PLACE_Y[1]
        assert all(math.dist(a, b) >= MIN_GAP for i, a in enumerate(bottles) for b in bottles[i + 1:])


def test_blocking_distractor_stands_between_the_arm_and_a_bottle():
    params = {**SCENE_PARAMS, 'bottle_p': 1.0, 'distractors': [0, 1], 'block_p': 1.0, 'named_distractor_p': 0.0}
    rng = random.Random(1)
    for _ in range(200):
        layout = sample_layout(rng, params)
        [d] = [v['xy'] for k, v in layout.items() if k.startswith('distractor')]
        # Closer to the arm than some bottle it is right in front of.
        assert any(math.dist(d, layout[b]['xy']) < 0.17 and
                   math.dist(d, ARM_BASE) < math.dist(layout[b]['xy'], ARM_BASE) for b in OBJECTS)


def test_distractor_kinds_restrict_the_pool():
    params = {**SCENE_PARAMS, 'distractors': [0, 0, 1], 'distractor_kinds': ['thermos'], 'named_distractor_p': 0.0}
    layout = sample_layout(random.Random(2), params)
    assert [v['kind'] for k, v in layout.items() if k.startswith('distractor')] == ['thermos']


def test_arm_ik_reaches_where_asked():
    lift, elbow = arm_ik(0.70, 0.50)
    up = -lift  # upper arm angle above horizontal
    r = UPPER_ARM * math.cos(up) + FOREARM * math.cos(up - elbow) + REACH_FUDGE
    z = SHOULDER_Z + UPPER_ARM * math.sin(up) + FOREARM * math.sin(up - elbow) - WRIST_DROP
    assert abs(r - 0.70) < 1e-9 and abs(z - 0.50) < 1e-9
    assert arm_ik(2.0, 0.5) is None


def test_over_bottle_aims_at_its_bottle():
    params = {**SCENE_PARAMS, 'bottle_p': 1.0, 'arm_pose': {'over_bottle': 1.0}}
    aimed = 0
    for seed in range(20):  # a far bottle is out of reach and falls back to over_table
        rng = random.Random(seed)
        layout = sample_layout(rng, params)
        kind, target, joints = sample_arm(rng, params, layout)
        if kind != 'over_bottle':
            continue
        bx, by = layout[target]['xy']
        aim = math.atan2(by - ARM_BASE[1], bx - ARM_BASE[0])
        # Turned short of it by the wrist's side offset, at most that of the shortest reach drawn.
        shortest = math.dist((bx, by), ARM_BASE) - 0.10
        assert 0 < aim - (joints[0] - math.pi / 2) <= math.asin(min(1.0, WRIST_LATERAL / shortest)) + 1e-9
        aimed += 1
    assert aimed


def test_appearance_can_be_turned_off():
    look, colours = sample_appearance(random.Random(4), {**SCENE_PARAMS, 'appearance_p': 0.0})
    assert look is None and len(colours) >= 3
    look, _ = sample_appearance(random.Random(4), {**SCENE_PARAMS, 'appearance_p': 1.0})
    assert look['sun']['direction'][2] < 0  # the sun shines down


def test_over_table_always_finds_a_reachable_pose():
    params = {**SCENE_PARAMS, 'arm_pose': {'over_table': 1.0}}
    rng = random.Random(6)
    for _ in range(2000):
        kind, _, joints = sample_arm(rng, params, {})
        assert kind == 'over_table' and len(joints) == 6
def test_held_bottle_leaves_the_table_and_the_arm_takes_it():
    params = {**SCENE_PARAMS, 'bottle_p': 1.0, 'fallen_p': 0.0, 'held_p': 1.0}
    rng = random.Random(5)
    layout = sample_layout(rng, params)
    held = sample_held(rng, params, layout)
    assert held in OBJECTS and held not in layout
    assert sample_arm(rng, params, layout, held)[:2] == ('holding', held)


def test_held_copy_is_a_static_labelled_visual():
    sdf = held_sdf('beer', 3)
    assert '<model name="held_beer"><static>true</static>' in sdf
    assert '<collision' not in sdf and 'detachable-joint' not in sdf
    assert '<label>3</label>' in sdf and '<uri>model://beer_bottle/meshes/' in sdf


def test_named_distractor_counts_as_a_distractor_with_its_own_model():
    params = {**SCENE_PARAMS, 'distractors': [1.0], 'named_distractor_p': 1.0}
    layout = sample_layout(random.Random(7), params)
    assert layout['distractor_ballantines']['kind'] == 'ballantines'
    assert MODELS['distractor_ballantines'] == 'ballantines_bottle'


def test_every_model_is_in_the_world_with_its_label():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    world = Path(__file__).resolve().parents[1] / 'ros2_ws/src/bartender_gazebo/worlds/workcell_world.sdf'
    labels = {inc.findtext('name'): int(inc.findtext('plugin/label'))
              for inc in ET.parse(world).iter('include') if inc.find('plugin/label') is not None}
    assert {OBJECTS[b]: LABELS[b] for b in OBJECTS}.items() <= labels.items()
    assert 20 <= labels[MODELS['distractor_ballantines']] <= 29
    assert 2 not in {labels[m] for m in MODELS.values()}  # cola's label stays retired
