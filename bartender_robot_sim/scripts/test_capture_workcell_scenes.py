import math
import random

from capture_workcell_scenes import (ARM_BASE, BEER_CAP_HEIGHT, FOREARM, MIN_GAP, OBJECTS, PLACE_X, PLACE_Y,
                                     REACH_FUDGE, SCENE_PARAMS, SHOULDER_Z, UPPER_ARM, WRIST_DROP, arm_ik,
                                     cap_xyz, look_at, sample_appearance, sample_arm, sample_layout)


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
    params = {**SCENE_PARAMS, 'bottle_p': 1.0, 'distractors': [0, 1], 'block_p': 1.0}
    rng = random.Random(1)
    for _ in range(200):
        layout = sample_layout(rng, params)
        [d] = [v['xy'] for k, v in layout.items() if k.startswith('distractor')]
        # Closer to the arm than some bottle it is right in front of.
        assert any(math.dist(d, layout[b]['xy']) < 0.17 and
                   math.dist(d, ARM_BASE) < math.dist(layout[b]['xy'], ARM_BASE) for b in OBJECTS)


def test_distractor_kinds_restrict_the_pool():
    params = {**SCENE_PARAMS, 'distractors': [0, 0, 1], 'distractor_kinds': ['wine']}
    layout = sample_layout(random.Random(2), params)
    assert [v['kind'] for k, v in layout.items() if k.startswith('distractor')] == ['wine']


def test_arm_ik_reaches_where_asked():
    lift, elbow = arm_ik(0.70, 0.50)
    up = -lift  # upper arm angle above horizontal
    r = UPPER_ARM * math.cos(up) + FOREARM * math.cos(up - elbow) + REACH_FUDGE
    z = SHOULDER_Z + UPPER_ARM * math.sin(up) + FOREARM * math.sin(up - elbow) - WRIST_DROP
    assert abs(r - 0.70) < 1e-9 and abs(z - 0.50) < 1e-9
    assert arm_ik(2.0, 0.5) is None


def test_over_bottle_aims_at_its_bottle():
    params = {**SCENE_PARAMS, 'bottle_p': 1.0, 'arm_pose': {'over_bottle': 1.0}}
    rng = random.Random(3)
    layout = sample_layout(rng, params)
    kind, target, joints = sample_arm(rng, params, layout)
    bx, by = layout[target]['xy']
    assert kind == 'over_bottle'
    aim = math.atan2(by - ARM_BASE[1], bx - ARM_BASE[0])
    assert 0 < aim - (joints[0] - math.pi / 2) < 0.5  # turned short of it by the wrist's side offset


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
