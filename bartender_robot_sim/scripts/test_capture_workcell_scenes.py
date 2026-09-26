import math
import random

from capture_workcell_scenes import (ARM_BASE, BEER_CAP_HEIGHT, MIN_GAP, OBJECTS, PLACE_X, PLACE_Y,
                                     SCENE_PARAMS, cap_xyz, look_at, sample_layout)


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
        d = layout['distractor0']['xy']
        # Closer to the arm than some bottle it is right in front of.
        assert any(math.dist(d, layout[b]['xy']) < 0.17 and
                   math.dist(d, ARM_BASE) < math.dist(layout[b]['xy'], ARM_BASE) for b in OBJECTS)
