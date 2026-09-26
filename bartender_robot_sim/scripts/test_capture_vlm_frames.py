import math
import random
from types import SimpleNamespace

from capture_vlm_frames import DISTRACTOR_P, GLASS_HOME, GLASS_JITTER, held_bottle, sample_scene, tilt_deg



def test_held_follows_the_pour_state_machine():
    assert held_bottle('pouring_whiskey') == 'whiskey'
    assert held_bottle('moving_to_glass_cola') == 'cola'
    assert held_bottle('closing_on_whiskey') is None
    assert held_bottle('releasing_whiskey') is None
    assert held_bottle('') is None
    assert held_bottle('returning_home') is None


def test_tilt_of_a_toppled_bottle():
    upright = SimpleNamespace(x=0.0, y=0.0, z=0.3, w=0.95)
    on_its_side = SimpleNamespace(x=math.sin(math.pi / 4), y=0.0, z=0.0, w=math.cos(math.pi / 4))
    assert tilt_deg(upright) < 1
    assert abs(tilt_deg(on_its_side) - 90) < 1


def test_scenes_stay_in_bounds_and_mix_distractors():
    rng = random.Random(0)
    scenes = [sample_scene(rng) for _ in range(2000)]
    for s in scenes:
        assert abs(s['glass'][0] - GLASS_HOME[0]) <= GLASS_JITTER
        assert abs(s['glass'][1] - GLASS_HOME[1]) <= GLASS_JITTER
        assert set(s['shown']) <= {'whiskey', 'cola'}
    share = sum(s['distractor'] is not None for s in scenes) / len(scenes)
    assert abs(share - DISTRACTOR_P) < 0.03
