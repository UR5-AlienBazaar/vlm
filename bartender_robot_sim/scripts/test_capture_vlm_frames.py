import math
import random
from types import SimpleNamespace

from capture_vlm_frames import (DISTRACTOR_P, GLASS_HOME, GLASS_JITTER, held_bottle, pour_outcome,
                                sample_scene, tilt_deg)



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


PHASES_DONE = [(0.0, 'pouring_whiskey'), (1.0, 'returning_upright_whiskey'),
               (2.0, 'pouring_cola'), (3.0, 'returning_upright_cola')]


def _poses(**tilts):
    return {name: {'xyz': [0.08, y, 0.9], 'tilt_deg': tilts.get(name, 0.0)}
            for name, y in (('whiskey', -0.30), ('cola', -0.15), ('beer', 0.0))}


def test_a_pour_that_tips_bottles_is_not_clean_even_if_the_action_succeeded():
    outcome = pour_outcome(True, PHASES_DONE, _poses(), _poses(whiskey=85.0, cola=90.0))
    assert outcome['action_success'] and not outcome['clean']
    assert outcome['bottles']['whiskey']['upright'] is False


def test_a_bottle_left_off_its_station_is_not_clean():
    end = _poses()
    end['cola']['xyz'] = [0.08, -0.10, 0.9]
    outcome = pour_outcome(True, PHASES_DONE, _poses(), end)
    assert not outcome['clean'] and not outcome['bottles']['cola']['on_station']


def test_clean_needs_every_pour_finished():
    assert pour_outcome(True, PHASES_DONE, _poses(), _poses())['clean']
    cut_short = PHASES_DONE[:3] + [(3.0, 'checking_grip_cola')]
    outcome = pour_outcome(True, cut_short, _poses(), _poses())
    assert outcome['poured'] == {'whiskey': True, 'cola': False} and not outcome['clean']
