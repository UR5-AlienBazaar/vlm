import copy
import json

from compare_cameras import fuse, moment
from test_vlm_reward import _label
from vlm_reward import read_scene


def test_fused_scene_sees_what_any_camera_sees_and_is_blocked_by_any():
    hidden = copy.deepcopy(_label())
    hidden['bottles'][0].update(visible=False, bbox=None)
    hidden['obstruction'] = {'value': True, 'confidence': 0.9}
    fused = fuse([read_scene(json.dumps(hidden)), None, read_scene(json.dumps(_label()))])
    assert fused['bottles']['whiskey'][0] is True
    assert fused['obstruction'] == (True, 0.9)


def test_no_usable_view_is_no_answer():
    assert fuse([None, None]) is None


def test_views_of_one_moment_share_a_key():
    assert moment('/r/vlm_raw/s1_00002/0003_wrist_rgb.png') == moment('/r/vlm_raw/s1_00002/0003_overhead_rgb.png')
