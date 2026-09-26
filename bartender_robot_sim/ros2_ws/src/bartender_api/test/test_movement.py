"""Tests for movement.py -- goto / jog / gripper over Pendant.dispatch().

Pendant itself needs a live rclpy TeachNode to actually move anything, so
these tests split in two: the pre-validation MovementBridge does BEFORE
ever calling dispatch (real Pendant, real PointStore, a bare stand-in
node), and the classification of dispatch's captured text AFTER it runs
(dispatch itself faked, since exercising the real one needs move_group).
Both halves are real code paths, just on either side of the one call this
module cannot unit-test without a simulator.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    *([os.pardir] * 4), 'ros2_ws', 'src', 'bartender_teach'))

from bartender_api import movement                          # noqa: E402
from bartender_teach.point_store import Point, PointStore    # noqa: E402
from bartender_teach.teach_points import ARMS                # noqa: E402


def _bridge(with_point=True):
    store = PointStore('/tmp/does-not-matter.yaml')
    if with_point:
        store.add(Point('whiskey_approach',
                        {j: 0.0 for j in ARMS['a'].joints}))
    node = types.SimpleNamespace(
        get_logger=lambda: types.SimpleNamespace(error=lambda m: None))
    return movement.MovementBridge(node, store)


# -- pre-validation: refused before dispatch ever runs -----------------

def test_goto_refuses_an_unknown_point_without_touching_dispatch():
    bridge = _bridge(with_point=False)
    calls = []
    bridge.pendant.dispatch = lambda line: calls.append(line)
    result = bridge.goto('a', 'no_such_point')
    assert result['ok'] is False
    assert 'no point named' in result['message']
    assert calls == []


def test_goto_refuses_an_unknown_arm():
    bridge = _bridge()
    result = bridge.goto('c', 'whiskey_approach')
    assert result['ok'] is False
    assert 'no arm' in result['message']


def test_jog_refuses_an_unknown_axis():
    bridge = _bridge()
    result = bridge.jog('a', 'q', 10)
    assert result['ok'] is False
    assert 'unknown jog axis' in result['message']


def test_jog_refuses_a_non_numeric_amount():
    bridge = _bridge()
    result = bridge.jog('a', 'z', 'a lot')
    assert result['ok'] is False
    assert 'must be a number' in result['message']


def test_jog_refuses_a_linear_amount_past_max_jog_mm():
    bridge = _bridge()
    result = bridge.jog('a', 'z', movement.MAX_JOG_MM + 1)
    assert result['ok'] is False
    assert 'Refusing rather than clamping' in result['message']


def test_jog_refuses_a_rotation_amount_past_max_jog_deg():
    bridge = _bridge()
    result = bridge.jog('a', 'rz', movement.MAX_JOG_DEG + 1)
    assert result['ok'] is False
    assert 'Refusing rather than clamping' in result['message']


def test_jog_accepts_every_real_axis_shape_within_bound():
    """One reading past the refusal for each family cmd_jog recognises."""
    bridge = _bridge()
    for axis in ('x', 'y', 'z', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz',
                 'j1', 'j6'):
        bridge.pendant.dispatch = lambda line: print('  ok')
        result = bridge.jog('a', axis, 1.0)
        assert result['ok'] is True, f'axis {axis} was refused: {result}'


def test_jog_refuses_a_joint_index_out_of_range():
    bridge = _bridge()
    result = bridge.jog('a', 'j7', 1.0)
    assert result['ok'] is False


def test_gripper_refuses_a_position_outside_the_usable_band():
    bridge = _bridge()
    result = bridge.gripper('a', 0.0)
    assert result['ok'] is False
    assert 'outside' in result['message']


def test_gripper_refuses_a_non_numeric_position():
    bridge = _bridge()
    result = bridge.gripper('a', 'wide open')
    assert result['ok'] is False
    assert 'must be a number' in result['message']


# -- locking: refused, not queued ---------------------------------------

def test_a_second_command_is_refused_while_one_is_running():
    bridge = _bridge()
    assert bridge._lock.acquire(blocking=False)
    try:
        result = bridge.gripper('a', 0.3)
    finally:
        bridge._lock.release()
    assert result['ok'] is False
    assert 'busy' in result['message']


# -- classification of dispatch's own captured text ----------------------

def test_a_successful_report_line_classifies_as_ok():
    text = '  now driving arm A ...\n  moving to whiskey_approach ...\n  at whiskey_approach\n'
    assert movement._classify(text) is True


def test_a_failed_report_line_classifies_as_not_ok():
    text = "  jog z +20mm ...\n  FAILED: flange is 12.0mm from where it was sent\n"
    assert movement._classify(text) is False


def test_an_unreachable_pose_service_classifies_as_not_ok():
    """_require_pose's refusal: no "FAILED:" prefix, but still a failure."""
    text = ('  cannot read tool0 through /compute_fk, so there is no pose '
            'to jog FROM. Is move_group running? Joint jogs still work.\n')
    assert movement._classify(text) is False


def test_empty_output_is_not_ok():
    """Dispatch always prints something for a real command; nothing is a bug."""
    assert movement._classify('') is False


# -- end to end through the real Pendant, dispatch's prints faked --------

def test_goto_runs_arm_select_then_goto_through_dispatch():
    bridge = _bridge()
    lines = []

    def fake_dispatch(line):
        lines.append(line)
        print('  at whiskey_approach' if line.startswith('goto') else '  ok')

    bridge.pendant.dispatch = fake_dispatch
    result = bridge.goto('a', 'whiskey_approach')
    assert lines == ['arm a', 'goto whiskey_approach']
    assert result == {'ok': True, 'message': 'ok\n  at whiskey_approach'}


# -- pick: grab_<bottle> pipelines ---------------------------------------

def _pick_bridge(tmp_path, pipelines=None, points=('grab_point',)):
    """Build a bridge over a real point file on disk, since pick re-reads it."""
    from bartender_teach.pipelines import Pipeline, Step
    store = PointStore(str(tmp_path / 'points.yaml'))
    for name in points:
        store.add(Point(name, {j: 0.0 for j in ARMS['a'].joints}))
    default = {'grab_whiskey': ['grab_point']}
    for name, gotos in (default if pipelines is None else pipelines).items():
        store.pipelines[name] = Pipeline(
            name, [Step.from_dict({'goto': g}) for g in gotos])
    store.save()
    node = types.SimpleNamespace(
        get_logger=lambda: types.SimpleNamespace(error=lambda m: None))
    return movement.MovementBridge(node, store)


def test_bottles_lists_one_per_grab_pipeline_and_nothing_else(tmp_path):
    bridge = _pick_bridge(tmp_path, {
        'grab_whiskey': ['grab_point'], 'grab_gin': ['grab_point'],
        'pour_whiskey': ['grab_point'], 'grab_': ['grab_point']})
    out = bridge.bottles()
    assert [b['bottle'] for b in out['bottles']] == ['gin', 'whiskey']
    assert out['bottles'][1] == {'bottle': 'whiskey',
                                 'pipeline': 'grab_whiskey', 'steps': 1}


def test_bottles_sees_a_bottle_taught_after_startup(tmp_path):
    bridge = _pick_bridge(tmp_path)
    # The pendant, another process, records grab_gin into the same file.
    from bartender_teach.pipelines import Pipeline, Step
    other = PointStore.load(bridge.pendant.store.path)
    other.pipelines['grab_gin'] = Pipeline(
        'grab_gin', [Step.from_dict({'goto': 'grab_point'})])
    other.save()
    assert 'gin' in [b['bottle'] for b in bridge.bottles()['bottles']]


def test_pick_runs_the_grab_pipeline(tmp_path):
    bridge = _pick_bridge(tmp_path)
    calls = []

    def fake_dispatch(line):
        calls.append(line)
        print('  running grab_whiskey: 1 step(s)\n  finished grab_whiskey')
    bridge.pendant.dispatch = fake_dispatch
    result = bridge.pick(' Whiskey ')
    assert calls == ['run grab_whiskey']
    assert result['ok'] is True
    assert result['bottle'] == 'whiskey'


def test_pick_reports_a_stopped_run_as_failed(tmp_path):
    bridge = _pick_bridge(tmp_path)
    bridge.pendant.dispatch = lambda line: print(
        '   1/1  goto grab_point ...\n  STOPPED at step 1 of 1: error_code -6')
    result = bridge.pick('whiskey')
    assert result['ok'] is False
    assert 'error_code -6' in result['message']


def test_pick_refuses_an_untaught_bottle_and_names_the_known_ones(tmp_path):
    bridge = _pick_bridge(tmp_path)
    calls = []
    bridge.pendant.dispatch = lambda line: calls.append(line)
    result = bridge.pick('gin')
    assert result['ok'] is False
    assert 'Known: whiskey' in result['message']
    assert 'record grab_gin' in result['message']
    assert calls == []


def test_pick_refuses_a_pipeline_with_missing_points(tmp_path):
    bridge = _pick_bridge(tmp_path, {'grab_gin': ['gin_grasp']})
    calls = []
    bridge.pendant.dispatch = lambda line: calls.append(line)
    result = bridge.pick('gin')
    assert result['ok'] is False
    assert 'gin_grasp' in result['message']
    assert calls == []


def test_pick_refuses_no_bottle(tmp_path):
    bridge = _pick_bridge(tmp_path)
    assert bridge.pick(None)['ok'] is False
    assert bridge.pick('  ')['ok'] is False


def test_pick_refuses_while_busy(tmp_path):
    bridge = _pick_bridge(tmp_path)
    bridge._lock.acquire()
    try:
        result = bridge.pick('whiskey')
    finally:
        bridge._lock.release()
    assert result == {'ok': False,
                      'message': 'busy: a command is already running'}


# -- make: a drink is a list of scripts ----------------------------------

MENU = '''
drinks:
  whiskey_cola:
    name: Whiskey & Cola
    scripts: [grab_whiskey, pour_cola]
  gin_sprite:
    scripts: [grab_gin]
'''


def _make_bridge(tmp_path, pipelines=None):
    bridge = _pick_bridge(tmp_path, pipelines or {
        'grab_whiskey': ['grab_point'], 'pour_cola': ['grab_point']})
    path = tmp_path / 'menu.yaml'
    path.write_text(MENU)
    bridge.menu_path = str(path)
    return bridge


def _finishing(calls):
    def dispatch(line):
        calls.append(line)
        print(f'  finished {line.split()[1]}')
    return dispatch


def test_choices_answers_while_a_drink_is_being_made(tmp_path):
    bridge = _make_bridge(tmp_path, {'grab_whiskey': ['grab_point'],
                                     'grab_gin': ['grab_point'],
                                     'pour_cola': ['grab_point']})
    bridge._lock.acquire()   # what /make holds for the whole drink
    try:
        drinks, not_ready, bottles = bridge.choices()
    finally:
        bridge._lock.release()
    assert drinks == {'whiskey_cola': 'Whiskey & Cola', 'gin_sprite': 'gin_sprite'}
    assert (not_ready, bottles) == ([], ['gin', 'whiskey'])


def test_choices_names_the_drinks_whose_scripts_are_not_taught(tmp_path):
    _, not_ready, bottles = _make_bridge(tmp_path).choices()
    assert (not_ready, bottles) == (['gin_sprite'], ['whiskey'])


def test_drinks_says_which_are_ready_and_what_is_missing(tmp_path):
    out = {d['drink']: d for d in _make_bridge(tmp_path).drinks()['drinks']}
    assert out['whiskey_cola']['ready'] is True
    assert out['whiskey_cola']['name'] == 'Whiskey & Cola'
    assert out['gin_sprite']['ready'] is False
    assert out['gin_sprite']['missing_scripts'] == ['grab_gin']


def test_make_runs_every_script_in_order(tmp_path):
    bridge = _make_bridge(tmp_path)
    calls = []
    bridge.pendant.dispatch = _finishing(calls)
    result = bridge.make('whiskey_cola')
    assert result['ok'] is True
    assert result['drink'] == 'whiskey_cola'
    assert calls == ['run grab_whiskey', 'run pour_cola']


def test_make_stops_at_the_first_script_that_does_not_finish(tmp_path):
    bridge = _make_bridge(tmp_path)
    calls = []

    def dispatch(line):
        calls.append(line)
        print('  STOPPED at step 1 of 1: error_code -6')
    bridge.pendant.dispatch = dispatch
    result = bridge.make('whiskey_cola')
    assert result['ok'] is False
    assert result['stopped_at'] == 'script 1 of 2: grab_whiskey'
    assert calls == ['run grab_whiskey']


def test_make_refuses_a_drink_with_untaught_scripts_before_moving(tmp_path):
    bridge = _make_bridge(tmp_path)
    calls = []
    bridge.pendant.dispatch = _finishing(calls)
    result = bridge.make('gin_sprite')
    assert result['ok'] is False
    assert 'scripts not taught yet: grab_gin' in result['message']
    assert calls == []


def test_make_refuses_a_script_with_missing_points(tmp_path):
    bridge = _make_bridge(tmp_path, {'grab_whiskey': ['grab_point'],
                                     'pour_cola': ['cola_pour_point']})
    calls = []
    bridge.pendant.dispatch = _finishing(calls)
    result = bridge.make('whiskey_cola')
    assert result['ok'] is False
    assert 'points missing: cola_pour_point' in result['message']
    assert calls == []


def test_make_refuses_an_unknown_drink_and_lists_the_menu(tmp_path):
    result = _make_bridge(tmp_path).make('mojito')
    assert result['ok'] is False
    assert 'whiskey_cola' in result['message']


def test_make_without_a_menu_says_how_to_get_one(tmp_path):
    bridge = _pick_bridge(tmp_path)
    assert '--menu' in bridge.make('whiskey_cola')['message']
    assert '--menu' in bridge.drinks()['error']


def test_the_shipped_workcell_menu_loads_and_uses_only_its_six_bottles():
    from bartender_api import menu
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        *([os.pardir] * 2), 'bartender_teach', 'config',
                        'workcell_menu.yaml')
    drinks = menu.load(path)
    assert 'whiskey_cola' in drinks
    names = {s.split('_', 1)[1] for d in drinks.values() for s in d.scripts}
    assert names <= {'gin', 'sprite', 'cola', 'fanta', 'whiskey', 'vodka'}
