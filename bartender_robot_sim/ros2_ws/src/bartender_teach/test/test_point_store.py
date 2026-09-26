"""Tests for the taught-point file: parsing, refusals, round-trip, wrapping."""
import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.point_store import (      # noqa: E402
    Point, PointStore, PointStoreError, wrap_angle,
)

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
SAMPLE = dict(zip(ARM, [0.079, -1.905, 2.337, -0.432, 1.65, 0.0]))


def write(tmp_path, text):
    p = tmp_path / 'points.yaml'
    p.write_text(text)
    return str(p)


# -- wrapping ---------------------------------------------------------------

@pytest.mark.parametrize('raw', [0.0, 1.0, -1.0, math.pi - 1e-6, 7.0, -7.0,
                                 2 * math.pi, 100.0])
def test_wrap_lands_in_range(raw):
    assert -math.pi - 1e-9 <= wrap_angle(raw) <= math.pi + 1e-9


@pytest.mark.parametrize('raw', [7.0, -7.0, 2 * math.pi + 0.5, -3 * math.pi])
def test_wrap_preserves_the_pose(raw):
    """Wrapping must only ever remove whole turns.

    A wrap that changed the angle by anything else would move the arm, not
    just shorten the path it takes to get there.
    """
    turns = (raw - wrap_angle(raw)) / (2 * math.pi)
    assert abs(turns - round(turns)) < 1e-9


def test_wrapped_reports_what_it_changed():
    point = Point('p', {'a': 7.0, 'b': 0.5})
    out, changed = point.wrapped()
    assert [c[0] for c in changed] == ['a']
    assert out.joints['b'] == 0.5
    assert abs(out.joints['a'] - wrap_angle(7.0)) < 1e-12


# -- joint order ------------------------------------------------------------

def test_order_follows_the_names_not_the_file():
    """A file written in one order must come back in the caller's order.

    This is the whole reason joints are stored as a mapping.
    """
    scrambled = dict(reversed(list(SAMPLE.items())))
    assert Point('p', scrambled).joints_in_order(ARM) == [SAMPLE[j] for j in ARM]


def test_missing_joint_raises_rather_than_defaulting():
    with pytest.raises(PointStoreError) as exc:
        Point('p', {'shoulder_pan_joint': 0.1}).joints_in_order(ARM)
    assert 'elbow_joint' in str(exc.value)
    assert 'p' in str(exc.value)


# -- round trip -------------------------------------------------------------

def test_round_trip(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('grasp', SAMPLE, pose={'xyz': [0.5, 0.1, 0.2],
                                           'quat_xyzw': [0.5, 0.5, 0.5, 0.5]},
                    gripper=0.25, note='beside the whiskey'))
    store.save()

    back = PointStore.load(store.path)
    point = back.get('grasp')
    assert point.joints_in_order(ARM) == pytest.approx([SAMPLE[j] for j in ARM])
    assert point.gripper == pytest.approx(0.25)
    assert point.note == 'beside the whiskey'
    assert point.pose['xyz'] == [0.5, 0.1, 0.2]


def test_saved_file_is_valid_yaml_with_a_header(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('a', SAMPLE))
    store.save()
    text = open(store.path).read()
    assert text.startswith('#')
    assert yaml.safe_load(text)['points']['a']['joints']


def test_save_is_atomic_and_leaves_no_temp_files(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('a', SAMPLE))
    store.save()
    store.add(Point('b', SAMPLE))
    store.save()
    assert sorted(os.listdir(tmp_path)) == ['p.yaml']


def test_shipped_point_file_loads():
    """The seeded config must parse with the same loader the tool uses."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    store = PointStore.load(os.path.join(here, 'config', 'taught_points.yaml'),
                            missing_ok=False)
    assert 'home' in store
    assert 'b_home' in store, 'arm B\'s rest pose should be teachable too'
    for name in store.names():
        point = store.get(name)
        # Each point is complete FOR ITS OWN ARM. Asking every point for arm
        # A's joint names was right while there was one arm and is exactly the
        # bug this file exists to catch now there are two.
        prefix = 'b_' if all(j.startswith('b_') for j in point.joints) else ''
        point.joints_in_order([prefix + j for j in ARM])


# -- refusals ---------------------------------------------------------------

def test_missing_file_is_an_empty_store(tmp_path):
    assert len(PointStore.load(str(tmp_path / 'nope.yaml'))) == 0


def test_missing_file_can_be_fatal_instead(tmp_path):
    with pytest.raises(PointStoreError):
        PointStore.load(str(tmp_path / 'nope.yaml'), missing_ok=False)


def test_broken_yaml_names_the_file(tmp_path):
    path = write(tmp_path, 'points: [unclosed\n')
    with pytest.raises(PointStoreError) as exc:
        PointStore.load(path)
    assert path in str(exc.value)


@pytest.mark.parametrize('text,expect', [
    ('- a\n- b\n', 'expected a mapping'),
    ('points: 3\n', 'expected a mapping of name'),
    ('points:\n  a: 3\n', 'expected a mapping with at least'),
    ('points:\n  a: {}\n', 'no usable `joints:`'),
    ('points:\n  a: {joints: {}}\n', 'no usable `joints:`'),
    ('points:\n  a: {joints: {j1: hello}}\n', 'expected a number in radians'),
    ('points:\n  a: {joints: {j1: true}}\n', 'expected a number in radians'),
])
def test_malformed_files_are_refused_with_a_reason(tmp_path, text, expect):
    with pytest.raises(PointStoreError) as exc:
        PointStore.load(write(tmp_path, text))
    assert expect in str(exc.value)


def test_empty_file_is_an_empty_store(tmp_path):
    assert len(PointStore.load(write(tmp_path, ''))) == 0


def test_unknown_point_lists_the_known_ones(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('alpha', SAMPLE))
    with pytest.raises(PointStoreError) as exc:
        store.get('beta')
    assert 'alpha' in str(exc.value)


def test_add_refuses_to_clobber_by_default(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('a', SAMPLE))
    with pytest.raises(PointStoreError):
        store.add(Point('a', SAMPLE))
    store.add(Point('a', SAMPLE), overwrite=True)


def test_remove_unknown_raises(tmp_path):
    with pytest.raises(PointStoreError):
        PointStore(str(tmp_path / 'p.yaml')).remove('a')


# -- pipelines in the same file ---------------------------------------------
#
# Pipelines share the point file because a pipeline is meaningless without the
# points it names. What matters here is that they cost the readers which only
# want points -- the two action servers -- nothing at all.

def test_a_file_with_no_pipelines_loads_as_having_none(tmp_path):
    path = write(tmp_path, 'points:\n  p:\n    joints: {a: 0.0}\n')
    assert PointStore.load(path).pipelines == {}


def test_saving_without_pipelines_writes_no_pipelines_key(tmp_path):
    """So a workspace that never records one keeps an unchanged diff."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('here', SAMPLE))
    store.save()
    assert 'pipelines' not in yaml.safe_load(open(store.path))


def test_a_pipeline_survives_save_and_load(tmp_path):
    from bartender_teach.pipelines import Pipeline, Step
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('here', SAMPLE))
    store.pipelines['go'] = Pipeline(
        'go', [Step('goto', 'here'), Step('grip', 0.25, 'clamp', 'a'),
               Step('wait', 0.5)], note='a note')
    store.save()

    back = PointStore.load(store.path).pipelines['go']
    assert back.note == 'a note'
    assert [s.describe() for s in back.steps] == [
        'goto here', 'grip 0.250 (arm a)  -- clamp', 'wait 0.5s']


def test_a_pipeline_may_name_points_that_do_not_exist_yet(tmp_path):
    """Allow a pipeline written before its points are taught.

    Refusing would take the whole file down, including for the action
    servers, which ignore pipelines entirely.
    """
    path = write(tmp_path, 'points: {}\npipelines:\n  go:\n'
                           '    steps: [{goto: later}]\n')
    store = PointStore.load(path)
    assert store.pipelines['go'].missing_points(store) == ['later']


def test_a_malformed_pipeline_is_a_point_store_error(tmp_path):
    """Every caller already handles that type; a new one would escape them."""
    path = write(tmp_path, 'points: {}\npipelines:\n  go:\n'
                           '    steps: [{grip: 99.0}]\n')
    with pytest.raises(PointStoreError):
        PointStore.load(path)


def test_pipelines_that_are_not_a_mapping_are_refused(tmp_path):
    path = write(tmp_path, 'points: {}\npipelines: 7\n')
    with pytest.raises(PointStoreError):
        PointStore.load(path)


# -- which file ---------------------------------------------------------------

def test_a_bare_file_name_is_a_points_file_in_the_config_directory():
    """`--file workcell` must land beside taught_points.yaml, not in the cwd.

    A path relative to wherever the pendant was started would scatter a
    cell's points across whatever directories people happened to be in.
    """
    from bartender_teach.point_store import default_points_path, points_path_for
    default_dir = os.path.dirname(default_points_path())
    assert points_path_for('workcell') == os.path.join(
        default_dir, 'workcell_points.yaml')


@pytest.mark.parametrize('arg', ['./workcell.yaml', 'mine.yml', '/tmp/x/points.yaml'])
def test_anything_that_looks_like_a_path_is_a_path(arg):
    from bartender_teach.point_store import points_path_for
    assert points_path_for(arg) == os.path.abspath(arg)


def test_no_file_argument_means_the_default_file():
    from bartender_teach.point_store import default_points_path, points_path_for
    assert points_path_for(None) == default_points_path()


def test_the_shipped_workcell_file_loads_with_home():
    """It is what `--file workcell` opens, and taught points never replace home."""
    from bartender_teach.point_store import points_path_for
    path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'config', 'workcell_points.yaml')
    store = PointStore.load(path, missing_ok=False)
    assert 'home' in store.points
    assert not store.points['home'].pose  # the SRDF state, never taught
    assert store.points['home'].group == 'ur_manipulator'
    assert points_path_for('workcell').endswith('workcell_points.yaml')
