"""Tests for the pipeline model: what a step may be, and what it refuses.

No robot. What is checked here is that a Step which EXISTS is one the pendant
can run -- every bound and every shape is enforced in the constructor, so the
places that execute steps do not each have to re-check, and a hand-edited
file is rejected at load with the position of the mistake rather than
halfway through moving the arm.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.pipelines import (            # noqa: E402
    MAX_GRIP_RAD, MAX_WAIT_S, MIN_GRIP_RAD, MIN_WAIT_S, Pipeline,
    PipelineError, STEP_KINDS, Step,
)
from bartender_teach.point_store import Point, PointStore   # noqa: E402

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
SAMPLE = dict(zip(ARM, [0.079, -1.905, 2.337, -0.432, 1.65, 0.0]))


# -- what a step may be -----------------------------------------------------

def test_the_three_kinds_are_the_ones_the_pendant_runs():
    """If a kind is added here without an executor, a file can outrun it."""
    assert STEP_KINDS == ('goto', 'grip', 'wait')


def test_a_goto_keeps_its_point_name():
    assert Step('goto', 'whiskey_approach').arg == 'whiskey_approach'


def test_a_goto_name_is_stripped():
    assert Step('goto', '  home \n').arg == 'home'


@pytest.mark.parametrize('empty', ['', '   ', None])
def test_a_goto_to_nothing_is_refused(empty):
    with pytest.raises(PipelineError):
        Step('goto', empty)


def test_an_unknown_kind_is_refused_and_lists_the_known_ones():
    with pytest.raises(PipelineError) as exc:
        Step('jog', 5.0)
    assert 'goto' in str(exc.value)


@pytest.mark.parametrize('kind,value', [('grip', MIN_GRIP_RAD),
                                        ('grip', MAX_GRIP_RAD),
                                        ('wait', MIN_WAIT_S),
                                        ('wait', MAX_WAIT_S)])
def test_the_bounds_themselves_are_allowed(kind, value):
    assert Step(kind, value).arg == value


def test_a_grip_step_may_not_ask_for_the_joints_lower_limit():
    """Asking for 0 is what kills the gripper for the rest of the run.

    It reads like the obvious way to write "open", which is exactly why
    it has to be refused rather than quietly turned into MIN_GRIP_RAD:
    the file is what a person reads to find out what the robot will do.
    See GRIPPER_LOWER_LIMIT in bartender_open/arm.py.
    """
    assert MIN_GRIP_RAD > 0.0
    with pytest.raises(PipelineError) as exc:
        Step('grip', 0.0)
    assert f'{MIN_GRIP_RAD:g}' in str(exc.value)


@pytest.mark.parametrize('kind,value', [
    ('grip', -0.01), ('grip', MAX_GRIP_RAD + 0.01),
    ('wait', -1.0), ('wait', MAX_WAIT_S + 1.0),
])
def test_out_of_range_numbers_are_refused_not_clamped(kind, value):
    """Clamping would make the file say one thing and the robot do another."""
    with pytest.raises(PipelineError):
        Step(kind, value)


@pytest.mark.parametrize('kind', ['grip', 'wait'])
def test_a_bool_is_not_a_number(kind):
    """`grip: true` is plausible YAML, and would arrive as 1.0 rad."""
    with pytest.raises(PipelineError):
        Step(kind, True)


@pytest.mark.parametrize('kind', ['grip', 'wait'])
def test_a_string_is_not_a_number(kind):
    with pytest.raises(PipelineError):
        Step(kind, 'quite a lot')


def test_a_grip_step_can_name_its_arm():
    assert Step('grip', 0.3, arm='b').arm == 'b'


def test_a_step_with_no_arm_says_so_rather_than_guessing():
    assert Step('goto', 'home').arm is None


@pytest.mark.parametrize('bad', ['', '   ', 7])
def test_an_unusable_arm_is_refused(bad):
    with pytest.raises(PipelineError):
        Step('grip', 0.3, arm=bad)


# -- how a step reads -------------------------------------------------------

def test_describe_is_the_command_you_would_type():
    assert Step('goto', 'home').describe() == 'goto home'


def test_a_grip_describes_which_arm_it_drives():
    assert 'arm b' in Step('grip', 0.25, arm='b').describe()


def test_a_note_is_appended_to_the_description():
    assert Step('wait', 0.5, 'settle').describe().endswith('-- settle')


# -- round trip -------------------------------------------------------------

@pytest.mark.parametrize('step', [
    Step('goto', 'home'),
    Step('grip', 0.25, arm='a'),
    Step('grip', 0.25, 'clamp', 'b'),
    Step('wait', 1.5),
])
def test_a_step_survives_a_round_trip(step):
    assert Step.from_dict(step.to_dict()) == step


def test_a_step_with_no_note_or_arm_writes_neither():
    """An unset field must not appear, or every diff carries empty keys."""
    assert Step.from_dict({'goto': 'home'}).to_dict() == {'goto': 'home'}


def test_a_step_naming_two_kinds_is_refused():
    with pytest.raises(PipelineError):
        Step.from_dict({'goto': 'home', 'wait': 1.0})


def test_a_step_naming_no_kind_is_refused():
    with pytest.raises(PipelineError):
        Step.from_dict({'note': 'just a note'})


def test_an_unexpected_key_is_refused_rather_than_ignored():
    """A typo'd key that was ignored would silently drop what it meant."""
    with pytest.raises(PipelineError) as exc:
        Step.from_dict({'goto': 'home', 'armm': 'b'})
    assert 'armm' in str(exc.value)


def test_a_step_that_is_not_a_mapping_is_refused():
    with pytest.raises(PipelineError):
        Step.from_dict(['goto', 'home'])


# -- pipelines --------------------------------------------------------------

def test_a_pipeline_counts_its_steps():
    assert len(Pipeline('p', [Step('wait', 1.0), Step('goto', 'home')])) == 2


def test_point_names_are_in_order_and_keep_repeats():
    """A pipeline may legitimately visit the same point twice."""
    p = Pipeline('p', [Step('goto', 'a'), Step('grip', 0.1),
                       Step('goto', 'b'), Step('goto', 'a')])
    assert p.point_names() == ['a', 'b', 'a']


def test_missing_points_names_only_what_is_absent(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('here', SAMPLE))
    p = Pipeline('p', [Step('goto', 'here'), Step('goto', 'gone'),
                       Step('goto', 'gone')])
    assert p.missing_points(store) == ['gone']


def test_a_pipeline_of_gripper_steps_needs_no_points(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    assert Pipeline('p', [Step('grip', 0.2, arm='a')]).missing_points(store) == []


def test_a_pipeline_survives_a_round_trip():
    p = Pipeline('pour', [Step('goto', 'home'), Step('grip', 0.2, arm='a')],
                 note='spirit first')
    back = Pipeline.from_dict('pour', p.to_dict())
    assert back.name == 'pour' and back.note == 'spirit first'
    assert back.steps == p.steps


def test_a_pipeline_with_no_steps_loads_as_empty():
    """Written before its points are taught; not an error."""
    assert len(Pipeline.from_dict('p', {'note': 'later'})) == 0


def test_steps_that_are_not_a_list_are_refused():
    with pytest.raises(PipelineError):
        Pipeline.from_dict('p', {'steps': 7})


def test_a_pipeline_that_is_not_a_mapping_is_refused():
    with pytest.raises(PipelineError):
        Pipeline.from_dict('p', ['goto home'])


def test_a_bad_step_is_reported_with_its_position():
    """A bare bound message is useless in a file with thirty steps."""
    with pytest.raises(PipelineError) as exc:
        Pipeline.from_dict('pour', {'steps': [{'goto': 'ok'}, {'grip': 9.0}]},
                           'points.yaml')
    message = str(exc.value)
    assert 'step 2' in message and 'pour' in message
