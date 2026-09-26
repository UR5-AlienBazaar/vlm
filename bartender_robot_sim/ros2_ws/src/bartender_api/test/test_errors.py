"""Tests for the error-taxonomy classifier.

Driven by real messages this project's action servers have actually
produced (see docs/CONTROL_API.md and open_action_server.py), not invented
ones, so a rule that only matches its own test fixture would be caught by
nothing.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api import errors                            # noqa: E402


def test_the_control_api_docs_own_example_classifies_correctly():
    """The exact message from docs/CONTROL_API.md's worked example."""
    message = (
        'the opener is not seated on the cap: 280.7mm down '
        '(needs 14) and 344.9mm off centre (allows 6.5)')
    result = errors.classify(message)
    assert result['code'] == 'OBJECT_NOT_SEATED'
    assert result['measurements'] == {
        'depth_mm': 280.7, 'depth_required_mm': 14.0,
        'offset_mm': 344.9, 'offset_allowed_mm': 6.5,
    }
    assert result['retryable'] is True


def test_object_disturbed_extracts_both_numbers():
    message = 'the beer moved 6.0mm while being pushed on (limit 25mm)'
    result = errors.classify(message)
    assert result['code'] == 'OBJECT_DISTURBED'
    assert result['measurements'] == {'shift_mm': 6.0, 'shift_limit_mm': 25.0}
    assert result['retryable'] is True


def test_unknown_bottle_message_classifies_as_unknown_station():
    message = (
        'no capped bottle called "whiskey"; the scene has one, '
        'and it is "beer"')
    assert errors.classify(message)['code'] == 'UNKNOWN_STATION'
    assert errors.classify(message)['retryable'] is False


def test_already_open_is_not_retryable():
    message = (
        'the beer is already open (cap is 313mm off the mouth). '
        "Ignition's DetachableJoint cannot re-attach, so restart "
        'the sim to run this again.')
    result = errors.classify(message)
    assert result['code'] == 'ALREADY_OPEN'
    assert result['retryable'] is False


def test_scene_stale_covers_both_its_real_messages():
    bridge_down = (
        'no model poses on /world/bar_world/dynamic_pose/info '
        '-- is the ros_gz bridge in sim.launch.py running?')
    lost_cap = 'lost sight of the cap'
    assert errors.classify(bridge_down)['code'] == 'SCENE_STALE'
    assert errors.classify(lost_cap)['code'] == 'SCENE_STALE'


def test_gripper_not_following_is_not_retryable():
    message = (
        'arm B: gripper is not moving -- commanded 0.500 rad and the '
        'joint is at 0.000, 0.500 behind, having left 0.000 where the '
        'close began. The pads are 85.0mm apart and nothing 24.0mm wide '
        'can be stopping them; it is the gripper not following.')
    result = errors.classify(message)
    assert result['code'] == 'GRIPPER_NOT_FOLLOWING'
    assert result['retryable'] is False
    assert 'restart' in result['suggest']


def test_grasp_stopped_wide_is_a_different_code_and_is_retryable():
    """The split this project made on purpose: same symptom, opposite fault.

    Conflating these two sent one real investigation (an opener pick
    stopped 49.1mm wide on a 24.0mm shaft) to the wrong half of the robot.
    A classifier that cannot tell them apart reintroduces that bug one
    layer up.
    """
    message = (
        'arm B: fingers closed from 0.000 to 0.345 rad and stopped '
        'there, 49.1mm apart, with the command 0.200 past them. The '
        'gripper is working -- but nothing 24.0mm wide is that far '
        'apart, so they are on something else and the arm is probably '
        'not where it should be.')
    result = errors.classify(message)
    assert result['code'] == 'GRASP_STOPPED_WIDE'
    assert result['retryable'] is True


def test_a_coarse_stage_wrapper_is_stage_failed_not_a_guess():
    """The messages that have NOT been enriched with a specific reason.

    "arm B could not pick up the opener" alone carries no measurement --
    Arm.last_error fixes this for open_action_server's own call sites (see
    open_action_server.py's _why), but a message that genuinely has
    nothing more to say must not be forced into a specific code it cannot
    support.
    """
    result = errors.classify('arm B could not pick up the opener')
    assert result['code'] == 'STAGE_FAILED'
    assert result['retryable'] is True


def test_an_enriched_coarse_message_still_finds_the_specific_code():
    """The _why-enriched version of the same failure, once last_error is set."""
    message = (
        'arm B could not pick up the opener (arm B: fingers closed '
        'from 0.000 to 0.345 rad and stopped there, 49.1mm apart, '
        'with the command 0.200 past them. The gripper is working '
        '-- but nothing 24.0mm wide is that far apart, so they are '
        'on something else and the arm is probably not where it '
        'should be.)')
    assert errors.classify(message)['code'] == 'GRASP_STOPPED_WIDE'


def test_pours_cartesian_plan_fraction_is_plan_failed():
    """pour_action_server's own wording for a short Cartesian plan.

    Written independently of open_action_server's ("only reached X of the
    path" is _follow_cartesian's phrasing in both packages), so this is
    real evidence the rule generalises rather than having been fitted to
    one server's exact sentence.
    """
    message = (
        'failed while pouring cola: tilting_to_pour_cola '
        '(Cartesian plan "cola tilt" only reached 0.83 of the path)')
    result = errors.classify(message)
    assert result['code'] == 'PLAN_FAILED'
    assert result['measurements'] == {'fraction_reached': 0.83}


def test_pours_own_grasp_lost_phrasing_is_recognised():
    """pour_action_server._still_holding_bottle's wording, not open's.

    Added once PourActionServer.last_error started threading this message
    into the execute_callback result -- before that it never reached this
    classifier at all.
    """
    message = (
        'failed while pouring cola: checking_grip_cola '
        '(cola lost: fingers have closed to 0.4400 rad, the commanded '
        '0.4400, so they are now empty)')
    assert errors.classify(message)['code'] == 'GRASP_LOST'


def test_a_genuinely_unrecognised_message_is_unclassified_not_guessed():
    """A wrong code is worse than an honest "do not know".

    Exactly the reasoning that split GRIPPER_NOT_FOLLOWING from
    GRASP_STOPPED_WIDE in the first place, applied to this classifier's
    own default.
    """
    result = errors.classify('the flux capacitor reported negative charm')
    assert result['code'] == 'UNCLASSIFIED'
    assert result['retryable'] is False


def test_every_taxonomy_entry_has_a_meaning_and_a_retryable_flag():
    for code, (meaning, retryable) in errors.TAXONOMY.items():
        assert isinstance(meaning, str) and meaning
        assert isinstance(retryable, bool)


def test_stage_is_carried_through_when_given():
    result = errors.classify('lost sight of the cap', stage='lifting the opener')
    assert result['stage'] == 'lifting the opener'


def test_stage_is_absent_when_not_given():
    assert 'stage' not in errors.classify('lost sight of the cap')
