"""Tests for the arm controllers' goal tolerances.

These exist because their ABSENCE was a silent, system-wide defect rather
than a missing nicety.

joint_trajectory_controller defaults every per-joint tolerance to 0.0, and
0.0 means NOT CHECKED. With only `goal_time` and
`stopped_velocity_tolerance` set -- which is how this file shipped -- the
goal check passed trivially and the controller reported SUCCEEDED wherever
it happened to stop. Measured, driving arm A's wrist 250mm into the bar top
returned SUCCEEDED with error_code 0 and wrist_1 0.756 rad from its goal;
at the skill level the same defect put a flange 267mm from where it was
sent after MoveIt had planned the path at 100%.

Nothing above the controller can recover from that, because every layer is
told the move worked. So the tolerances are load-bearing, and a test that
notices if they are removed or loosened is worth more than its length.
"""
import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(HERE, os.pardir, 'config')

# (file, controller). The two-armed bar, then the one-armed workcell in sim
# and on the real robot. The workcell files carry the same tolerances for
# the same reason; the real one is where a silent success would cost most.
ARM_CONTROLLERS = (
    ('controllers.yaml', 'ur_arm_controller'),
    ('controllers.yaml', 'b_ur_arm_controller'),
    ('workcell_controllers.yaml', 'ur_arm_controller'),
    ('workcell_ur_controllers.yaml', 'ur_arm_controller'),
)
UR_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')

# Measured on the running sim, both arms, 4s down to 0.6s moves: free-space
# goal error never exceeded 0.0009 rad. Anything at or below this would
# start failing honest moves.
WORST_HONEST_ERROR = 0.0009
# ...and the jam it has to catch reached 0.756 rad.
MEASURED_JAM = 0.756


def load(name):
    with open(os.path.join(CONFIG_DIR, name)) as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope='module')
def config():
    return load('controllers.yaml')


def params(config, controller):
    if isinstance(controller, tuple):
        config, controller = load(controller[0]), controller[1]
    return config[controller]['ros__parameters']


def prefix_of(controller):
    name = controller[1] if isinstance(controller, tuple) else controller
    return 'b_' if name.startswith('b_') else ''


@pytest.mark.parametrize('controller', ARM_CONTROLLERS)
def test_every_arm_joint_has_a_goal_tolerance(config, controller):
    """A joint with no entry is a joint the controller will not check."""
    constraints = params(config, controller)['constraints']
    prefix = prefix_of(controller)
    for joint in UR_JOINTS:
        name = prefix + joint
        assert name in constraints, (
            f'{controller} has no goal tolerance for {name}; it will report '
            f'success however far that joint stops from its target')
        assert 'goal' in constraints[name]


@pytest.mark.parametrize('controller', ARM_CONTROLLERS)
def test_the_tolerance_is_above_the_worst_honest_error(config, controller):
    """Tight enough is not the only risk; too tight fails good moves."""
    constraints = params(config, controller)['constraints']
    prefix = prefix_of(controller)
    for joint in UR_JOINTS:
        goal = constraints[prefix + joint]['goal']
        assert goal > WORST_HONEST_ERROR * 2, (
            f'{prefix + joint} goal tolerance {goal} is too close to the '
            f'{WORST_HONEST_ERROR} rad a free-space move actually settles to')


@pytest.mark.parametrize('controller', ARM_CONTROLLERS)
def test_the_tolerance_is_well_below_a_real_jam(config, controller):
    """It has to catch the failure it was added for, with margin."""
    constraints = params(config, controller)['constraints']
    prefix = prefix_of(controller)
    for joint in UR_JOINTS:
        goal = constraints[prefix + joint]['goal']
        assert goal < MEASURED_JAM / 10.0, (
            f'{prefix + joint} goal tolerance {goal} is not far enough below '
            f'the {MEASURED_JAM} rad jam this exists to catch')


@pytest.mark.parametrize('controller', ARM_CONTROLLERS)
def test_no_trajectory_tolerance_is_set(config, controller):
    """Deliberately absent, and it would break recovery if it were not.

    A `trajectory:` tolerance polices |desired - actual| DURING the move.
    Measured: free moves peak at 0.0045 (4s) to 0.0299 (0.6s), a real pour
    reaches 0.74 on the elbow, a jam 1.35 -- and driving back OUT of a jam
    peaked at 1.95, because the trajectory starts from a setpoint left
    inside the worktop. Any value that caught the jam would also abort the
    move that recovers from it, which is worse.
    """
    constraints = params(config, controller)['constraints']
    prefix = prefix_of(controller)
    for joint in UR_JOINTS:
        assert 'trajectory' not in constraints[prefix + joint], (
            f'{prefix + joint} has a trajectory tolerance; see this test\'s '
            f'docstring for why that breaks recovery from a jam')


@pytest.mark.parametrize('controller', ARM_CONTROLLERS)
def test_goal_time_leaves_room_to_settle(config, controller):
    """Zero would demand the tolerance be met the instant the path ends."""
    assert params(config, controller)['constraints']['goal_time'] > 0.0


def test_both_arms_are_configured_identically(config):
    """The arms differ only by joint prefix; a tolerance that differed would
    make one arm quietly more forgiving than the other.
    """
    tolerances = []
    for controller in ('ur_arm_controller', 'b_ur_arm_controller'):
        constraints = params(config, controller)['constraints']
        prefix = prefix_of(controller)
        tolerances.append([constraints[prefix + j]['goal'] for j in UR_JOINTS])
    assert tolerances[0] == tolerances[1]


def test_the_workcell_matches_the_bar():
    """The workcell's arm is arm A, and its tolerances came from there.

    A divergence would be one of the two drifting, not a decision.
    """
    bar = params(None, ('controllers.yaml', 'ur_arm_controller'))
    for name in ('workcell_controllers.yaml', 'workcell_ur_controllers.yaml'):
        arm = params(None, (name, 'ur_arm_controller'))
        assert ([arm['constraints'][j]['goal'] for j in UR_JOINTS]
                == [bar['constraints'][j]['goal'] for j in UR_JOINTS]), name
