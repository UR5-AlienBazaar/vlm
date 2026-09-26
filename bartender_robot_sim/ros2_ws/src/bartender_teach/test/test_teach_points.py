"""Tests for the parts of the pendant that do not need a robot.

The quaternion helpers and the command parsing are where a bug would be
silent: a wrong rotation still produces a perfectly valid pose, and the arm
goes somewhere nobody asked for. The motion calls themselves are stubbed --
what is checked here is WHICH pose the pendant asks for, and which commands it
refuses.
"""
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.point_store import Point, PointStore   # noqa: E402
from bartender_teach.teach_points import (                  # noqa: E402
    ARMS, ARM_JOINTS, GRIPPER_OPEN_POS, MAX_JOG_DEG, MAX_JOG_MM, Pendant,
    TeachNode,
)
from bartender_teach.tool_frames import (                   # noqa: E402,I100
    quat_about, quat_mul, quat_rotate,
)

ARM_A = ARMS['a']
ARM_B = ARMS['b']

SAMPLE = dict(zip(ARM_JOINTS, [0.079, -1.905, 2.337, -0.432, 1.65, 0.0]))
IDENTITY = (0.0, 0.0, 0.0, 1.0)
# The side grasp's tool orientation: the cyclic axis permutation that maps
# tool0's local +Z onto base +X, local +X onto base +Y, local +Y onto base +Z.
SIDE_QUAT = (0.5, 0.5, 0.5, 0.5)


# -- quaternion helpers -----------------------------------------------------

def test_identity_rotates_nothing():
    assert quat_rotate(IDENTITY, (1.0, 2.0, 3.0)) == pytest.approx((1, 2, 3))


@pytest.mark.parametrize('axis,vec,expect', [
    ('z', (1, 0, 0), (0, 1, 0)),
    ('z', (0, 1, 0), (-1, 0, 0)),
    ('x', (0, 1, 0), (0, 0, 1)),
    ('y', (0, 0, 1), (1, 0, 0)),
])
def test_quarter_turns_are_right_handed(axis, vec, expect):
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    q = quat_about(axes[axis], math.pi / 2)
    assert quat_rotate(q, vec) == pytest.approx(expect, abs=1e-12)


def test_side_quat_is_the_permutation_pour_action_server_documents():
    """If this fails, `jog tz` runs the tool in the wrong direction."""
    assert quat_rotate(SIDE_QUAT, (0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-12)
    assert quat_rotate(SIDE_QUAT, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-12)
    assert quat_rotate(SIDE_QUAT, (0, 1, 0)) == pytest.approx((0, 0, 1), abs=1e-12)


def test_rotation_preserves_length():
    q = quat_about((1 / math.sqrt(3),) * 3, 0.7)
    v = (0.3, -1.2, 0.8)
    assert (sum(c * c for c in quat_rotate(q, v))
            == pytest.approx(sum(c * c for c in v)))


def test_quat_mul_matches_composed_rotation():
    a = quat_about((0, 0, 1), 0.4)
    b = quat_about((1, 0, 0), -0.9)
    v = (0.2, 0.5, -0.3)
    assert quat_rotate(quat_mul(a, b), v) == pytest.approx(
        quat_rotate(a, quat_rotate(b, v)), abs=1e-12)


# -- a pendant with the robot stubbed out -----------------------------------

class FakeRobot:
    """RobotControl's surface, recording calls instead of making them."""

    def __init__(self, real=True, running=True):
        self.real = real
        self.running = running
        self.freedrive = False
        self.calls = []

    def status(self):
        return {'real': self.real, 'robot_mode': 'RUNNING' if self.real else None,
                'safety_mode': 'NORMAL' if self.real else None,
                'program_running': self.running if self.real else None,
                'speed_scaling': 25.0 if self.real else None,
                'freedrive': self.freedrive}

    def set_mode(self, target, play=False):
        self.calls.append(('set_mode', target))
        return True, ''

    def dashboard(self, verb):
        self.calls.append(('dashboard', verb))
        return True, verb + ' done'

    def resend(self):
        self.calls.append(('resend',))
        return True, 'sent'

    def set_speed(self, fraction):
        self.calls.append(('speed', fraction))
        return True, ''

    def set_freedrive(self, on):
        self.calls.append(('freedrive', on))
        self.freedrive = on
        return True, ''


class FakeNode:
    """Records what it was asked to do instead of doing it.

    Every call records the ARM it was made against as well as its arguments,
    because with two arms "the pendant asked for the right pose" is only half
    the question -- it has to have asked the right arm for it.
    """

    def __init__(self, pose=((0.5, 0.1, 0.3), IDENTITY)):
        self._pose = pose
        self.joint_moves = []
        self.cartesian = []
        self.gripper = []
        self.arms_asked = []
        self.robot = FakeRobot()

    def arm_joints(self, arm=ARM_A):
        self.arms_asked.append(arm.key)
        return dict(zip(arm.joints, SAMPLE.values()))

    def gripper_position(self, arm=ARM_A):
        return 0.1

    def tool_pose(self, joints=None, arm=ARM_A):
        return self._pose

    def move_to_joints(self, positions, label='', arm=ARM_A):
        self.joint_moves.append(dict(positions))
        self.arms_moved = getattr(self, 'arms_moved', [])
        self.arms_moved.append(arm.key)
        return True, ''

    def move_cartesian(self, pose, avoid_collisions=True, label='', arm=ARM_A):
        self.cartesian.append((pose, avoid_collisions))
        self.arms_jogged = getattr(self, 'arms_jogged', [])
        self.arms_jogged.append(arm.key)
        return True, ''

    def command_gripper(self, position, arm=ARM_A):
        self.gripper.append(position)
        self.arms_gripped = getattr(self, 'arms_gripped', [])
        self.arms_gripped.append(arm.key)
        return True, ''


@pytest.fixture
def pendant(tmp_path):
    return Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))


def xyz(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def quat(pose):
    return (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)


# -- jogging ----------------------------------------------------------------

def test_base_jog_moves_the_named_axis_only(pendant):
    pendant.dispatch('jog z 20')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.32))
    assert quat(pose) == pytest.approx(IDENTITY)


def test_jog_is_in_millimetres(pendant):
    pendant.dispatch('jog x 100')
    assert xyz(pendant.node.cartesian[0][0]) == pytest.approx((0.6, 0.1, 0.3))


def test_tool_jog_follows_the_tool_orientation(tmp_path):
    """Tool-frame jogs must follow the tool, not the base.

    The tz axis is the approach direction. Under the side grasp's orientation
    that is base +X, so `jog tz 50` must run the tool 50mm toward the bottle,
    not 50mm upward.
    """
    node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('jog tz 50')
    assert xyz(node.cartesian[0][0]) == pytest.approx((0.55, 0.1, 0.3), abs=1e-9)


def test_rotation_jog_holds_position(pendant):
    pendant.dispatch('jog ry 30')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.3))
    assert quat(pose) == pytest.approx(
        quat_about((0, 1, 0), math.radians(30)), abs=1e-12)


def test_joint_jog_adds_to_that_joint_in_degrees(pendant):
    pendant.dispatch('jog j1 10')
    moved = pendant.node.joint_moves[0]
    assert moved['shoulder_pan_joint'] == pytest.approx(
        SAMPLE['shoulder_pan_joint'] + math.radians(10))
    for j in ARM_JOINTS[1:]:
        assert moved[j] == pytest.approx(SAMPLE[j])


@pytest.mark.parametrize('line', [
    f'jog z {MAX_JOG_MM + 1}',
    f'jog x -{MAX_JOG_MM + 1}',
    f'jog tz {MAX_JOG_MM + 1}',
    f'jog j1 {MAX_JOG_DEG + 1}',
    f'jog rx -{MAX_JOG_DEG + 1}',
])
def test_oversized_jogs_are_refused_not_clamped(pendant, line):
    pendant.dispatch(line)
    assert pendant.node.cartesian == []
    assert pendant.node.joint_moves == []


@pytest.mark.parametrize('line', [
    'jog', 'jog z', 'jog z 1 2', 'jog w 10', 'jog j9 10', 'jog j0 10',
    'jog z banana', 'jog qz 10',
])
def test_bad_jog_commands_move_nothing(pendant, line):
    pendant.dispatch(line)
    assert pendant.node.cartesian == []
    assert pendant.node.joint_moves == []


def test_jog_at_the_limit_is_allowed(pendant):
    pendant.dispatch(f'jog z {MAX_JOG_MM}')
    assert len(pendant.node.cartesian) == 1


# -- safety -----------------------------------------------------------------

def test_cartesian_jogs_are_collision_checked_by_default(pendant):
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[0][1] is True


def test_safety_off_is_passed_through(pendant):
    pendant.dispatch('safety off')
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[0][1] is False
    pendant.dispatch('safety on')
    pendant.dispatch('jog z 10')
    assert pendant.node.cartesian[1][1] is True


def test_bad_safety_argument_does_not_change_the_setting(pendant):
    pendant.dispatch('safety maybe')
    assert pendant.safety is True


# -- saving -----------------------------------------------------------------

def test_save_writes_immediately(pendant):
    pendant.dispatch('save grasp beside the whiskey')
    assert os.path.exists(pendant.store.path)
    point = PointStore.load(pendant.store.path).get('grasp')
    assert point.joints_in_order(ARM_JOINTS) == pytest.approx(
        [SAMPLE[j] for j in ARM_JOINTS])
    assert point.note == 'beside the whiskey'
    assert point.gripper == pytest.approx(0.1)
    assert point.pose['xyz'] == [0.5, 0.1, 0.3]


def test_save_does_not_clobber_but_resave_does(pendant):
    pendant.dispatch('save grasp first')
    pendant.dispatch('save grasp second')
    assert pendant.store.get('grasp').note == 'first'
    pendant.dispatch('resave grasp second')
    assert pendant.store.get('grasp').note == 'second'


def test_save_wraps_branches_out_of_range(tmp_path):
    node = FakeNode()
    node.arm_joints = lambda arm=ARM_A: dict(SAMPLE, wrist_3_joint=7.0)
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('save far')
    assert abs(p.store.get('far').joints['wrist_3_joint']) <= math.pi


def test_save_without_fk_still_records_joints(tmp_path):
    node = FakeNode()
    node.tool_pose = lambda joints=None, arm=ARM_A: None
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('save nofk')
    point = PointStore.load(p.store.path).get('nofk')
    assert point.pose is None
    assert point.joints_in_order(ARM_JOINTS)


def test_rm_persists(pendant):
    pendant.dispatch('save a')
    pendant.dispatch('rm a')
    assert len(PointStore.load(pendant.store.path)) == 0


# -- goto, gripper, dispatch ------------------------------------------------

def test_goto_sends_the_stored_joints(pendant):
    pendant.store.add(Point('target', SAMPLE))
    pendant.dispatch('goto target')
    assert pendant.node.joint_moves[0] == pytest.approx(SAMPLE)


def test_goto_unknown_point_moves_nothing(pendant):
    pendant.dispatch('goto nowhere')
    assert pendant.node.joint_moves == []


def test_goto_incomplete_point_moves_nothing(pendant):
    pendant.store.add(Point('half', {'shoulder_pan_joint': 0.1}))
    pendant.dispatch('goto half')
    assert pendant.node.joint_moves == []


def test_gripper_commands(pendant):
    pendant.dispatch('open')
    pendant.dispatch('close')
    pendant.dispatch('close 0.42')
    assert pendant.node.gripper == pytest.approx(
        [GRIPPER_OPEN_POS, 0.5, 0.42])


def test_open_does_not_command_the_joints_lower_limit(pendant):
    """`open` used to send 0.0, which killed the gripper for the run.

    Goals went on being accepted and the joint never moved again. See
    GRIPPER_LOWER_LIMIT in bartender_open/arm.py for the measurements.
    """
    pendant.dispatch('open')
    assert pendant.node.gripper == [GRIPPER_OPEN_POS]
    assert GRIPPER_OPEN_POS > 0.0


def test_a_gripper_command_on_the_lower_limit_is_refused():
    """Refused rather than clamped, like the jog bounds.

    Driven through the REAL TeachNode.command_gripper rather than the
    pendant's FakeNode, because the fake is what would otherwise be
    under test and it has no bound. The guard returns before it touches
    anything on self, so an empty stand-in is enough.
    """
    ok, why = TeachNode.command_gripper(types.SimpleNamespace(), 0.0)
    assert not ok
    assert 'lower limit' in why
    ok, why = TeachNode.command_gripper(types.SimpleNamespace(), 0.9)
    assert not ok


@pytest.mark.parametrize('line', ['quit', 'exit', 'q', 'QUIT'])
def test_quit_words(pendant, line):
    assert pendant.dispatch(line) is False


@pytest.mark.parametrize('line', ['', '   ', 'help', 'state', 'list', 'file',
                                  'nonsense', 'show', 'rm', 'export',
                                  'save', 'goto', 'jog z "unclosed'])
def test_no_command_can_end_the_session(pendant, line):
    """A typo must never drop the operator out of the pendant.

    Quitting mid-session means re-teaching every point recorded in it.
    """
    assert pendant.dispatch(line) is True


def test_export_emits_a_pastable_snippet(pendant, capsys):
    pendant.store.add(Point('target', SAMPLE))
    pendant.dispatch('export target')
    out = capsys.readouterr().out
    assert 'approach_joints=[' in out
    assert '0.0790' in out and '-1.9050' in out


# -- tool centre points -----------------------------------------------------

def test_default_tool_is_the_flange(pendant):
    assert pendant.tool.name == 'tool0'


def test_tool_lists_without_selecting(pendant, capsys):
    pendant.dispatch('tool')
    out = capsys.readouterr().out
    assert 'whiskey_spout' in out and 'cola_spout' in out
    assert pendant.tool.name == 'tool0'


def test_tool_selects(pendant):
    pendant.dispatch('tool whiskey_spout')
    assert pendant.tool.name == 'whiskey_spout'


def test_unknown_tool_lists_the_alternatives_and_changes_nothing(pendant, capsys):
    pendant.dispatch('tool spanner')
    assert 'whiskey_spout' in capsys.readouterr().out
    assert pendant.tool.name == 'tool0'


def test_rotation_jog_holds_the_selected_tools_tip(tmp_path):
    """The whole request: turn the bottle, keep the spout where it is."""
    from bartender_teach.tool_frames import get_tool, tcp_from_tool0
    node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('tool whiskey_spout')
    tool = get_tool('whiskey_spout')
    before, _ = tcp_from_tool0((0.5, 0.1, 0.3), SIDE_QUAT, tool)

    p.dispatch('jog ry 30')
    pose, _ = node.cartesian[0]
    after, _ = tcp_from_tool0(xyz(pose), quat(pose), tool)
    assert after == pytest.approx(before, abs=1e-9)
    # ...and the flange really did move, so this is not a no-op.
    assert math.dist(xyz(pose), (0.5, 0.1, 0.3)) > 0.02


def test_rotation_jog_with_tool0_is_unchanged(pendant):
    """Adding tool frames must not alter the default behaviour."""
    pendant.dispatch('jog ry 30')
    pose, _ = pendant.node.cartesian[0]
    assert xyz(pose) == pytest.approx((0.5, 0.1, 0.3))


def test_translation_jog_is_the_same_whatever_the_tool(tmp_path):
    moved = []
    for tool in ('tool0', 'whiskey_spout'):
        node = FakeNode(pose=((0.5, 0.1, 0.3), SIDE_QUAT))
        p = Pendant(node, PointStore(str(tmp_path / f'{tool}.yaml')))
        p.dispatch(f'tool {tool}')
        p.dispatch('jog z 20')
        moved.append(xyz(node.cartesian[0][0]))
    assert moved[0] == pytest.approx(moved[1])


def test_saved_point_records_the_active_tool(pendant):
    pendant.dispatch('tool cola_spout')
    pendant.dispatch('save p')
    assert PointStore.load(pendant.store.path).get('p').tool == 'cola_spout'


def test_state_shows_the_tool_tip_when_one_is_selected(pendant, capsys):
    pendant.dispatch('tool whiskey_spout')
    capsys.readouterr()
    pendant.dispatch('state')
    assert 'whiskey_spout' in capsys.readouterr().out


# -- two arms ----------------------------------------------------------------

def test_the_pendant_starts_on_arm_a(pendant):
    """Arm A is the pouring arm and everything that came before is its.

    A pendant that came up on arm B would make every pre-existing habit --
    and every note in pour_action_server -- quietly wrong.
    """
    assert pendant.arm.key == 'a'
    assert pendant.arm.group == 'ur_manipulator'


def test_selecting_arm_b_moves_every_command_to_it(pendant):
    pendant.dispatch('arm b')
    pendant.dispatch('jog j1 5')
    pendant.dispatch('close 0.3')
    assert pendant.node.arms_moved == ['b']
    assert pendant.node.arms_gripped == ['b']
    # ...and it jogged a b_ joint, not arm A's.
    assert all(n.startswith('b_') for n in pendant.node.joint_moves[-1])


def test_an_unknown_arm_is_refused_and_changes_nothing(pendant):
    with pytest.raises(ValueError, match='no arm'):
        pendant.cmd_arm(['c'])
    assert pendant.arm.key == 'a'


def test_arm_b_plans_in_its_own_base_frame(pendant):
    """b_base_link, not base_link.

    The arms are 1.06 x 0.80 apart, so a Cartesian jog sent in the wrong
    frame lands most of a metre away. This is the mistake the Arm record
    exists to make impossible.
    """
    assert ARM_B.frame == 'b_base_link'
    assert ARM_B.eef_link == 'b_tool0'
    assert ARM_A.frame == 'base_link'


def test_goto_uses_the_arm_the_point_was_taught_on(tmp_path):
    """Not the selected arm -- a point belongs to exactly one arm."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('b_rest', dict(zip(ARM_B.joints, [0.0, -2.0, 1.6, -1.17,
                                                      -1.57, 0.0])),
                    group=ARM_B.group))
    p = Pendant(FakeNode(), store)          # selected arm is A
    p.dispatch('goto b_rest')
    assert p.node.arms_moved == ['b']
    assert all(n.startswith('b_') for n in p.node.joint_moves[-1])


def test_a_point_saved_on_arm_b_records_arm_bs_group(tmp_path):
    node = FakeNode()
    p = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('arm b')
    p.dispatch('save over_the_cap')
    saved = PointStore.load(p.store.path).get('over_the_cap')
    assert saved.group == 'b_ur_manipulator'
    assert all(n.startswith('b_') for n in saved.joints)


def test_switching_arms_drops_a_selected_tool(tmp_path):
    """A tool offset is arm A's bottle spout; applying it to arm B is wrong."""
    from bartender_teach.tool_frames import TOOLS
    p = Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))
    p.tool = list(TOOLS.values())[0]
    p.dispatch('arm b')
    assert p.tool.name == 'tool0'


def test_list_shows_both_arms_points(tmp_path, capsys):
    """`list` answers "what does the robot know", so it never hides an arm."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('a_point', dict(zip(ARM_A.joints, [0.1] * 6)),
                    group=ARM_A.group))
    store.add(Point('b_point', dict(zip(ARM_B.joints, [0.2] * 6)),
                    group=ARM_B.group))
    p = Pendant(FakeNode(), store)
    p.dispatch('list')
    out = capsys.readouterr().out
    assert 'a_point' in out and 'b_point' in out
    assert 'arm A' in out and 'arm B' in out


# -- record mode ------------------------------------------------------------
#
# The mode exists so that a sequence gets built out of the teaching somebody
# was doing anyway. So what these check is mostly about what does and does not
# become a step: a pipeline that quietly recorded the wrong things would be
# worse than no pipeline, because it looks like a plan.

def test_recording_starts_empty_and_is_in_the_file_at_once(pendant):
    """Persisted from the start, like a point: a session cannot be redone."""
    pendant.dispatch('record pour_v2')
    assert pendant.recording is not None
    assert 'pour_v2' in PointStore.load(pendant.store.path).pipelines


def test_saving_a_point_while_recording_appends_a_goto(pendant):
    pendant.dispatch('record pour_v2')
    pendant.dispatch('save whiskey_grip')
    assert [s.describe() for s in pendant.recording.steps] == \
        ['goto whiskey_grip']


def test_saving_without_a_name_auto_names_after_the_pipeline(pendant):
    """Jog, save, jog, save -- naming every waypoint is work for nothing."""
    pendant.dispatch('record pour_v2')
    pendant.dispatch('save')
    pendant.dispatch('save')
    assert 'pour_v2_01' in pendant.store
    assert 'pour_v2_02' in pendant.store
    assert pendant.recording.point_names() == ['pour_v2_01', 'pour_v2_02']


def test_auto_naming_skips_a_name_already_taken(pendant):
    """Re-recording must not redefine the points the old one still runs on."""
    pendant.store.add(Point('pour_v2_01', dict(zip(ARM_A.joints, [0.4] * 6))))
    pendant.dispatch('record pour_v2')
    pendant.dispatch('save')
    assert pendant.recording.point_names() == ['pour_v2_02']
    assert pendant.store.get('pour_v2_01').joints[ARM_A.joints[0]] == 0.4


def test_saving_without_a_name_is_still_refused_when_not_recording(pendant):
    pendant.dispatch('save')
    assert not len(pendant.store)


def test_a_jog_never_becomes_a_step(pendant):
    """A relative move cannot replay -- see the pipelines module docstring."""
    pendant.dispatch('record pour_v2')
    pendant.dispatch('jog z 20')
    pendant.dispatch('jog j1 5')
    assert len(pendant.recording) == 0


def test_the_gripper_commands_become_steps_carrying_their_arm(pendant):
    pendant.dispatch('record grab')
    pendant.dispatch('close 0.3')
    pendant.dispatch('open')
    assert [(s.kind, s.arg, s.arm) for s in pendant.recording.steps] == \
        [('grip', 0.3, 'a'), ('grip', GRIPPER_OPEN_POS, 'a')]


def test_a_gripper_step_records_the_selected_arm_not_the_default(tmp_path):
    p = Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('arm b')
    p.dispatch('record grab')
    p.dispatch('close 0.3')
    assert p.recording.steps[0].arm == 'b'


def test_going_to_an_existing_point_appends_a_step(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('home', dict(zip(ARM_A.joints, [0.0] * 6)),
                    group=ARM_A.group))
    p = Pendant(FakeNode(), store)
    p.dispatch('record tidy')
    p.dispatch('goto home')
    assert p.recording.point_names() == ['home']


def test_a_move_that_failed_does_not_become_a_step(tmp_path):
    """A recorded failure is a pipeline known not to work on its first run."""
    class Refuses(FakeNode):
        def move_to_joints(self, positions, label='', arm=ARM_A):
            return False, 'no plan'

    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('home', dict(zip(ARM_A.joints, [0.0] * 6)),
                    group=ARM_A.group))
    p = Pendant(Refuses(), store)
    p.dispatch('record tidy')
    p.dispatch('goto home')
    assert len(p.recording) == 0


def test_a_second_record_is_refused_while_one_is_open(pendant):
    """Two at once would put every step into both."""
    pendant.dispatch('record one')
    pendant.dispatch('record two')
    assert pendant.recording.name == 'one'
    assert 'two' not in pendant.store.pipelines


def test_recording_over_an_existing_name_is_refused(pendant):
    pendant.dispatch('record one')
    pendant.dispatch('save')
    pendant.dispatch('stop')
    pendant.dispatch('record one')
    assert pendant.recording is None
    assert len(pendant.store.pipelines['one']) == 1


def test_stop_discards_a_pipeline_that_recorded_nothing(pendant):
    """Clutter, not data -- and it would block re-recording the name."""
    pendant.dispatch('record empty')
    pendant.dispatch('stop')
    assert pendant.store.pipelines == {}
    assert pendant.recording is None


def test_stop_when_not_recording_says_so_rather_than_crashing(pendant):
    pendant.dispatch('stop')
    assert pendant.recording is None


def test_steps_survive_to_disk_as_they_are_recorded(pendant):
    """Not written on stop: a crash mid-session must not lose the order."""
    pendant.dispatch('record pour_v2')
    pendant.dispatch('save')
    pendant.dispatch('close 0.3')
    reloaded = PointStore.load(pendant.store.path).pipelines['pour_v2']
    assert [s.describe() for s in reloaded.steps] == \
        ['goto pour_v2_01', 'grip 0.300 (arm a)']


def test_dropping_removes_the_last_step_by_default(pendant):
    pendant.dispatch('record p')
    pendant.dispatch('save')
    pendant.dispatch('close 0.3')
    pendant.dispatch('pipeline drop')
    assert [s.kind for s in pendant.recording.steps] == ['goto']


def test_dropping_a_numbered_step_removes_that_one(pendant):
    pendant.dispatch('record p')
    pendant.dispatch('save')
    pendant.dispatch('close 0.3')
    pendant.dispatch('pipeline drop 1')
    assert [s.kind for s in pendant.recording.steps] == ['grip']


def test_dropping_out_of_range_is_refused(pendant):
    pendant.dispatch('record p')
    pendant.dispatch('save')
    pendant.dispatch('pipeline drop 9')
    assert len(pendant.recording) == 1


def test_a_hand_added_goto_to_a_missing_point_is_refused(pendant):
    pendant.dispatch('record p')
    pendant.dispatch('pipeline step goto nowhere')
    assert len(pendant.recording) == 0


def test_a_hand_added_wait_is_appended(pendant):
    pendant.dispatch('record p')
    pendant.dispatch('pipeline step wait 0.5 settle')
    assert pendant.recording.steps[0].describe() == 'wait 0.5s  -- settle'


# -- replay -----------------------------------------------------------------

def _recorded(pendant):
    """Record a two-point, one-grip pipeline and return the node it used."""
    pendant.dispatch('record demo')
    pendant.dispatch('save')
    pendant.dispatch('close 0.3')
    pendant.dispatch('save')
    pendant.dispatch('stop')
    return pendant


def test_running_replays_every_step_in_order(pendant):
    _recorded(pendant)
    pendant.node.joint_moves.clear()
    pendant.node.gripper.clear()
    pendant.dispatch('run demo')
    assert len(pendant.node.joint_moves) == 2
    assert pendant.node.gripper == [0.3]


def test_a_dry_run_moves_nothing(pendant):
    _recorded(pendant)
    pendant.node.joint_moves.clear()
    pendant.node.gripper.clear()
    pendant.dispatch('run demo dry')
    assert pendant.node.joint_moves == [] and pendant.node.gripper == []


def test_running_stops_at_the_first_failure(tmp_path):
    """Carrying on past a failed move drives the rest from the wrong place.

    The failing move is deliberately NOT the last step: a pipeline that
    stopped only because it had run out of steps would pass a test that
    merely counted them.
    """
    class FailsSecondMove(FakeNode):
        def move_to_joints(self, positions, label='', arm=ARM_A):
            self.joint_moves.append(dict(positions))
            return len(self.joint_moves) < 2, 'no plan'

    p = Pendant(FailsSecondMove(), PointStore(str(tmp_path / 'p.yaml')))
    # goto, grip, goto(fails), grip -- the trailing grip must not happen.
    p.dispatch('record demo')
    p.dispatch('save')
    p.dispatch('close 0.3')
    p.dispatch('save')
    p.dispatch('open')
    p.dispatch('stop')
    assert [s.kind for s in p.store.pipelines['demo'].steps] == \
        ['goto', 'grip', 'goto', 'grip']

    p.node.joint_moves.clear()
    p.node.gripper.clear()
    p.dispatch('run demo')
    assert len(p.node.joint_moves) == 2      # stopped on the second move
    assert p.node.gripper == [0.3]           # the grip AFTER it never ran


def test_running_while_recording_is_refused(pendant):
    """The replay would be appended to the pipeline being recorded."""
    _recorded(pendant)
    pendant.dispatch('record another')
    pendant.node.joint_moves.clear()
    pendant.dispatch('run demo')
    assert pendant.node.joint_moves == []


def test_running_is_refused_when_a_point_has_been_deleted(pendant):
    """Checked before anything moves, not at step 9 of 11."""
    _recorded(pendant)
    pendant.dispatch('rm demo_01')
    pendant.node.joint_moves.clear()
    pendant.dispatch('run demo')
    assert pendant.node.joint_moves == []


def test_running_an_unknown_pipeline_says_which_exist(pendant, capsys):
    _recorded(pendant)
    capsys.readouterr()
    pendant.dispatch('run nope')
    assert 'demo' in capsys.readouterr().out


def test_a_grip_step_drives_the_arm_it_recorded(tmp_path):
    """Not the selected one: the other arm may be holding the bottle."""
    p = Pendant(FakeNode(), PointStore(str(tmp_path / 'p.yaml')))
    p.dispatch('arm b')
    p.dispatch('record grab')
    p.dispatch('close 0.3')
    p.dispatch('stop')
    p.dispatch('arm a')
    p.dispatch('run grab')
    assert p.node.arms_gripped[-1] == 'b'


def test_a_goto_step_drives_the_arm_the_point_was_taught_on(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('b_spot', dict(zip(ARM_B.joints, [0.2] * 6)),
                    group=ARM_B.group))
    p = Pendant(FakeNode(), store)
    p.dispatch('record go')
    p.dispatch('goto b_spot')
    p.dispatch('stop')
    p.dispatch('run go')
    assert p.node.arms_moved[-1] == 'b'


def test_a_wait_step_does_not_move_anything(pendant):
    pendant.dispatch('record pause')
    pendant.dispatch('pipeline step wait 0')
    pendant.dispatch('stop')
    pendant.dispatch('run pause')
    assert pendant.node.joint_moves == [] and pendant.node.gripper == []


# -- listing and export -----------------------------------------------------

def test_pipeline_list_names_the_one_being_recorded(pendant, capsys):
    pendant.dispatch('record live')
    pendant.dispatch('save')
    capsys.readouterr()
    pendant.dispatch('pipeline')
    assert 'recording' in capsys.readouterr().out


def test_removing_the_pipeline_being_recorded_is_refused(pendant):
    pendant.dispatch('record live')
    pendant.dispatch('save')
    pendant.dispatch('pipeline rm live')
    assert 'live' in pendant.store.pipelines


def test_export_prints_a_python_literal_of_the_steps(pendant, capsys):
    _recorded(pendant)
    capsys.readouterr()
    pendant.dispatch('pipeline export demo')
    out = capsys.readouterr().out
    assert 'DEMO = [' in out
    assert "('goto', 'demo_01')," in out
    assert "('grip', 0.3000, 'a')," in out


def test_an_unknown_pipeline_subcommand_is_refused(pendant, capsys):
    pendant.dispatch('pipeline frobnicate')
    assert 'frobnicate' in capsys.readouterr().out


# -- the real robot's controls ----------------------------------------------

def test_robot_on_and_off_go_through_set_mode(pendant):
    from bartender_teach.robot_control import MODE_POWER_OFF, MODE_RUNNING
    pendant.dispatch('robot on')
    pendant.dispatch('robot off')
    assert pendant.node.robot.calls == [('set_mode', MODE_RUNNING),
                                        ('set_mode', MODE_POWER_OFF)]


@pytest.mark.parametrize('verb', ['play', 'pause', 'stop', 'unlock'])
def test_program_verbs_go_to_the_dashboard(pendant, verb):
    pendant.dispatch(f'robot {verb}')
    assert pendant.node.robot.calls == [('dashboard', verb)]


def test_an_unknown_robot_verb_does_nothing(pendant, capsys):
    pendant.dispatch('robot explode')
    assert pendant.node.robot.calls == []
    assert 'explode' in capsys.readouterr().out


def test_robot_status_in_the_simulation_says_so(tmp_path, capsys):
    node = FakeNode()
    node.robot = FakeRobot(real=False)
    Pendant(node, PointStore(str(tmp_path / 'p.yaml'))).dispatch('robot')
    assert 'simulation' in capsys.readouterr().out


def test_robot_status_on_the_real_robot(pendant, capsys):
    pendant.dispatch('robot')
    out = capsys.readouterr().out
    assert 'RUNNING' in out and 'NORMAL' in out and '25%' in out


def test_speed_is_percent(pendant):
    pendant.dispatch('speed 25')
    pendant.dispatch('speed 100%')
    assert pendant.node.robot.calls == [('speed', 0.25), ('speed', 1.0)]


@pytest.mark.parametrize('line', ['speed 0', 'speed -5', 'speed 250', 'speed x'])
def test_bad_speeds_are_refused_not_clamped(pendant, line):
    pendant.dispatch(line)
    assert pendant.node.robot.calls == []


def test_freedrive_refuses_every_motion_command(pendant):
    pendant.store.add(Point('target', SAMPLE))
    pendant.dispatch('freedrive on')
    for line in ('jog z 10', 'jog j1 5', 'goto target'):
        pendant.dispatch(line)
    assert pendant.node.cartesian == []
    assert pendant.node.joint_moves == []


def test_freedrive_does_not_stop_saving(pendant):
    """Saving where your hands put the arm is what freedrive is for."""
    pendant.dispatch('freedrive on')
    pendant.dispatch('save byhand')
    assert 'byhand' in pendant.store


def test_freedrive_off_gives_motion_back(pendant):
    pendant.dispatch('freedrive on')
    pendant.dispatch('freedrive off')
    pendant.dispatch('jog z 10')
    assert len(pendant.node.cartesian) == 1


def test_power_off_is_refused_in_freedrive(pendant):
    pendant.dispatch('freedrive on')
    pendant.dispatch('robot off')
    assert ('set_mode', 3) not in pendant.node.robot.calls


def test_leaving_the_pendant_ends_freedrive(pendant):
    pendant.dispatch('freedrive on')
    pendant.release()
    assert pendant.node.robot.freedrive is False
    assert pendant.node.robot.calls[-1] == ('freedrive', False)


def test_motion_is_refused_while_the_robot_program_is_stopped(tmp_path, capsys):
    """Otherwise MoveIt plans, then fails with a bare error_code -4."""
    node = FakeNode()
    node.robot = FakeRobot(running=False)
    pendant = Pendant(node, PointStore(str(tmp_path / 'p.yaml')))
    pendant.store.add(Point('target', SAMPLE))
    for line in ('jog z 10', 'jog j1 5', 'goto target'):
        pendant.dispatch(line)
    assert node.cartesian == [] and node.joint_moves == []
    assert 'robot resend' in capsys.readouterr().out


def test_motion_in_the_simulation_is_not_held_up_by_robot_status(tmp_path):
    node = FakeNode()
    node.robot = FakeRobot(real=False)
    Pendant(node, PointStore(str(tmp_path / 'p.yaml'))).dispatch('jog z 10')
    assert len(node.cartesian) == 1


def test_robot_resend(pendant):
    pendant.dispatch('robot resend')
    assert pendant.node.robot.calls == [('resend',)]
