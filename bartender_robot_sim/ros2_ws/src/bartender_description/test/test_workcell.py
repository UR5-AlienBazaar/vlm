"""Tests for the workcell description: one UR5e on the 1.40 x 0.70 table.

This is the description the REAL robot is brought up from, so the things
held here are the ones that would be wrong on hardware without anything
failing loudly: where the table is relative to the arm (MoveIt's only idea
of it), and which hardware interface each mode ends up talking to.
"""
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
RENDER = os.path.join(PACKAGE, 'scripts', 'render_bartender_urdf.py')
XACRO = os.path.join(PACKAGE, 'urdf', 'workcell.urdf.xacro')


def rendered(*args):
    result = subprocess.run([sys.executable, RENDER, XACRO] + list(args),
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def plugins(robot):
    return sorted(block.findtext('hardware/plugin')
                  for block in robot.findall('ros2_control'))


def joint_to(robot, child):
    return next(joint for joint in robot.findall('joint')
                if joint.find('child').get('link') == child)


@pytest.fixture(scope='module')
def sim():
    return rendered('sim_ignition:=true')


def test_there_is_one_arm(sim):
    names = {link.get('name') for link in sim.findall('link')}
    assert 'base_link' in names and 'tool0' in names
    assert not any(name.startswith('b_') for name in names)


def test_the_arm_stands_at_35_35_on_the_table(sim):
    """The table's corner is `world`; the base is 0.35 along and across."""
    xyz = [float(v) for v in joint_to(sim, 'base_link').find('origin').get('xyz').split()]
    assert xyz == pytest.approx([0.35, 0.35, 0.0])


def test_the_table_is_140_by_70_from_the_corner(sim):
    """And it runs from the corner in +x and +y, so the arm is on it."""
    table = next(link for link in sim.findall('link')
                 if link.get('name') == 'workcell_table')
    collision = table.find('collision')
    size = [float(v) for v in collision.find('geometry/box').get('size').split()]
    centre = [float(v) for v in collision.find('origin').get('xyz').split()]
    assert size[:2] == pytest.approx([1.40, 0.70])
    assert centre[:2] == pytest.approx([0.70, 0.35])
    # Top surface at z = 0, where the arm's base sits.
    assert centre[2] + size[2] / 2 == pytest.approx(0.0)


def test_sim_drives_everything_through_gazebo(sim):
    assert plugins(sim) == ['ign_ros2_control/IgnitionSystem'] * 2


def test_real_drives_the_arm_and_mocks_the_gripper():
    """The default until the gripper's wiring is decided; see the xacro."""
    robot = rendered('sim_ignition:=false', 'robot_ip:=192.0.2.1')
    assert plugins(robot) == ['mock_components/GenericSystem',
                              'ur_robot_driver/URPositionHardwareInterface']
    ip = robot.find(".//ros2_control/hardware/param[@name='robot_ip']")
    assert ip.text == '192.0.2.1'


def test_the_real_gripper_is_opt_in():
    robot = rendered('sim_ignition:=false', 'gripper_fake_hardware:=false')
    assert 'robotiq_driver/RobotiqGripperHardwareInterface' in plugins(robot)


def test_fake_hardware_mocks_both():
    """What MoveIt and the no-robot dry run are rendered with."""
    robot = rendered('sim_ignition:=false', 'use_fake_hardware:=true')
    assert plugins(robot) == ['mock_components/GenericSystem'] * 2


def test_the_gz_plugin_loads_the_one_arm_controllers(sim):
    parameters = [plugin.findtext('parameters') for gazebo in sim.findall('gazebo')
                  for plugin in gazebo.findall('plugin')
                  if 'gz_ros2_control' in (plugin.get('filename') or '')]
    assert len(parameters) == 1
    assert parameters[0].endswith('workcell_controllers.yaml')


def test_the_plain_sim_has_no_namespace(sim):
    plugin = sim.find(".//gazebo/plugin[@filename='gz_ros2_control-system']")
    assert plugin.find('ros') is None


def test_the_twin_is_namespaced_and_only_follows():
    """workcell_twin.launch.py runs this next to the real robot.

    Without the namespace its controller_manager would collide with the real
    one; with the plain sim's controllers it could be sent goals of its own.
    """
    twin = rendered('sim_ignition:=true', 'ros_namespace:=/twin',
                    'sim_controllers:=workcell_twin_controllers.yaml')
    plugin = twin.find(".//gazebo/plugin[@filename='gz_ros2_control-system']")
    assert plugin.findtext('ros/namespace') == '/twin'
    assert plugin.findtext('parameters').endswith(
        'config/workcell_twin_controllers.yaml')


def test_the_twin_controller_covers_the_arm_and_the_knuckle():
    import yaml
    with open(os.path.join(PACKAGE, 'config',
                           'workcell_twin_controllers.yaml')) as fh:
        config = yaml.safe_load(fh)
    joints = config['/**/twin_position_controller']['ros__parameters']['joints']
    assert joints == ['shoulder_pan_joint', 'shoulder_lift_joint',
                      'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                      'wrist_3_joint', 'robotiq_85_left_knuckle_joint']
    # No trajectory or gripper action: nothing the twin could be sent a goal on.
    controllers = config['/**/controller_manager']['ros__parameters']
    assert set(controllers) - {'update_rate'} == {
        'joint_state_broadcaster', 'twin_position_controller'}
