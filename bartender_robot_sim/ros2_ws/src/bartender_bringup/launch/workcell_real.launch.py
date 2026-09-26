"""Bring up the REAL workcell robot over Ethernet: one UR5e + Robotiq 2F-85.

    ros2 launch bartender_bringup workcell_real.launch.py
    ros2 launch bartender_bringup workcell_real.launch.py robot_ip:=<another ip>

Starts ur_robot_driver's hardware interface against this project's own
description (workcell.urdf.xacro: the arm, the gripper and the table), the
driver's helper nodes, the controllers in workcell_ur_controllers.yaml, and
MoveIt with the same SRDF and controller names as workcell_sim.launch.py.

Why not just include ur_robot_driver's ur_control.launch.py: it renders its
description with plain xacro and a fixed argument list, from a package that
must also hold the per-robot config/ files, so it cannot bring up a
description with a gripper and a table on it without copying ur_description
into this repo. This file starts the same nodes that one does (checked
against ur_robot_driver 2.14), with our description and controller names.

use_fake_hardware:=true runs the whole thing with no robot: the arm and
gripper become ros2_control mock hardware that follows whatever it is
commanded. That is the way to check the stack end to end before the robot
is on the network. robot_ip is unused then.

workcell_twin.launch.py includes this file and adds a Gazebo twin that
copies the arm.

BEFORE THE FIRST REAL MOVE, see "Real robot" in the README: the External
Control URCap on the teach pendant, the host IP it calls back to, and the
one workcell number (table_height) nobody has checked yet; arm_yaw is
-1.5708, set from the real cell.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'robot_ip', default_value='10.42.0.100',
            description=('IP address of the UR controller on the Ethernet '
                         'link. 10.42.0.100 is ours; this PC is 10.42.0.67 '
                         'on eno1 (docs/WORKCELL.md).')),
        DeclareLaunchArgument(
            'reverse_ip', default_value='0.0.0.0',
            description=('This PC\'s IP as the robot sees it. 0.0.0.0 lets the '
                         'driver work it out, which is right with one cable '
                         'between the two; set it if this PC has several '
                         'routes to the robot.')),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Mock the arm and gripper; no robot needed.'),
        DeclareLaunchArgument(
            'gripper_fake_hardware', default_value='true',
            description=('Mock the gripper while the arm is real. See '
                         'workcell.urdf.xacro for why this is the default.')),
        DeclareLaunchArgument(
            'gripper_com_port', default_value='/tmp/ttyUR',
            description=('Serial device for the real gripper. /tmp/ttyUR is '
                         'the tool connector, via use_tool_communication.')),
        DeclareLaunchArgument(
            'use_tool_communication', default_value='false',
            description=('Forward the UR tool connector\'s RS-485 to '
                         'tool_device_name. Needs the rs485 URCap.')),
        DeclareLaunchArgument('tool_voltage', default_value='0',
                              description='Tool connector voltage, 0/12/24.'),
        DeclareLaunchArgument('tool_device_name', default_value='/tmp/ttyUR'),
        DeclareLaunchArgument('tool_tcp_port', default_value='54321'),
        DeclareLaunchArgument(
            # true because our robot is kept in Remote Control mode, where
            # nothing can be started from the UR pendant anyway.
            'headless_mode', default_value='true',
            description=('Send the control script without a program on the '
                         'teach pendant (needs Remote Control mode). false '
                         'means the External Control program must be '
                         'started there by hand.')),
        DeclareLaunchArgument('launch_moveit', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='20'),
        DeclareLaunchArgument('table_height', default_value='0.75'),
        DeclareLaunchArgument('arm_x', default_value='0.35'),
        DeclareLaunchArgument('arm_y', default_value='0.35'),
        DeclareLaunchArgument('arm_yaw', default_value='-1.5708'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    pkg_description = get_package_share_directory('bartender_description')
    pkg_moveit_config = get_package_share_directory('bartender_moveit_config')

    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    headless_mode = LaunchConfiguration('headless_mode')
    fake = use_fake_hardware.perform(context) == 'true'

    workcell_args = [
        ' table_height:=', LaunchConfiguration('table_height'),
        ' arm_x:=', LaunchConfiguration('arm_x'),
        ' arm_y:=', LaunchConfiguration('arm_y'),
        ' arm_yaw:=', LaunchConfiguration('arm_yaw'),
    ]

    # Through render_bartender_urdf.py like every other launch, so the
    # robot_state_publisher and MoveIt see the same gripper geometry.
    xacro_file = os.path.join(pkg_description, 'urdf', 'workcell.urdf.xacro')
    render_script = os.path.join(pkg_description, 'scripts', 'render_bartender_urdf.py')
    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='python3'), ' ', render_script, ' ', xacro_file,
        ' sim_ignition:=false',
        ' use_fake_hardware:=', use_fake_hardware,
        ' gripper_fake_hardware:=', LaunchConfiguration('gripper_fake_hardware'),
        ' gripper_com_port:=', LaunchConfiguration('gripper_com_port'),
        ' robot_ip:=', robot_ip,
        ' reverse_ip:=', LaunchConfiguration('reverse_ip'),
        ' headless_mode:=', headless_mode,
        ' script_filename:=', PathJoinSubstitution(
            [FindPackageShare('ur_client_library'), 'resources',
             'external_control.urscript']),
        ' input_recipe_filename:=', PathJoinSubstitution(
            [FindPackageShare('ur_robot_driver'), 'resources', 'rtde_input_recipe.txt']),
        ' output_recipe_filename:=', PathJoinSubstitution(
            [FindPackageShare('ur_robot_driver'), 'resources', 'rtde_output_recipe.txt']),
        ' use_tool_communication:=', LaunchConfiguration('use_tool_communication'),
        ' tool_voltage:=', LaunchConfiguration('tool_voltage'),
        ' tool_device_name:=', LaunchConfiguration('tool_device_name'),
        ' tool_tcp_port:=', LaunchConfiguration('tool_tcp_port'),
    ] + workcell_args), value_type=str)}

    controllers = os.path.join(pkg_description, 'config', 'workcell_ur_controllers.yaml')
    control_parameters = [robot_description, controllers,
                          {'verify_payload_on_set': not fake}]

    nodes = [
        # With a robot: the driver's own controller_manager, which also
        # runs the UR communication. Without: the stock one on mock hardware.
        Node(package='ur_robot_driver', executable='ur_ros2_control_node',
             parameters=control_parameters, output='screen',
             condition=UnlessCondition(use_fake_hardware)),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=control_parameters, output='screen',
             condition=IfCondition(use_fake_hardware)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='both'),
    ]

    if not fake:
        nodes += [
            # Power on, brake release, play/stop, load program: /dashboard_client/*.
            Node(package='ur_robot_driver', executable='dashboard_client',
                 name='dashboard_client', output='screen', emulate_tty=True,
                 parameters=[{'robot_ip': robot_ip},
                             {'receive_timeout': 20.0},
                             {'autoconnect': True}]),
            Node(package='ur_robot_driver', executable='robot_state_helper',
                 name='ur_robot_state_helper', output='screen',
                 parameters=[{'headless_mode': headless_mode},
                             {'robot_ip': robot_ip}]),
            # Stops the motion controllers whenever the program on the robot
            # stops (e-stop, protective stop, someone pressing stop), so a
            # trajectory cannot resume by itself when it is restarted.
            # Everything listed here is left running because none of it
            # moves the arm.
            Node(package='ur_robot_driver', executable='controller_stopper_node',
                 name='controller_stopper', output='screen', emulate_tty=True,
                 parameters=[{'headless_mode': headless_mode},
                             {'joint_controller_active': True},
                             {'consistent_controllers': [
                                 'io_and_status_controller',
                                 'force_torque_sensor_broadcaster',
                                 'joint_state_broadcaster',
                                 'speed_scaling_state_broadcaster',
                                 'tcp_pose_broadcaster',
                                 'ur_configuration_controller',
                                 'gripper_controller',
                             ]}]),
            Node(package='ur_robot_driver', executable='urscript_interface',
                 parameters=[{'robot_ip': robot_ip}], output='screen'),
            ExecuteProcess(
                name='ur_tool_comm',
                condition=IfCondition(LaunchConfiguration('use_tool_communication')),
                cmd=[PathJoinSubstitution([FindPackagePrefix('ur_client_library'), 'lib',
                                           'ur_client_library', 'tool_communication.py']),
                     robot_ip,
                     '--tcp-port', LaunchConfiguration('tool_tcp_port'),
                     '--device-name', LaunchConfiguration('tool_device_name')],
                output='screen'),
        ]

    active = [
        'joint_state_broadcaster',
        'io_and_status_controller',
        'speed_scaling_state_broadcaster',
        'force_torque_sensor_broadcaster',
        'ur_configuration_controller',
        'ur_arm_controller',
        'gripper_controller',
    ]
    # Mock hardware has no TCP pose to broadcast; the driver drops it too.
    if not fake:
        active.insert(4, 'tcp_pose_broadcaster')
    inactive = ['freedrive_mode_controller']

    timeout = LaunchConfiguration('controller_spawner_timeout')
    nodes += [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['--controller-manager-timeout', timeout] + active),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['--controller-manager-timeout', timeout, '--inactive'] + inactive),
    ]

    # MoveIt needs the robot's geometry, not its hardware, so it is always
    # rendered with mock hardware and needs none of the driver's arguments.
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        launch_arguments={
            'use_sim_time': 'false',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'description_file': 'workcell.urdf.xacro',
            'description_args': ['sim_ignition:=false use_fake_hardware:=true']
            + workcell_args,
            'srdf_file': 'workcell.srdf',
            'moveit_controllers_file': 'workcell_moveit_controllers.yaml',
            # The speed slider stretches every move; see move_group.launch.py.
            'execution_duration_monitoring': 'false',
        }.items(),
    ))
    return nodes
