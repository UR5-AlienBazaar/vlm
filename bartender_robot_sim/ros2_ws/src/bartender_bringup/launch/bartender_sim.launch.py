"""Bring up the whole stack.

Gazebo Sim, the bar scene, the two-armed robot, its four ros2_control
controller spawners, MoveIt move_group, and both skill action servers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bartender_gazebo')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo server only, no GUI.',
    )
    headless_rendering_arg = DeclareLaunchArgument(
        'headless_rendering', default_value='false',
        description='With headless:=true, render the cameras through EGL.',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'headless_rendering': LaunchConfiguration('headless_rendering'),
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    ur_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_arm_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Arm B. One controller_manager owns all four controllers -- both arms
    # are links of the same spawned model -- so these are spawned against the
    # same node and differ only in name.
    b_ur_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['b_ur_arm_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    b_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['b_gripper_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Give Gazebo + the spawned robot's controller_manager time to come up
    # before asking it to spawn controllers, rather than racing it.
    #
    # 12s rather than 5s because the scene loads several bottle meshes (12MB
    # between them, up from 5MB): at 5s the spawners fired before
    # gz_ros2_control had registered its interfaces and all three died with
    # "Failed loading controller", leaving a sim that looks up but cannot
    # move. --controller-manager-timeout makes each spawner wait rather than
    # give up, so this stays a head start and not a deadline.
    delayed_controller_spawners = TimerAction(
        period=12.0,
        actions=[
            joint_state_broadcaster_spawner,
            ur_arm_controller_spawner,
            gripper_controller_spawner,
            b_ur_arm_controller_spawner,
            b_gripper_controller_spawner,
        ],
    )

    pour_action_server = Node(
        package='bartender_pour',
        executable='pour_action_server',
        output='screen',
    )

    open_action_server = Node(
        package='bartender_open',
        executable='open_action_server',
        output='screen',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bartender_moveit_config'),
                         'launch', 'move_group.launch.py')
        ),
        # RViz's MotionPlanning display currently throws a kinematics
        # parameter type error on startup here (rviz-only; move_group
        # itself loads the same kinematics.yaml fine) -- off by default
        # until that's tracked down. Pass launch_rviz:=true to re-enable.
        launch_arguments={'launch_rviz': 'false'}.items(),
    )
    # move_group needs the ur_arm_controller/gripper_controller action
    # servers to exist before it will accept execution requests.
    delayed_move_group = TimerAction(period=16.0, actions=[move_group])

    return LaunchDescription([
        headless_arg,
        headless_rendering_arg,
        sim,
        delayed_controller_spawners,
        pour_action_server,
        open_action_server,
        delayed_move_group,
    ])
