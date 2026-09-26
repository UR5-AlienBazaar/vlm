"""Bring up the workcell in simulation: one UR5e on the 1.40 x 0.70 table.

Gazebo Sim with the workcell world, the one-armed description, its three
controllers and MoveIt. The same description, SRDF and MoveIt controller
names as workcell_real.launch.py, so whatever is taught or tuned here is
addressed the same way on the real robot.

The table's layout (the arm at 0.35, 0.35 from its corner) and the two
numbers still to be checked against the real cell are in
bartender_description/urdf/workcell.urdf.xacro.

No skills are started: bartender_pour's stations are the bar's, not this
table's.
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
    pkg_moveit_config = get_package_share_directory('bartender_moveit_config')

    table_height = LaunchConfiguration('table_height')
    arguments = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only, no GUI.'),
        DeclareLaunchArgument('launch_rviz', default_value='false',
                              description=('RViz with the MotionPlanning display. '
                                           'Off by default for the reason given in '
                                           'bartender_sim.launch.py.')),
        DeclareLaunchArgument('table_height', default_value='0.75'),
        DeclareLaunchArgument('arm_x', default_value='0.35'),
        DeclareLaunchArgument('arm_y', default_value='0.35'),
        DeclareLaunchArgument('arm_yaw', default_value='-1.5708'),
    ]

    # Gazebo and MoveIt must be given the same table and the same arm pose,
    # so both are rendered from this one argument string.
    description_args = [
        'sim_ignition:=true',
        ' table_height:=', table_height,
        ' arm_x:=', LaunchConfiguration('arm_x'),
        ' arm_y:=', LaunchConfiguration('arm_y'),
        ' arm_yaw:=', LaunchConfiguration('arm_yaw'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': 'workcell_world.sdf',
            'description_file': 'workcell.urdf.xacro',
            'description_args': description_args,
            # The description's `world` link is the table's corner on its
            # top surface, so it goes at the table's height above the floor.
            'spawn_x': '0.0',
            'spawn_y': '0.0',
            'spawn_z': table_height,
        }.items(),
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager-timeout', '60'],
            output='screen',
        )

    # Same head start as bartender_sim.launch.py, for the same reason.
    delayed_controller_spawners = TimerAction(
        period=12.0,
        actions=[
            spawner('joint_state_broadcaster'),
            spawner('ur_arm_controller'),
            spawner('gripper_controller'),
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'description_file': 'workcell.urdf.xacro',
            'description_args': description_args,
            'srdf_file': 'workcell.srdf',
            'moveit_controllers_file': 'workcell_moveit_controllers.yaml',
        }.items(),
    )
    delayed_move_group = TimerAction(period=16.0, actions=[move_group])

    return LaunchDescription(arguments + [
        sim,
        delayed_controller_spawners,
        delayed_move_group,
    ])
