"""Start MoveIt's move_group node, and optionally RViz.

Serves both arms, against the controllers spawned by
bartender_bringup/launch/bartender_sim.launch.py. The workcell launches
(one arm, sim or real) point it at their own description, SRDF and
controller list through the launch arguments below; the defaults are the
two-armed bar.

Hand-written instead of MoveIt-Setup-Assistant-generated -- see
bartender_moveit_config/srdf/bartender.srdf for why -- but follows the same
structure/parameter set the Assistant's generated launch file would use.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_share_dir, relative_path):
    with open(os.path.join(package_share_dir, relative_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'description_file', default_value='bartender.urdf.xacro',
            description='xacro in bartender_description/urdf.'),
        DeclareLaunchArgument(
            'description_args', default_value='sim_ignition:=true',
            description='Arguments passed to that xacro.'),
        DeclareLaunchArgument(
            'srdf_file', default_value='bartender.srdf',
            description='SRDF in bartender_moveit_config/srdf.'),
        DeclareLaunchArgument(
            'moveit_controllers_file', default_value='moveit_controllers.yaml',
            description='Controller list in bartender_moveit_config/config.'),
        DeclareLaunchArgument(
            'execution_duration_monitoring', default_value='true',
            description='Cancel a move that runs much longer than planned. '
                        'The real UR must have this false: see below.'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    pkg_description = get_package_share_directory('bartender_description')
    pkg_moveit_config = get_package_share_directory('bartender_moveit_config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    description_file = LaunchConfiguration('description_file').perform(context)
    description_args = LaunchConfiguration('description_args').perform(context)
    srdf_file = LaunchConfiguration('srdf_file').perform(context)
    moveit_controllers_file = LaunchConfiguration(
        'moveit_controllers_file').perform(context)

    # Must render the same way bartender_gazebo/launch/sim.launch.py does --
    # through render_bartender_urdf.py, which grooves the gripper pads -- or
    # MoveIt would collision-check against a gripper the physics engine is not
    # simulating.
    xacro_file = os.path.join(pkg_description, 'urdf', description_file)
    render_script = os.path.join(pkg_description, 'scripts',
                                 'render_bartender_urdf.py')
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='python3'), ' ', render_script, ' ',
                     xacro_file, ' ', description_args]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            open(os.path.join(pkg_moveit_config, 'srdf', srdf_file)).read(),
            value_type=str,
        )
    }

    # Passed as a file path (not parsed into a dict): ROS2's "/**:
    # ros__parameters:" wildcard YAML syntax is only resolved when a node
    # parameter is given a file path, not when pre-parsed into a plain dict.
    robot_description_kinematics = os.path.join(pkg_moveit_config, 'config',
                                                'kinematics.yaml')
    robot_description_planning = {
        'robot_description_planning': load_yaml(
            pkg_moveit_config, os.path.join('config', 'joint_limits.yaml'))
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml(pkg_moveit_config, os.path.join('config', 'ompl_planning.yaml'))
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(
            pkg_moveit_config, os.path.join('config', moveit_controllers_file)
        )['moveit_simple_controller_manager'],
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        # The real UR's scaled trajectory controller slows every move down
        # with the pendant's speed slider (and in reduced mode), so a move
        # at 15% takes ~7x its planned time. With monitoring on, MoveIt
        # cancels it after 1.2x, the arm stops dead, the retry is cancelled
        # the same way, and the goto fails with -6 (TIMED_OUT). The
        # controller's own goal tolerances still catch a move that fails.
        # ur_moveit_config turns it off for the same reason.
        'trajectory_execution.execution_duration_monitoring': ParameterValue(
            LaunchConfiguration('execution_duration_monitoring'), value_type=bool),
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', os.path.join(pkg_moveit_config, 'rviz', 'view_robot.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [move_group_node, rviz_node]
