"""The real workcell robot, with a Gazebo twin that copies it live.

    ros2 launch bartender_bringup workcell_twin.launch.py
    ros2 launch bartender_bringup workcell_twin.launch.py use_fake_hardware:=true

Everything workcell_real.launch.py starts (the UR driver, the controllers,
MoveIt), unchanged and under the same names, so bartender_teach and every
other client drive the real arm exactly as they drive it without a twin.
Next to it, the workcell simulation, in which the arm does not plan or take
goals: twin_mirror copies the real arm's joint angles onto it. The twin
shows what the robot is doing, including moves made on the UR pendant.

The real robot leads and the twin follows, never the other way round.
Nothing the twin does can move the robot.

With use_fake_hardware:=true the "real" robot is ros2_control's mock
hardware, and the twin mirrors that, which is how to see the whole thing
work with no robot on the cable.

THE TWIN IS NAMESPACED. Its controller_manager, joint states,
robot_description and TF are all under /twin, because the real robot owns
the plain names. Nothing outside this file and twin_mirror should need to
know the twin is there.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

TWIN_NS = 'twin'


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bartender_bringup')
    pkg_gazebo = get_package_share_directory('bartender_gazebo')

    table_height = LaunchConfiguration('table_height')
    arguments = [
        # Only the arguments this file itself reads. Every other
        # workcell_real.launch.py argument (robot_ip, use_fake_hardware,
        # the gripper and tool settings) can be given here too and goes
        # straight through to it.
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the twin\'s Gazebo without a GUI.'),
        DeclareLaunchArgument('table_height', default_value='0.75'),
        DeclareLaunchArgument('arm_x', default_value='0.35'),
        DeclareLaunchArgument('arm_y', default_value='0.35'),
        DeclareLaunchArgument('arm_yaw', default_value='-1.5708'),
    ]

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'workcell_real.launch.py')))

    # The same table and arm pose as the real robot's description, so the
    # twin stands where the robot stands.
    description_args = [
        'sim_ignition:=true',
        f' ros_namespace:=/{TWIN_NS}',
        ' sim_controllers:=workcell_twin_controllers.yaml',
        ' table_height:=', table_height,
        ' arm_x:=', LaunchConfiguration('arm_x'),
        ' arm_y:=', LaunchConfiguration('arm_y'),
        ' arm_yaw:=', LaunchConfiguration('arm_yaw'),
    ]

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[controller,
                       '--controller-manager', f'/{TWIN_NS}/controller_manager',
                       '--controller-manager-timeout', '60'])

    twin = GroupAction([
        PushRosNamespace(TWIN_NS),
        # tf2 broadcasters publish to the ABSOLUTE /tf, which a namespace
        # does not move. Without these the twin's robot_state_publisher and
        # the real one would both publish base_link -> tool0, and every
        # consumer of TF would see the arm flicker between two poses.
        SetRemap(src='/tf', dst=f'/{TWIN_NS}/tf'),
        SetRemap(src='/tf_static', dst=f'/{TWIN_NS}/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'world': 'workcell_world.sdf',
                'description_file': 'workcell.urdf.xacro',
                'description_args': description_args,
                'spawn_x': '0.0',
                'spawn_y': '0.0',
                'spawn_z': table_height,
            }.items(),
        ),
        # Same head start as workcell_sim.launch.py gives Gazebo.
        TimerAction(period=12.0, actions=[
            spawner('joint_state_broadcaster'),
            spawner('twin_position_controller'),
        ]),
    ])

    # Outside the namespace: it reads the real /joint_states and writes the
    # twin's controller by its full name.
    mirror = TimerAction(period=14.0, actions=[
        Node(package='bartender_bringup', executable='twin_mirror',
             output='screen',
             parameters=[{
                 'source_topic': '/joint_states',
                 'target_topic':
                     f'/{TWIN_NS}/twin_position_controller/commands',
             }]),
    ])

    return LaunchDescription(arguments + [real, twin, mirror])
