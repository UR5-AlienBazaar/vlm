"""Launch Gazebo Sim with the bar scene and spawn the robot into it.

Loads the bar world (Fortress), spawns the two-armed bartender from
bartender_description's xacro, and bridges /clock plus the topics the
beer-opening sequence reads and writes.

The arguments below let the one-armed workcell reuse it (see
bartender_bringup/launch/workcell_sim.launch.py); the defaults are the bar.

This is Phase 2's world/scene launch. It does NOT start ros2_control
controllers or MoveIt -- see bartender_bringup for the full stack.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description=('Run Gazebo server only, no GUI (avoids the Ignition Qt '
                     'GUI, useful when scripting/tuning against the sim).'),
    )
    headless_rendering = LaunchConfiguration('headless_rendering')
    headless_rendering_arg = DeclareLaunchArgument(
        'headless_rendering', default_value='false',
        description=('With headless:=true, render the camera sensors through '
                     'EGL. Needed on a GPU box with no display, where the '
                     'cameras publish nothing otherwise.'),
    )
    scene_args = [
        DeclareLaunchArgument('world', default_value='bar_world.sdf',
                              description='World file in bartender_gazebo/worlds.'),
        DeclareLaunchArgument('description_file', default_value='bartender.urdf.xacro',
                              description='xacro in bartender_description/urdf.'),
        DeclareLaunchArgument('description_args', default_value='sim_ignition:=true',
                              description='Arguments passed to that xacro.'),
        # Where the description's `world` link goes. For the bar that is arm
        # A's base; for the workcell it is the table's corner.
        DeclareLaunchArgument('spawn_x', default_value='-0.45'),
        DeclareLaunchArgument('spawn_y', default_value='-0.40'),
        DeclareLaunchArgument('spawn_z', default_value='0.9'),
    ]
    pkg_gazebo = get_package_share_directory('bartender_gazebo')
    pkg_description = get_package_share_directory('bartender_description')

    # bartender_robot/models sits one level above the ros2_ws source tree;
    # add it to GZ_SIM_RESOURCE_PATH so `model://jack_daniels_bottle` etc.
    # resolve when Gazebo loads the world.
    models_path = os.path.normpath(
        os.path.join(pkg_gazebo, '..', '..', '..', '..', '..', 'models'))
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = models_path if not existing_resource_path else \
        existing_resource_path + os.pathsep + models_path

    world_path = PathJoinSubstitution(
        [pkg_gazebo, 'worlds', LaunchConfiguration('world')])

    # ROS2 Humble's setup.bash does not add /opt/ros/humble/lib to Gazebo's
    # own plugin search path, so gz_ros2_control-system (referenced by
    # bartender.urdf.xacro's <gazebo><plugin> tag) fails to load with
    # "couldn't find shared library" unless this is set explicitly.
    ros_lib_path = '/opt/ros/humble/lib'
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = ros_lib_path if not existing_plugin_path else \
        existing_plugin_path + os.pathsep + ros_lib_path

    gz_sim_env = {
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
    }
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        additional_env=gz_sim_env,
        output='screen',
        condition=UnlessCondition(headless),
    )
    # Off by default: EGL needs a GPU driver that exposes an EGL device, which
    # a Docker-on-WSL setup without GPU passthrough does not have.
    gz_sim_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_path],
        additional_env=gz_sim_env,
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", headless, "' == 'true' and '", headless_rendering, "' != 'true'"])),
    )
    gz_sim_headless_rendering = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '--headless-rendering', '-r', world_path],
        additional_env=gz_sim_env,
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", headless, "' == 'true' and '", headless_rendering, "' == 'true'"])),
    )

    # Rendered through render_bartender_urdf.py rather than xacro directly:
    # it runs xacro and then cuts the cylinder-holding groove into the gripper
    # pads, which has to replace the stock fingertip collision and so cannot be
    # done in xacro alone. move_group.launch.py renders the same way, so the
    # planner and the physics engine agree on the gripper's shape.
    xacro_file = PathJoinSubstitution(
        [pkg_description, 'urdf', LaunchConfiguration('description_file')])
    render_script = os.path.join(pkg_description, 'scripts', 'render_bartender_urdf.py')
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='python3'), ' ', render_script, ' ',
                 xacro_file, ' ', LaunchConfiguration('description_args')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bartender_ur5e',
            # The defaults are for the bar; the workcell passes the table
            # corner instead. On the bar, this is:
            # Arm A's base, and therefore the origin of base_link, the
            # frame bartender_pour plans everything in. On the bar top
            # (z=0.9), at one end of the bar and 0.40 off its centreline.
            # Arm B stands 1.06 down the bar and 0.80 across, turned to
            # face back at arm A; the description states that relative to
            # this point. See the bar layout block in worlds/bar_world.sdf.
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Two more bridges, both for the beer-opening sequence. Separate node
    # from the clock bridge only because that one is Phase 2 infrastructure
    # and these are not; there is no technical reason they could not share.
    #
    # dynamic_pose/info is SceneBroadcaster's running report of where every
    # NON-STATIC model in the world actually is. It is bridged under its own
    # name rather than onto /tf on purpose: it arrives at 60Hz and carries
    # every bottle, glass and cap in the scene, and dumping that into /tf
    # would bury the robot's own transforms. bartender_open subscribes to it
    # directly, and it is how that node knows whether the beer moved while it
    # was being pushed on -- which is the whole test for "the other arm is
    # really holding it".
    #
    # The detach topic goes the other way, ROS to Gazebo (`]`), and is what
    # actually releases the cap. See make_beer_and_opener.py's docstring for
    # why the release is commanded rather than pried.
    beer_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/bar_world/dynamic_pose/info'
            '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/beer/cap/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
        ],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/bartender/arm_a/wrist_camera/image_raw'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/arm_b/wrist_camera/image_raw'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/stand_camera/depth'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/stand_camera/camera_info'
            '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/bartender/stand_camera/rgb'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/overhead_camera/image_raw'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/overhead_camera/segmentation/labels_map'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/stand_camera/segmentation/labels_map'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/bartender/arm_a/wrist_camera/segmentation/labels_map'
            '@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen',
    )

    return LaunchDescription(scene_args + [
        headless_arg,
        headless_rendering_arg,
        gz_sim,
        gz_sim_headless,
        gz_sim_headless_rendering,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        beer_bridge,
        camera_bridge,
    ])
