"""
UR10e proximity-clutch teleop — simulation launch.

No real hardware needed.  Robot model is visualized in RViz.

IMPORTANT: start the Quest2 TCP endpoint SEPARATELY after this launch is up,
to avoid a race condition where Quest2 reconnects before the teleop node
has registered its topics (which crashes the endpoint):

    ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_proximity_sim.launch.py
    # wait ~5 s for "IK chain" + "Ready" in log, then in a new terminal:
    source ~/projects/LearnROS2/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=69 ros2 run ros_tcp_endpoint default_server_endpoint \\
        --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000

Tune the snap zone if the hand sphere appears in the wrong place:
    ros2 param set /ur10e_proximity_teleop quest_offset_z 0.1
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_env = dict(os.environ)
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                os.path.expanduser('~/.local/bin/xacro'), ' ', xacro_file,
                ' ur_type:=ur10e name:=ur'
            ]),
            value_type=str
        )
    }

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz2')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # NOTE: tcp_endpoint is intentionally NOT in this launch.
    # Start it separately after this launch prints "Ready" to avoid the
    # race condition where Quest2 sends messages before topics are registered.

    teleop = Node(
        package='q2r2_bringup',
        executable='ur10e_proximity_teleop',
        name='ur10e_proximity_teleop',
        output='screen',
        parameters=[{
            'sim_mode':             True,
            'scale_factor':         1.0,
            'publish_rate':         30.0,
            'max_joint_delta':      0.10,   # ~6° per step — roomy enough for sim
            'engage_threshold':     0.08,   # 8 cm snap zone
            'proximate_frames':     10,     # 0.33 s debounce
            'disengage_fail_count': 15,     # tolerate more IK misses during testing
            # Initial hand sphere starts 35 cm below TCP — lift to engage:
            'initial_hand_offset_x': 0.0,
            'initial_hand_offset_y': 0.0,
            'initial_hand_offset_z': -0.35,
            # Fine-tune hand sphere position in robot frame if needed:
            'quest_offset_x':       0.0,
            'quest_offset_y':       0.0,
            'quest_offset_z':       0.0,
        }],
    )

    try:
        rviz_cfg = ['-d', os.path.join(
            get_package_share_directory('ur_description'), 'rviz', 'view_robot.rviz')]
    except Exception:
        rviz_cfg = []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_cfg,
        condition=IfCondition(LaunchConfiguration('rviz')),
        env=rviz_env,
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        teleop,
        rviz,
    ])
