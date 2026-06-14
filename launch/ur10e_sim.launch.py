"""
UR10e simulation launch — no real hardware required.

Starts robot_state_publisher (with the ur10e URDF) + the teleop controller
in sim_mode so joint states are published directly to /joint_states and
visualized in RViz2. Also starts the ROS-TCP endpoint for the Quest 2 headset.

Run:
    ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_sim.launch.py
    ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_sim.launch.py rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_env = dict(os.environ)
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'

    xacro_executable = os.path.expanduser('~/.local/bin/xacro')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro'])

    robot_description_content = Command([
        xacro_executable, ' ', xacro_file,
        ' ur_type:=ur10e name:=ur'
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Launch RViz2')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_endpoint',
        output='screen',
        parameters=[{'ROS_IP': '0.0.0.0'}, {'ROS_TCP_PORT': 10000}],
    )

    teleop_controller = Node(
        package='q2r2_bringup',
        executable='ur10e_teleop_controller',
        name='ur10e_teleop_controller',
        output='screen',
        parameters=[{
            'sim_mode':        True,
            'scale_factor':    1.2,
            'publish_rate':    30.0,
            'max_joint_delta': 0.15,
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

    return LaunchDescription([rviz_arg, robot_state_publisher, tcp_endpoint, teleop_controller, rviz])
