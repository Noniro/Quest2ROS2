"""
UR10e real-hardware teleoperation launch.

What this does:
  1. Launches ur_robot_driver with the calibrated robot description.
  2. After 5 s, switches ros2_control to forward_position_controller so the
     teleop controller can stream positions at 30 Hz.
  3. Starts the UR10e teleop controller in HW mode.
  4. Starts the ROS-TCP endpoint for the Quest 2 headset.
  5. Optionally starts RViz2.

Prerequisites:
  - External Control URCap is installed on the pendant and a program with
    the External Control block is loaded (but NOT yet running — the driver
    will start it automatically once connected).
  - The robot is reachable: ping <ROBOT_IP>
  - Calibration has been extracted:
      ros2 run ur_calibration calibration_correction \\
        --ros-args -p robot_ip:=<ROBOT_IP> \\
        -p target_filename:=<path>/calibration.yaml

Run:
    ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_teleop.launch.py \\
        robot_ip:=192.168.1.100

Optional args:
    robot_ip        — IP of the UR10e controller box (default: 192.168.1.100)
    kinematics_yaml — path to calibration.yaml (default: empty = nominal)
    rviz            — true/false (default: true)
    scale_factor    — VR motion amplification (default: 1.2)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_env = dict(os.environ)
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'

    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.100',
                                          description='IP address of the UR10e controller')
    kinematics_arg = DeclareLaunchArgument('kinematics_yaml', default_value='',
                                            description='Path to calibration.yaml (leave empty for nominal kinematics)')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    scale_arg = DeclareLaunchArgument('scale_factor', default_value='1.2')
    headless_arg = DeclareLaunchArgument(
        'headless_mode', default_value='true',
        description='Send URScript directly (true) instead of waiting for External Control URCap (false)')

    # UR driver — brings up ros2_control + joint_state_broadcaster.
    # headless_mode=true sends the URScript directly via port 30001, so no
    # pendant Play button is needed and the URCap timing race is avoided.
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ]),
        launch_arguments={
            'ur_type':            'ur10e',
            'robot_ip':           LaunchConfiguration('robot_ip'),
            'launch_rviz':        'false',
            'headless_mode':      LaunchConfiguration('headless_mode'),
            'initial_joint_controller': 'forward_position_controller',
            'reverse_ip':         '192.168.1.202',
        }.items(),
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
            'sim_mode':        False,
            'scale_factor':    LaunchConfiguration('scale_factor'),
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
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=rviz_cfg,
        condition=IfCondition(LaunchConfiguration('rviz')),
        env=rviz_env,
    )

    return LaunchDescription([
        robot_ip_arg, kinematics_arg, rviz_arg, scale_arg, headless_arg,
        ur_control,
        tcp_endpoint,
        teleop_controller,
        rviz,
    ])
