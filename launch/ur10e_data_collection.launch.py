"""
UR10e teleoperation + RealSense D435i data collection launch.

Extends ur10e_teleop.launch.py with:
  - realsense2_camera node (RGB + depth + IMU from the D435i)
  - ur10e_data_collector node (episode-aware rosbag2 writer)

Data is saved to ~/teleop_data/ as individual rosbag2 bags, one per clutch-
engage / clutch-release cycle.  Each bag contains:
  /camera/camera/color/image_raw
  /camera/camera/depth/image_rect_raw
  /camera/camera/imu
  /joint_states
  /q2r_right_hand_pose
  /q2r_right_hand_inputs

Run:
    ROS_DOMAIN_ID=69 ros2 launch q2r2_bringup ur10e_data_collection.launch.py \\
        robot_ip:=192.168.1.100

Optional args: same as ur10e_teleop.launch.py plus:
    data_dir   — output directory (default: ~/teleop_data)
    rgb_width  — camera width (default: 640)
    rgb_height — camera height (default: 480)
    fps        — camera frame rate (default: 30)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_env = dict(os.environ)
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'

    robot_ip_arg   = DeclareLaunchArgument('robot_ip',   default_value='192.168.1.100')
    rviz_arg       = DeclareLaunchArgument('rviz',        default_value='true')
    scale_arg      = DeclareLaunchArgument('scale_factor', default_value='1.2')
    data_dir_arg   = DeclareLaunchArgument('data_dir',    default_value=os.path.expanduser('~/teleop_data'))
    rgb_w_arg      = DeclareLaunchArgument('rgb_width',   default_value='640')
    rgb_h_arg      = DeclareLaunchArgument('rgb_height',  default_value='480')
    fps_arg        = DeclareLaunchArgument('fps',         default_value='30')

    # Include the base teleop launch (UR driver + controller switch + teleop node)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('q2r2_bringup'), 'launch', 'ur10e_teleop.launch.py'])
        ]),
        launch_arguments={
            'robot_ip':    LaunchConfiguration('robot_ip'),
            'rviz':        LaunchConfiguration('rviz'),
            'scale_factor': LaunchConfiguration('scale_factor'),
        }.items(),
    )

    # RealSense D435i — RGB + depth streams + IMU
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'rgb_camera.color_profile':  LaunchConfiguration('rgb_width'),
            'depth_module.depth_profile': LaunchConfiguration('rgb_width'),
            'enable_gyro':   True,
            'enable_accel':  True,
            'unite_imu_method': 1,         # linear interpolation
            'align_depth.enable': True,    # depth aligned to RGB
            'colorizer.enable': False,
            'pointcloud.enable': False,    # save bandwidth; enable if needed
        }],
    )

    # Episode-aware data collector
    data_collector = Node(
        package='q2r2_bringup',
        executable='ur10e_data_collector',
        name='ur10e_data_collector',
        output='screen',
        parameters=[{'data_dir': LaunchConfiguration('data_dir')}],
    )

    return LaunchDescription([
        robot_ip_arg, rviz_arg, scale_arg,
        data_dir_arg, rgb_w_arg, rgb_h_arg, fps_arg,
        teleop_launch,
        realsense,
        data_collector,
    ])
