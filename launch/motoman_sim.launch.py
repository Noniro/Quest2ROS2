import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Prepare environment for RViz (must carry over DISPLAY and other variables)
    rviz_env = dict(os.environ)
    rviz_env["QT_QPA_PLATFORM"] = "xcb"

    # Find xacro executable (use user pip directory path to guarantee accessibility)
    xacro_executable = os.path.expanduser('~/.local/bin/xacro')

    # Path to sda10f xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("motoman_sda10f_support"),
        "urdf",
        "sda10f.xacro"
    ])

    # Generate robot description XML via xacro command
    robot_description_content = Command([
        xacro_executable,
        " ",
        xacro_file
    ])

    robot_description = {"robot_description": robot_description_content}

    # Declare launch argument for RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # Node: motoman_sim_controller wrapper
    motoman_sim_controller_node = Node(
        package="q2r2_bringup",
        executable="motoman_sim_controller",
        output="screen"
    )

    # Node: RViz2
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("motoman_resources"),
        "rviz",
        "view_robot.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        env=rviz_env
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        motoman_sim_controller_node,
        rviz_node
    ])
