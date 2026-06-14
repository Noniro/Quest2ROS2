import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _find_support_package():
    # Team standalone package first, upstream Yaskawa support as fallback.
    for pkg in ("hc10dtp_b00_support", "motoman_hc10_support"):
        try:
            get_package_share_directory(pkg)
            return pkg
        except Exception:
            continue
    raise RuntimeError(
        "Neither 'hc10dtp_b00_support' nor 'motoman_hc10_support' is installed")


def generate_launch_description():
    # Prepare environment for RViz (must carry over DISPLAY and other variables)
    rviz_env = dict(os.environ)
    rviz_env["QT_QPA_PLATFORM"] = "xcb"

    # Find xacro executable
    xacro_executable = os.path.expanduser('~/.local/bin/xacro')

    # Path to hc10dtp xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare(_find_support_package()),
        "urdf",
        "hc10dtp_b00.xacro"
    ])

    # Generate robot description XML via xacro command
    robot_description_content = Command([
        xacro_executable,
        " ",
        xacro_file
    ])

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    # Node: hc10dtp_teleop_controller running in simulation mode
    hc10_sim_controller_node = Node(
        package="q2r2_bringup",
        executable="hc10dtp_teleop_controller",
        name="hc10dtp_teleop_controller",
        output="screen",
        parameters=[{
            'sim_mode': True,
            'scale_factor': 1.2,
            'publish_rate': 30.0,
            'max_joint_delta': 0.15
        }]
    )

    # Node: RViz2 (preset config is optional — motoman_resources is only
    # present in the LearnROS2 workspace, not the team workspace)
    try:
        rviz_args = ["-d", os.path.join(
            get_package_share_directory("motoman_resources"), "rviz", "view_robot.rviz")]
    except Exception:
        rviz_args = []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration('rviz')),
        env=rviz_env
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        hc10_sim_controller_node,
        rviz_node
    ])
