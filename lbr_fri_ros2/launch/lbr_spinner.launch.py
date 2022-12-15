from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    safety_package_arg = DeclareLaunchArgument(
        name="safety_package",
        description="The safety file is located relative to this package.",
        default_value="lbr_fri_ros2"
    )
    safety_file_arg = DeclareLaunchArgument(
        name="safety_file",
        description="YAML file containing safety configurations for the LBR.",
        default_value="config/lbr_safety_limits.yml"
    )

    safety_file = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("safety_package")),
        LaunchConfiguration("safety_file")
    ])
    
    lbr_spinner_node = Node(
        package="lbr_fri_ros2",
        executable="lbr_spinner",
        parameters=[safety_file],
        emulate_tty=True,
        output="screen"
    )

    return LaunchDescription([
        safety_package_arg,
        safety_file_arg,
        lbr_spinner_node
    ])
