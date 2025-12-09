from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="keyboard_config_pkg",
                default_value="lbr_moveit",
                description="The package containing the keyboard configurations.",
            ),
            DeclareLaunchArgument(
                name="keyboard_config",
                default_value="config/forward_keyboard.yaml",
                description="Location of the keyboard configuration file relative to keyboard_config_pkg.",
            ),
            LBRDescriptionMixin.arg_robot_name(),
            Node(
                package="lbr_moveit",
                executable="forward_keyboard",
                output="screen",
                parameters=[
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("keyboard_config_pkg"))
                    )
                    / LaunchConfiguration("keyboard_config"),
                ],
                namespace=LaunchConfiguration("robot_name"),
            ),
        ]
    )
