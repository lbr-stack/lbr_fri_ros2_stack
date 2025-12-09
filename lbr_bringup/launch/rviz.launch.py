from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rviz_cfg",
                default_value="config/mock.rviz",
                description="The RViz configuration file relative to rviz_cfg_pkg.",
            ),
            DeclareLaunchArgument(
                name="rviz_cfg_pkg",
                default_value="lbr_bringup",
                description="The package containing the RViz configuration file.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("rviz_cfg_pkg"))
                    )
                    / LaunchConfiguration("rviz_cfg"),
                ],
            ),
        ]
    )
