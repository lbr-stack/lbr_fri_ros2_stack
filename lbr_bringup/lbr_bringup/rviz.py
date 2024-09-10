from typing import Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class RVizMixin:
    @staticmethod
    def arg_rviz(default_value: str = "false") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz",
            default_value=default_value,
            description="Whether to launch RViz.",
        )

    @staticmethod
    def arg_rviz_config_pkg(
        default_value: str = "lbr_bringup",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config(
        default_value: str = "config/mock.rviz",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_pkg: Optional[
            Union[LaunchConfiguration, str]
        ] = LaunchConfiguration("rviz_config_pkg", default="lbr_bringup"),
        rviz_config: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "rviz_config", default="config/mock.rviz"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(rviz_config_pkg),
                        rviz_config,
                    ]
                ),
            ],
            **kwargs,
        )
