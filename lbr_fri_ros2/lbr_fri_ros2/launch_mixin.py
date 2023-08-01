from typing import Dict

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


class LBRFRIROS2Mixin:
    @staticmethod
    def node_lbr_app(robot_description: Dict[str, str]) -> DeclareLaunchArgument:
        return Node(
            package="lbr_fri_ros2",
            executable="lbr_app",
            emulate_tty=True,
            output="screen",
            parameters=[robot_description],
        )
