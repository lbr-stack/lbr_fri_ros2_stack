from typing import Dict

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class LBRFRIROS2Mixin:
    @staticmethod
    def arg_port_id() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="port_id",
            description="Port ID to open UDP connection at. Valid values are [30200, 30209]",
            default_value="30200",
        )

    @staticmethod
    def param_port_id() -> Dict[str, str]:
        return {"port_id": LaunchConfiguration("port_id")}

    @staticmethod
    def node_lbr_app(**kwargs) -> DeclareLaunchArgument:
        return Node(
            package="lbr_fri_ros2",
            executable="lbr_app",
            emulate_tty=True,
            output="screen",
            **kwargs,
        )
