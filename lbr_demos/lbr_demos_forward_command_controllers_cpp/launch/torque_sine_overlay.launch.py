from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescriptionMixin
from lbr_fri_ros2 import LBRFRIROS2Mixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    ld.add_action(
        LBRFRIROS2Mixin.node_app(
            parameters=[
                robot_description,
            ]
        )
    )
    ld.add_action(
        Node(
            package="lbr_demos_fri_ros2_cpp",
            executable="torque_sine_overlay_node",
            output="screen",
        )
    )
    return ld
