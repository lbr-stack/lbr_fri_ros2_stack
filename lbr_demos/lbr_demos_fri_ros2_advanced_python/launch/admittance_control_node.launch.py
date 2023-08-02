from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    ld.add_action(
        Node(
            package="lbr_demos_fri_ros2_advanced_python",
            executable="admittance_control_node",
            output="screen",
            parameters=[robot_description],
        )
    )
    return ld
