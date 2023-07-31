from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescription


def generate_launch_description() -> LaunchDescription:
    ld = LBRDescription(sim=False)

    ld.add_action(
        Node(
            package="lbr_demos_fri_ros2_advanced_cpp",
            executable="admittance_control_node",
            output="screen",
            parameters=[ld.robot_description],
        )
    )

    return ld
