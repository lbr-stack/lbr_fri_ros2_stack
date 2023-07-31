from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescription


def generate_launch_description() -> LaunchDescription:
    ld = LBRDescription(sim=False)

    ld.add_action(
        Node(
            package="lbr_fri_ros2",
            executable="lbr_app",
            emulate_tty=True,
            output="screen",
            parameters=[ld.robot_description],
        )
    )

    return ld
