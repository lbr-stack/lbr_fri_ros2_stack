import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from lbr_description import LBRDescription


def generate_launch_description():
    ld = LBRDescription()

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[ld.robot_description],
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("lbr_description"), "config/config.rviz"
                ),
            ],
        )
    )

    return ld
