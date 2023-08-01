import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescriptionLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionLaunch.arg_model())
    ld.add_action(LBRDescriptionLaunch.arg_robot_name())
    robot_description = LBRDescriptionLaunch.description(sim=True)
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
            parameters=[robot_description],
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
