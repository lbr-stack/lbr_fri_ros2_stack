from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescriptionLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionLaunch.model_arg())
    ld.add_action(LBRDescriptionLaunch.robot_name_arg())
    robot_description = LBRDescriptionLaunch.descrption(sim=False)
    ld.add_action(
        Node(
            package="lbr_demos_fri_ros2_advanced_python",
            executable="admittance_control_node",
            output="screen",
            parameters=[robot_description],
        )
    )
    return ld
