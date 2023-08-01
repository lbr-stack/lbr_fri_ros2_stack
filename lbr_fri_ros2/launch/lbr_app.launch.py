from launch import LaunchDescription
from launch_ros.actions import Node

from lbr_description import LBRDescriptionLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld = LBRDescriptionLaunch.add_model_arg(ld)
    ld = LBRDescriptionLaunch.add_robot_name_arg(ld)
    robot_description = LBRDescriptionLaunch.description(sim=False)

    ld.add_action(
        Node(
            package="lbr_fri_ros2",
            executable="lbr_app",
            emulate_tty=True,
            output="screen",
            parameters=[robot_description],
        )
    )

    return ld
