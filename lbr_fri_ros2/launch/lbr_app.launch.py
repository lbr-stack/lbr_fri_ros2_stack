from launch import LaunchDescription

from lbr_fri_ros2 import LBRFRIROS2Mixin
from lbr_description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    ld.add_action(
        LBRFRIROS2Mixin.node_lbr_app(
            parameters=[
                robot_description,
                LBRDescriptionMixin.param_robot_name(),
            ]
        )
    )
    return ld
