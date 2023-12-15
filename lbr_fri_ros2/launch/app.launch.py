from launch import LaunchDescription

from lbr_description import LBRDescriptionMixin
from lbr_fri_ros2 import LBRFRIROS2Mixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    ld.add_action(LBRFRIROS2Mixin.arg_open_loop())
    ld.add_action(LBRFRIROS2Mixin.arg_rt_prio())
    ld.add_action(LBRFRIROS2Mixin.arg_pid_p())
    ld.add_action(LBRFRIROS2Mixin.arg_port_id())
    ld.add_action(
        LBRFRIROS2Mixin.node_app(
            parameters=[
                robot_description,
                LBRFRIROS2Mixin.param_open_loop(),
                LBRFRIROS2Mixin.param_rt_prio(),
                LBRFRIROS2Mixin.param_pid_p(),
                LBRFRIROS2Mixin.param_port_id(),
            ]
        )
    )
    return ld
