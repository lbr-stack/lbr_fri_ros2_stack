from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LBRDescriptionMixin.arg_model(),
            LBRDescriptionMixin.arg_robot_name(),
            LBRROS2ControlMixin.arg_ctrl_cfg_pkg(),
            LBRROS2ControlMixin.arg_ctrl_cfg(),
            LBRROS2ControlMixin.arg_ctrl(),
            LBRROS2ControlMixin.arg_init_jnt_pos(),
            LBRROS2ControlMixin.node_robot_state_publisher(
                robot_description=LBRDescriptionMixin.param_robot_description(
                    mode="mock"
                ),
                use_sim_time=False,
            ),
            LBRROS2ControlMixin.node_ros2_control(use_sim_time=False),
            LBRROS2ControlMixin.node_controller_spawner(
                controllers=["joint_state_broadcaster", LaunchConfiguration("ctrl")]
            ),
        ]
    )
