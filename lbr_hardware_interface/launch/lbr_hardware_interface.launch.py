from launch import LaunchDescription

from lbr_description import LBRDescriptionMixin
from lbr_hardware_interface import LBRHardwareInterfaceMixin


class LBRHardwareInterface(LBRDescriptionMixin, LBRHardwareInterfaceMixin):
    pass


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRHardwareInterface.arg_model())
    ld.add_action(LBRHardwareInterface.arg_robot_name())
    robot_description = LBRHardwareInterface.description(sim=False)
    ld.add_action(LBRHardwareInterface.arg_ctrl_cfg_pkg())
    ld.add_action(LBRHardwareInterface.arg_ctrl_cfg())
    ld.add_action(LBRHardwareInterface.arg_ctrl())
    ld.add_action(
        LBRHardwareInterface.node_ros2_control(robot_description=robot_description)
    )
    ld.add_action(LBRHardwareInterface.node_joint_state_broadcaster())
    ld.add_action(
        LBRHardwareInterface.node_robot_state_publisher(
            robot_description=robot_description
        )
    )
    ld.add_action(LBRHardwareInterface.node_controller())
    return ld
