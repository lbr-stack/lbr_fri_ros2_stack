from launch import LaunchDescription

from lbr_description import LBRDescriptionLaunch
from lbr_hardware_interface import LBRHardwareInterfaceLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld = LBRDescriptionLaunch.add_model_arg(ld)
    ld = LBRDescriptionLaunch.add_robot_name_arg(ld)
    robot_description = LBRDescriptionLaunch.description(sim=False)
    ld = LBRHardwareInterfaceLaunch.add_ctrl_args(ld)
    ld = LBRHardwareInterfaceLaunch.add_nodes(ld, robot_description)
    return ld
