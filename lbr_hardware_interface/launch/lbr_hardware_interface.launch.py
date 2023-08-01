from launch import LaunchDescription

from lbr_description import LBRDescriptionLaunch
from lbr_hardware_interface import LBRHardwareInterfaceLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionLaunch.model_arg())
    ld.add_action(LBRDescriptionLaunch.robot_name_arg())
    robot_description = LBRDescriptionLaunch.description(sim=False)
    hw_args = LBRHardwareInterfaceLaunch.args_dict()
    hw_nodes = LBRHardwareInterfaceLaunch.nodes_dict(robot_description)
    {ld.add_action(hw_args[key]) for key in hw_args}
    {ld.add_action(hw_nodes[key]) for key in hw_nodes}
    return ld
