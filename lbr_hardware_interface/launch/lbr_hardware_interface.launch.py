from launch import LaunchDescription

from lbr_description import LBRDescriptionLaunch
from lbr_hardware_interface import LBRHardwareInterfaceLaunch


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionLaunch.arg_model())
    ld.add_action(LBRDescriptionLaunch.arg_robot_name())
    robot_description = LBRDescriptionLaunch.description(sim=False)
    ld.add_action(LBRHardwareInterfaceLaunch.arg_ctrl_cfg_pkg())
    ld.add_action(LBRHardwareInterfaceLaunch.arg_ctrl_cfg())
    ld.add_action(LBRHardwareInterfaceLaunch.arg_ctrl())
    ld.add_action(
        LBRHardwareInterfaceLaunch.node_ros2_control(
            robot_description=robot_description
        )
    )
    ld.add_action(LBRHardwareInterfaceLaunch.node_joint_state_broadcaster())
    ld.add_action(
        LBRHardwareInterfaceLaunch.node_robot_state_publisher(
            robot_description=robot_description
        )
    )
    ld.add_action(LBRHardwareInterfaceLaunch.node_controller())
    return ld
