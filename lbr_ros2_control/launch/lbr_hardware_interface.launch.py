from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from lbr_description import LBRDescriptionMixin
from lbr_hardware_interface import LBRHardwareInterfaceMixin


class LBRHardwareInterface(LBRDescriptionMixin, LBRHardwareInterfaceMixin):
    pass


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRHardwareInterface.arg_base_frame())
    ld.add_action(LBRHardwareInterface.arg_model())
    ld.add_action(LBRHardwareInterface.arg_robot_name())
    robot_description = LBRHardwareInterface.param_robot_description(sim=False)
    ld.add_action(LBRHardwareInterface.arg_ctrl_cfg_pkg())
    ld.add_action(LBRHardwareInterface.arg_ctrl_cfg())
    ld.add_action(LBRHardwareInterface.arg_ctrl())
    ros2_control_node = LBRHardwareInterface.node_ros2_control(
        robot_description=robot_description
    )
    ld.add_action(ros2_control_node)
    joint_state_broadcaster = LBRHardwareInterface.node_joint_state_broadcaster()
    controller = LBRHardwareInterface.node_controller()
    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster, controller],
        )
    )
    ld.add_action(controller_event_handler)
    robot_state_publisher = LBRHardwareInterface.node_robot_state_publisher(
        robot_description=robot_description
    )
    ld.add_action(robot_state_publisher)
    return ld
