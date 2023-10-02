from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from lbr_description import LBRDescriptionMixin
from lbr_ros2_control import LBRSystemInterfaceMixin


class LBRSystemInterface(LBRDescriptionMixin, LBRSystemInterfaceMixin):
    pass


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRSystemInterface.arg_base_frame())
    ld.add_action(LBRSystemInterface.arg_model())
    ld.add_action(LBRSystemInterface.arg_robot_name())
    robot_description = LBRSystemInterface.param_robot_description(sim=False)
    ld.add_action(LBRSystemInterface.arg_ctrl_cfg_pkg())
    ld.add_action(LBRSystemInterface.arg_ctrl_cfg())
    ld.add_action(LBRSystemInterface.arg_ctrl())
    ld.add_action(LBRSystemInterface.arg_frame_prefix())
    ros2_control_node = LBRSystemInterface.node_ros2_control(
        robot_description=robot_description
    )
    ld.add_action(ros2_control_node)
    joint_state_broadcaster = LBRSystemInterface.node_joint_state_broadcaster()
    controller = LBRSystemInterface.node_controller()
    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster, controller],
        )
    )
    ld.add_action(controller_event_handler)
    robot_state_publisher = LBRSystemInterface.node_robot_state_publisher(
        robot_description=robot_description,
        use_sim_time=False,
        frame_prefix=LBRSystemInterface.param_frame_prefix(),
    )
    ld.add_action(robot_state_publisher)
    return ld
