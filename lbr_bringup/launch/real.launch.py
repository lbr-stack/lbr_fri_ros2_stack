from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from lbr_description import LBRDescriptionMixin, RVizMixin
from lbr_hardware_interface import LBRHardwareInterfaceMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.description(sim=False)

    # ros2 control node
    ld.add_action(LBRHardwareInterfaceMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRHardwareInterfaceMixin.arg_ctrl_cfg())
    ld.add_action(LBRHardwareInterfaceMixin.arg_ctrl())
    ros2_control_node = LBRHardwareInterfaceMixin.node_ros2_control(
        robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # joint state broad caster and controller on ros2 control node start
    joint_state_broadcaster = LBRHardwareInterfaceMixin.node_joint_state_broadcaster()
    controller = LBRHardwareInterfaceMixin.node_controller()

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster, controller],
        )
    )
    ld.add_action(controller_event_handler)

    # robot state publisher on joint state broadcaster spawn exit
    robot_state_publisher = LBRHardwareInterfaceMixin.node_robot_state_publisher(
        robot_description=robot_description
    )
    robot_state_publisher_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[robot_state_publisher]
        )
    )
    ld.add_action(robot_state_publisher_event_handler)

    # RViz
    ld.add_action(RVizMixin.arg_rviz_config_pkg())
    ld.add_action(RVizMixin.arg_rviz_config())
    rviz = RVizMixin.node_rviz()
    rviz_event_handler = RegisterEventHandler(
        OnProcessStart(target_action=robot_state_publisher, on_start=[rviz])
    )
    ld.add_action(rviz_event_handler)

    return ld
