from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="hardware")

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # ros2 control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(use_sim_time=False)
    ld.add_action(ros2_control_node)

    # controllers on ros2 control node start
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    estimated_wrench_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="estimated_wrench_broadcaster"
    )
    lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_state_broadcaster"
    )

    preceding_controllers_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                estimated_wrench_broadcaster,
                lbr_state_broadcaster,
            ],
        )
    )
    ld.add_action(preceding_controllers_event_handler)

    # controllers on estimated wrench broadcaster
    force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="force_torque_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    controller_event_handler = RegisterEventHandler(
        OnExecutionComplete(
            target_action=estimated_wrench_broadcaster,  # estimated wrench broadcaster is chained and exposes external wrench estimate state interfaces
            on_completion=[
                force_torque_broadcaster,
                controller,
            ],
        )
    )
    ld.add_action(controller_event_handler)
    return ld
