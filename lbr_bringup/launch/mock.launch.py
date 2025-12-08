from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="mock")

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # ros2 control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(use_sim_time=False)
    ld.add_action(ros2_control_node)

    # spawn controllers
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controllers=["joint_state_broadcaster", LaunchConfiguration("ctrl")]
        )
    )
    return ld
