import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from lbr_description import LBRDescriptionMixin
from lbr_fri_ros2 import LBRFRIROS2Mixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRFRIROS2Mixin.arg_port_id())
    ld.add_action(LBRDescriptionMixin.arg_model())
    robot_description = LBRDescriptionMixin.param_robot_description(sim=False)
    config = os.path.join(
        get_package_share_directory("lbr_demos_fri_ros2_advanced_cpp"),
        "config",
        "admittance_control_node.yaml",
    )
    ld.add_action(
        ComposableNodeContainer(
            name="admittance_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="lbr_fri_ros2",
                    plugin="lbr_fri_ros2::AppComponent",
                    name="app",
                    namespace="lbr",
                    parameters=[robot_description, LBRFRIROS2Mixin.param_port_id()],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="lbr_demos_fri_ros2_advanced_cpp",
                    plugin="lbr_fri_ros2::AdmittanceControlNode",
                    name="admittance_control_node",
                    parameters=[robot_description, config],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        )
    )
    return ld
