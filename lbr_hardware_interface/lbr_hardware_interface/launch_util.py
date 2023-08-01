from typing import Dict

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class LBRHardwareInterfaceLaunch:
    @staticmethod
    def arg_ctrl_cfg_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg_pkg",
            default_value="lbr_hardware_interface",
            description="Controller configuration package. The package containing the ctrl_cfg.",
        )

    @staticmethod
    def arg_ctrl_cfg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg",
            default_value="config/lbr_controllers.yml",
            description="Relative path from ctrl_cfg_pkg to the controllers.",
        )

    @staticmethod
    def arg_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl",
            default_value="position_trajectory_controller",
            description="Desired default controller. One of specified in ctrl_cfg.",
        )

    @staticmethod
    def node_ros2_control(robot_description: Dict[str, str], **kwargs) -> Node:
        return Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"use_sim_time": False},
                PathJoinSubstitution(
                    [
                        FindPackageShare(LaunchConfiguration("ctrl_cfg_pkg")),
                        LaunchConfiguration("ctrl_cfg"),
                    ]
                ),
                robot_description,
            ],
            **kwargs
        )

    @staticmethod
    def node_joint_state_broadcaster(**kwargs) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            **kwargs
        )

    @staticmethod
    def node_controller(**kwargs) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                LaunchConfiguration("ctrl"),
                "--controller-manager",
                "/controller_manager",
            ],
            **kwargs
        )

    @staticmethod
    def node_robot_state_publisher(robot_description: Dict[str, str], **kwargs) -> Node:
        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": False},
            ],
            **kwargs
        )
