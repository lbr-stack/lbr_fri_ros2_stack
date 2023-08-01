from typing import Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class LBRHardwareInterfaceLaunch:
    @staticmethod
    def add_ctrl_args(ld: LaunchDescription) -> LaunchDescription:
        ld.add_action(
            DeclareLaunchArgument(
                name="ctrl_cfg_pkg",
                default_value="lbr_hardware_interface",
                description="Controller configuration package. The package containing the ctrl_cfg.",
            )
        )

        ld.add_action(
            DeclareLaunchArgument(
                name="ctrl_cfg",
                default_value="config/lbr_controllers.yml",
                description="Relative path from ctrl_cfg_pkg to the controllers.",
            )
        )

        ld.add_action(
            DeclareLaunchArgument(
                name="ctrl",
                default_value="position_trajectory_controller",
                description="Desired default controller. One of specified in ctrl_cfg.",
            )
        )
        return ld

    @staticmethod
    def add_nodes(
        ld: LaunchDescription, robot_description: Dict[str, str]
    ) -> LaunchDescription:
        # controller manager
        ld.add_action(
            Node(
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
            )
        )

        # joint state broadcaster
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        )

        # controllers
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    LaunchConfiguration("ctrl"),
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        )

        # robot state publisher
        ld.add_action(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    robot_description,
                    {"use_sim_time": False},
                ],
            )
        )

        return ld
