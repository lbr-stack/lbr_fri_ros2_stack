from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from lbr_description import LBRDescription


class LBRHardwareInterface(LaunchDescription):
    def __init__(
        self,
    ) -> None:
        super().__init__()

        self._lbr_descrpition = LBRDescription(sim=False)

        for entity in self._lbr_descrpition.entities:
            self.add_entity(entity)

        self._add_launch_arguments()
        self._add_nodes()

    def _add_launch_arguments(self) -> None:
        self.add_action(
            DeclareLaunchArgument(
                name="ctrl_cfg_pkg",
                default_value="lbr_hardware_interface",
                description="Controller configuration package. The package containing the ctrl_cfg.",
            )
        )

        self.add_action(
            DeclareLaunchArgument(
                name="ctrl_cfg",
                default_value="config/lbr_controllers.yml",
                description="Relative path from ctrl_cfg_pkg to the controllers.",
            )
        )

        self.add_action(
            DeclareLaunchArgument(
                name="ctrl",
                default_value="position_trajectory_controller",
                description="Desired default controller. One of specified in ctrl_cfg.",
            )
        )

    def _add_nodes(self) -> None:
        # controller manager
        self.add_action(
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
                    self._lbr_descrpition.robot_description,
                ],
            )
        )

        # joint state broadcaster
        self.add_action(
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
        self.add_action(
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
        self.add_action(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    self._lbr_descrpition.robot_description,
                    {"use_sim_time": False},
                ],
            )
        )
