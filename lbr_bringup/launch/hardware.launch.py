from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value="iiwa7",
                description="The LBR model in use.",
                choices=["iiwa7", "iiwa14", "med7", "med14"],
            ),
            DeclareLaunchArgument(
                name="robot_name",
                default_value="lbr",
                description="The robot's name. Links in the tf tree will be prefixed as <robot_name>_link. Same applies to joints.",
            ),
            DeclareLaunchArgument(
                name="namespace",
                default_value="lbr",
                description="Nodes in this launch file will be spawned with this namespace.",
            ),
            DeclareLaunchArgument(
                name="sys_cfg_pkg",
                default_value="lbr_description",
                description="Package containing the lbr_system_config.yaml file for FRI configurations.",
            ),
            DeclareLaunchArgument(
                name="sys_cfg",
                default_value="ros2_control/lbr_system_config.yaml",
                description="The relative path from sys_cfg_pkg to the lbr_system_config.yaml file.",
            ),
            DeclareLaunchArgument(
                name="ctrl_cfg_pkg",
                default_value="lbr_description",
                description="Controller configuration package. The package containing the ctrl_cfg.",
            ),
            DeclareLaunchArgument(
                name="ctrl_cfg",
                default_value="ros2_control/hardware_controllers.yaml",
                description="Relative path from ctrl_cfg_pkg to the controllers.",
            ),
            DeclareLaunchArgument(
                name="ctrl",
                default_value="joint_trajectory_controller",
                description="Desired default controller. One of specified in ctrl_cfg.",
                choices=[
                    "admittance_controller",
                    "forward_position_controller",
                    "joint_trajectory_controller",
                    "lbr_joint_position_command_controller",
                    "lbr_torque_command_controller",
                    "lbr_wrench_command_controller",
                    "twist_controller",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                FindExecutable(name="xacro"),
                                " ",
                                PathSubstitution(FindPackageShare("lbr_description"))
                                / "urdf"
                                / LaunchConfiguration("model")
                                / LaunchConfiguration("model"),
                                ".xacro",
                                " robot_name:=",
                                LaunchConfiguration("robot_name"),
                                " mode:=hardware",
                                " system_config_path:=",
                                PathSubstitution(
                                    FindPackageShare(LaunchConfiguration("sys_cfg_pkg"))
                                )
                                / LaunchConfiguration("sys_cfg"),
                            ]
                        )
                    },
                    {"use_sim_time": False},
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"use_sim_time": False},
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("ctrl_cfg_pkg"))
                    )
                    / LaunchConfiguration("ctrl_cfg"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager",
                    "controller_manager",
                    "joint_state_broadcaster",
                    "estimated_wrench_interface",
                    "lbr_state_broadcaster",
                    "force_torque_broadcaster",
                    LaunchConfiguration("ctrl"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
        ]
    )
