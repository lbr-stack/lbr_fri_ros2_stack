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
                name="namespace",
                default_value="",
                description="Nodes in this launch file will be spawned with this namespace.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_x",
                default_value="0",
                description="X position of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_y",
                default_value="0.5",
                description="Y position of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_z",
                default_value="0",
                description="Z position of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_roll",
                default_value="0",
                description="Roll orientation of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_pitch",
                default_value="0.0",
                description="Pitch orientation of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_one_yaw",
                default_value="0",
                description="Yaw orientation of the first robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_x",
                default_value="0",
                description="X position of the second robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_y",
                default_value="-0.5",
                description="Y position of the second robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_z",
                default_value="0",
                description="Z position of the second robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_roll",
                default_value="0",
                description="Roll orientation of the second robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_pitch",
                default_value="0.0",
                description="Pitch orientation of the second robot.",
            ),
            DeclareLaunchArgument(
                name="lbr_two_yaw",
                default_value="0",
                description="Yaw orientation of the second robot.",
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
                                PathSubstitution(FindPackageShare("lbr_dual_arm"))
                                / "urdf"
                                / "lbr_dual_arm.xacro",
                                " mode:=mock",
                                " lbr_one_x:=",
                                LaunchConfiguration("lbr_one_x"),
                                " lbr_one_y:=",
                                LaunchConfiguration("lbr_one_y"),
                                " lbr_one_z:=",
                                LaunchConfiguration("lbr_one_z"),
                                " lbr_one_roll:=",
                                LaunchConfiguration("lbr_one_roll"),
                                " lbr_one_pitch:=",
                                LaunchConfiguration("lbr_one_pitch"),
                                " lbr_one_yaw:=",
                                LaunchConfiguration("lbr_one_yaw"),
                                " lbr_two_x:=",
                                LaunchConfiguration("lbr_two_x"),
                                " lbr_two_y:=",
                                LaunchConfiguration("lbr_two_y"),
                                " lbr_two_z:=",
                                LaunchConfiguration("lbr_two_z"),
                                " lbr_two_roll:=",
                                LaunchConfiguration("lbr_two_roll"),
                                " lbr_two_pitch:=",
                                LaunchConfiguration("lbr_two_pitch"),
                                " lbr_two_yaw:=",
                                LaunchConfiguration("lbr_two_yaw"),
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
                    PathSubstitution(FindPackageShare("lbr_dual_arm"))
                    / "config"
                    / "dual_arm_controllers.yaml",
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
                    "joint_trajectory_controller",
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
        ]
    )
