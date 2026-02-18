from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
                description="The robot's name. Links in the tf tree will be prefixed as <robot_name>_link. Same applies to joints. The robot's name will be used as namespace.",
            ),
            DeclareLaunchArgument(
                name="init_jnt_pos_pkg",
                default_value="lbr_description",
                description="Package containing the initial_joint_positions.yaml file.",
            ),
            DeclareLaunchArgument(
                name="init_jnt_pos",
                default_value="ros2_control/initial_joint_positions.yaml",
                description="The relative path from sys_cfg_pkg to the initial_joint_positions.yaml file.",
            ),
            DeclareLaunchArgument(
                name="ctrl",
                default_value="joint_trajectory_controller",
                description="Desired default controller. Gazebo loads controller configuration through lbr_description/gazebo/*.xacro from lbr_description/ros2_control/gazebo_controllers.yaml.",
                choices=[
                    "forward_position_controller",
                    "joint_trajectory_controller",
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
                                " mode:=gazebo",
                                " initial_joint_positions_path:=",
                                PathSubstitution(
                                    FindPackageShare(
                                        LaunchConfiguration("init_jnt_pos_pkg")
                                    )
                                )
                                / LaunchConfiguration(
                                    "init_jnt_pos",
                                ),
                            ]
                        )
                    },
                    {"use_sim_time": True},
                ],
                namespace=LaunchConfiguration("robot_name"),
            ),
            IncludeLaunchDescription(
                PathSubstitution(
                    FindPackageShare("ros_gz_sim"),
                )
                / "launch"
                / "gz_sim.launch.py",
                launch_arguments={"gz_args": "-r empty.sdf"}.items(),
            ),  # Gazebo has its own controller manager
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-allow_renaming",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                    "-R",
                    "0.0",
                    "-P",
                    "0.0",
                    "-Y",
                    "0.0",
                ],
                output="screen",
                namespace=LaunchConfiguration("robot_name"),
            ),  # spawns robot in Gazebo through robot_description topic of robot_state_publisher
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager",
                    "controller_manager",
                    "joint_state_broadcaster",
                    LaunchConfiguration("ctrl"),
                ],
                namespace=LaunchConfiguration("robot_name"),
            ),
        ]
    )
