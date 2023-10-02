from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class LBRSystemInterfaceMixin:
    @staticmethod
    def arg_ctrl_cfg_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg_pkg",
            default_value="lbr_ros2_control",
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
            choices=["position_trajectory_controller", "forward_position_controller"],
        )

    @staticmethod
    def arg_frame_prefix() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="frame_prefix",
            default_value="",
            description="Prefix for the tf frame names. Useful for multi-robot setups. E.g. 'robot1/'",
        )

    @staticmethod
    def arg_use_sim_time() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )

    @staticmethod
    def param_frame_prefix() -> Dict[str, LaunchConfiguration]:
        return {"frame_prefix": LaunchConfiguration("frame_prefix", default="")}

    @staticmethod
    def node_ros2_control(robot_description: Dict[str, str], **kwargs) -> Node:
        return Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"use_sim_time": False},
                PathJoinSubstitution(
                    [
                        FindPackageShare(
                            LaunchConfiguration(
                                "ctrl_cfg_pkg", default="lbr_ros2_control"
                            )
                        ),
                        LaunchConfiguration(
                            "ctrl_cfg", default="config/lbr_controllers.yml"
                        ),
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
                LaunchConfiguration("ctrl", default="position_trajectory_controller"),
                "--controller-manager",
                "/controller_manager",
            ],
            **kwargs
        )

    @staticmethod
    def node_robot_state_publisher(
        robot_description: Dict[str, str],
        use_sim_time: Optional[Union[LaunchConfiguration, bool]] = None,
        frame_prefix: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs
    ) -> Node:
        if use_sim_time is None:
            use_sim_time = LaunchConfiguration("use_sim_time", default="false")
        if frame_prefix is None:
            frame_prefix = LaunchConfiguration("frame_prefix", default="")
        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
                {"frame_prefix": frame_prefix},
            ],
            **kwargs
        )
