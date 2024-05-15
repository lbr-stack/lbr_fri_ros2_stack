from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class LBRROS2ControlMixin:
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
            default_value="config/lbr_controllers.yaml",
            description="Relative path from ctrl_cfg_pkg to the controllers.",
        )

    @staticmethod
    def arg_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl",
            default_value="joint_trajectory_controller",
            description="Desired default controller. One of specified in ctrl_cfg.",
            choices=[
                "joint_trajectory_controller",
                "forward_position_controller",
                "lbr_joint_position_command_controller",
                "lbr_torque_command_controller",
                "lbr_wrench_command_controller",
            ],
        )

    @staticmethod
    def arg_use_sim_time() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )

    @staticmethod
    def node_ros2_control(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        **kwargs,
    ) -> Node:
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
                            "ctrl_cfg", default="config/lbr_controllers.yaml"
                        ),
                    ]
                ),
            ],
            namespace=robot_name,
            remappings=[
                ("~/robot_description", "robot_description"),
            ],
            **kwargs,
        )

    @staticmethod
    def node_controller_spawner(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        controller: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "ctrl"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller,
                "--controller-manager",
                "controller_manager",
            ],
            namespace=robot_name,
            **kwargs,
        )

    @staticmethod
    def node_robot_state_publisher(
        robot_description: Dict[str, str],
        robot_name: Optional[LaunchConfiguration] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        use_sim_time: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "use_sim_time", default="false"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
                # use robot name as frame prefix
                {
                    "frame_prefix": PathJoinSubstitution([robot_name, ""])
                },  # neat hack to add trailing slash, which is required by frame_prefix
            ],
            namespace=robot_name,
            **kwargs,
        )
