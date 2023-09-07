from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class GazeboMixin:
    # https://answers.gazebosim.org//question/28813/how-to-spawn-a-urdf-robot-into-a-ignition-gazebo-world-from-ros2/
    @staticmethod
    def include_gazebo(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                ),
            ),
            launch_arguments={"gz_args": "-r empty.sdf"}.items(),
            **kwargs
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> Node:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name")
        return Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                LaunchConfiguration("robot_name"),
            ],
            output="screen",
            **kwargs
        )


class LBRDescriptionMixin:
    @staticmethod
    def param_robot_description(
        model: Optional[Union[LaunchConfiguration, str]] = None,
        robot_name: Optional[Union[LaunchConfiguration, str]] = None,
        sim: Optional[Union[LaunchConfiguration, bool]] = None,
        base_frame: Optional[Union[LaunchConfiguration, str]] = None,
    ) -> Dict[str, str]:
        if model is None:
            model = LaunchConfiguration("model", default="iiwa7")
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name", default="lbr")
        if sim is None:
            sim = LaunchConfiguration("sim", default="true")
        if type(sim) is bool:
            sim = "true" if sim else "false"
        if base_frame is None:
            base_frame = LaunchConfiguration("base_frame", default="world")
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_description"),
                            "urdf",
                            model,
                            model,
                        ]
                    ),
                    ".urdf.xacro",
                    " robot_name:=",
                    robot_name,
                    " sim:=",
                    sim,
                    " base_frame:=",
                    base_frame,
                ]
            )
        }
        return robot_description

    @staticmethod
    def arg_model() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="The LBR model in use.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )

    @staticmethod
    def arg_base_frame() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="base_frame",
            default_value="world",
            description="The robot's base frame.",
        )

    @staticmethod
    def arg_robot_name() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr",
            description="The robot's name.",
        )

    @staticmethod
    def arg_sim() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Whether to use the simulation or not.",
        )

    @staticmethod
    def param_base_frame() -> Dict[str, LaunchConfiguration]:
        return {"base_frame": LaunchConfiguration("base_frame", default="world")}

    @staticmethod
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name", default="lbr")}

    @staticmethod
    def param_sim() -> Dict[str, LaunchConfiguration]:
        return {"sim": LaunchConfiguration("sim", default="true")}


class RVizMixin:
    @staticmethod
    def arg_rviz_config_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value="lbr_description",
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value="config/config.rviz",
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_pkg: Optional[Union[LaunchConfiguration, str]] = None,
        rviz_config: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs
    ) -> Node:
        if rviz_config_pkg is None:
            rviz_config_pkg = LaunchConfiguration(
                "rviz_config_pkg", default="lbr_description"
            )
        if rviz_config is None:
            rviz_config = LaunchConfiguration(
                "rviz_config", default="config/config.rviz"
            )
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(rviz_config_pkg),
                        rviz_config,
                    ]
                ),
            ],
            **kwargs
        )
