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
    @staticmethod
    def include_gazebo(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            **kwargs
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> Node:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name")
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
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
    ) -> Dict[str, str]:
        if model is None:
            model = LaunchConfiguration("model")
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name")
        if sim is None:
            sim = LaunchConfiguration("sim")
        if type(sim) is bool:
            sim = "true" if sim else "false"
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
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name")}


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
            rviz_config_pkg = LaunchConfiguration("rviz_config_pkg")
        if rviz_config is None:
            rviz_config = LaunchConfiguration("rviz_config")
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
