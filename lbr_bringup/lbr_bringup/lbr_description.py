from typing import Dict, List, Optional, Union

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
            **kwargs,
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        **kwargs,
    ) -> Node:
        label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
        tf = [str(x) for x in tf]
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                robot_name,
            ]
            + [item for pair in zip(label, tf) for item in pair],
            output="screen",
            namespace=robot_name,
            **kwargs,
        )


class LBRDescriptionMixin:
    @staticmethod
    def param_robot_description(
        model: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "model", default="iiwa7"
        ),
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        port_id: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "port_id", default="30200"
        ),
        sim: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "sim", default="true"
        ),
    ) -> Dict[str, str]:
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
                    ".xacro",
                    " robot_name:=",
                    robot_name,
                    " port_id:=",
                    port_id,
                    " sim:=",
                    sim,
                ]
            )
        }
        return robot_description

    @staticmethod
    def arg_model(default_value: str = "iiwa7") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value=default_value,
            description="The LBR model in use.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )

    @staticmethod
    def arg_robot_name(default_value: str = "lbr") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value=default_value,
            description="The robot's name.",
        )

    @staticmethod
    def arg_port_id(default_value: str = "30200") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="port_id",
            default_value=default_value,
            description="Port ID of the FRI communication. Valid in range [30200, 30209].\n"
            "\tUsefull in multi-robot setups.",
        )

    @staticmethod
    def arg_sim(default_value: str = "true") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value=default_value,
            description="Whether to use the simulation or not.",
        )

    @staticmethod
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name", default="lbr")}

    @staticmethod
    def param_port_id() -> Dict[str, LaunchConfiguration]:
        return {"port_id": LaunchConfiguration("port_id", default="30200")}

    @staticmethod
    def param_sim() -> Dict[str, LaunchConfiguration]:
        return {"sim": LaunchConfiguration("sim", default="true")}

    @staticmethod
    def node_static_tf(
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        parent: Optional[Union[LaunchConfiguration, str]] = None,
        child: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs,
    ) -> Node:
        label = ["--x", "--y", "--z", "--roll", "--pitch", "--yaw"]
        tf = [str(x) for x in tf]
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[item for pair in zip(label, tf) for item in pair]
            + [
                "--frame-id",
                parent,
                "--child-frame-id",
                child,
            ],
            **kwargs,
        )


class RVizMixin:
    @staticmethod
    def arg_rviz_config_pkg(
        default_value: str = "lbr_description",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config(
        default_value: str = "config/config.rviz",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_pkg: Optional[
            Union[LaunchConfiguration, str]
        ] = LaunchConfiguration("rviz_config_pkg", default="lbr_description"),
        rviz_config: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "rviz_config", default="config/config.rviz"
        ),
        **kwargs,
    ) -> Node:
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
            **kwargs,
        )
