from typing import Dict, Optional, Union

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


class LBRDescriptionLaunch:
    @staticmethod
    def description(
        model: Union[LaunchConfiguration, str] = None,
        robot_name: Union[LaunchConfiguration, str] = None,
        sim: Union[LaunchConfiguration, bool] = None,
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
    def model_arg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="The LBR model in use.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )

    @staticmethod
    def robot_name_arg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr",
            description="The robot's name.",
        )

    @staticmethod
    def sim_arg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Whether to use the simulation or not.",
        )
