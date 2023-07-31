from typing import Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


class LBRDescription(LaunchDescription):
    _robot_description: Dict[str, str]

    def __init__(
        self,
        sim: bool = True,
    ) -> None:
        super().__init__()
        self._add_launch_arguments()
        self._robot_description = self._generate_robot_description(sim)

    @property
    def robot_description(self) -> Dict[str, str]:
        return self._robot_description

    def _add_launch_arguments(self) -> None:
        self.add_action(
            DeclareLaunchArgument(
                name="model",
                default_value="iiwa7",
                description="The LBR model in use.",
                choices=["iiwa7", "iiwa14", "med7", "med14"],
            )
        )

        self.add_action(
            DeclareLaunchArgument(
                name="robot_name",
                default_value="lbr",
                description="The robot's name.",
            )
        )

    def _generate_robot_description(self, sim: bool = True) -> Dict[str, str]:
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_description"),
                            "urdf",
                            LaunchConfiguration("model"),
                            LaunchConfiguration("model"),
                        ]
                    ),
                    ".urdf.xacro",
                    " robot_name:=",
                    LaunchConfiguration("robot_name"),
                    " sim:=",
                    "true" if sim else "false",
                ]
            )
        }

        return robot_description
