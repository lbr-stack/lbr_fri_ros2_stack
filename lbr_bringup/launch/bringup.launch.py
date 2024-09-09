from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_mode())

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "hardware.launch.py",
                    ]
                )
            ),
            condition=LaunchConfigurationEquals(
                LaunchConfiguration("mode"), "hardware"
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "mock.launch.py",
                    ]
                )
            ),
            condition=LaunchConfigurationEquals(LaunchConfiguration("mode"), "mock"),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_bringup"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            condition=LaunchConfigurationEquals(LaunchConfiguration("mode"), "gazebo"),
        )
    )

    return ld
