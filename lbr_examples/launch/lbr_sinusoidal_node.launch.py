import math

from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch_arg = DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Whether to launch in simulation."
    )

    amplitude_launch_arg = DeclareLaunchArgument(
        name="amplitude",
        default_value=f"{math.pi/4.}",
        description="Rotation amplitude of joint 6 in radians."
    )

    period_launch_arg = DeclareLaunchArgument(
        name="period",
        default_value=f"{20.}",
        description="Rotation period in seconds."
    )

    sinusoidal_node = Node(
        package="lbr_examples",
        executable="lbr_sinusoidal_node.py",
        parameters=[
            {"sim": LaunchConfiguration("sim")},
            {"amplitude": LaunchConfiguration("amplitude")},
            {"period": LaunchConfiguration("period")}
        ]
    )

    return LaunchDescription(
        [
            sim_launch_arg,
            amplitude_launch_arg,
            period_launch_arg,
            sinusoidal_node
        ]
    )
