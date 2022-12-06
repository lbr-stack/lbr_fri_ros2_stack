import os
import math
import xacro
from ament_index_python import get_package_share_directory
from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    sim_launch_arg = DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Whether to launch in simulation."
    )

    command_rate_launch_arg = DeclareLaunchArgument(
        name="command_rate",
        default_value=f"{100}",
        description="Command rate in Hz."
    )

    path_to_urdf = get_package_share_directory('lbr_description')
    path = os.path.join(path_to_urdf, 'urdf', 'med7', 'med7.urdf.xacro')
    xml = xacro.process(path)

    force_control_node = Node(
        package="lbr_examples",
        executable="lbr_force_control_node.py",
        parameters=[
            {"sim": LaunchConfiguration("sim")},
            {"command_rate": LaunchConfiguration("command_rate")},
            {"robot_description": xml},
        ]
    )

    return LaunchDescription(
        [
            sim_launch_arg,
            command_rate_launch_arg,
            force_control_node,
        ]
    )
