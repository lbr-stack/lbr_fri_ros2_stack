import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="iiwa7",
        description="The LBR model in use.",
        choices=["iiwa7", "iiwa14", "med7", "med14"],
    )

    lbr_app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lbr_fri_ros2"),
                "launch",
                "lbr_app.launch.py",
            )
        ),
        launch_arguments=[
            ("model", LaunchConfiguration("model")),
        ],
    )

    wrench_sine_overlay_node = Node(
        package="lbr_fri_ros2_cpp_demos",
        executable="wrench_sine_overlay_node",
        output="screen",
    )

    return LaunchDescription([model_arg, lbr_app, wrench_sine_overlay_node])
