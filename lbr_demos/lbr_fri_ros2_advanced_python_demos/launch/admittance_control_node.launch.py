import os

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> LaunchDescription:
    model = LaunchConfiguration("model").perform(context)

    robot_description = {
        "robot_description": xacro.process(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf",
                model,
                f"{model}.urdf.xacro",
            ),
            mappings={"sim": "false"},
        )
    }

    admittance_control_node = Node(
        package="lbr_fri_ros2_advanced_python_demos",
        executable="admittance_control_node",
        output="screen",
        parameters=[robot_description],
    )

    return [admittance_control_node]


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="iiwa7",
        description="The LBR model in use.",
        choices=["iiwa7", "iiwa14", "med7", "med14"],
    )
    return LaunchDescription([model_arg, OpaqueFunction(function=launch_setup)])
