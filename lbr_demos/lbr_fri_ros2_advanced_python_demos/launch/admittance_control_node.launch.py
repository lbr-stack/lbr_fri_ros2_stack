import os

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    model = "med7"
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

    return LaunchDescription([admittance_control_node])
