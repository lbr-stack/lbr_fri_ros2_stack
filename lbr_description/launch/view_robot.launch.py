import os
from typing import List

from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lbr_description import description_factory


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    model = LaunchConfiguration("model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)

    robot_description = description_factory(model=model, robot_name=robot_name)

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("lbr_description"), "config/config.rviz"
            ),
        ],
    )

    return [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="Set robot model.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name", default_value="lbr", description="Set robot name."
        )
    )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
