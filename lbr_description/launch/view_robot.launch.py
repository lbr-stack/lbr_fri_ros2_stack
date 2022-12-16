from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# for reference see
# https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot_description
def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            name="description_package",
            default_value="lbr_description",
            description="Description package.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="description_file",
            default_value="urdf/med7/med7.urdf.xacro",
            description="Path to URDF file, relative to description_package.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="rviz_pkg",
            default_value="lbr_description",
            description="Package containing rviz_config.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value="config/config.rviz",
            description="Rviz configuration relative to rviz_pkg.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="robot_name", default_value="lbr", description="Set robot name."
        )
    )

    # Load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    LaunchConfiguration("description_file"),
                ]
            ),
            " ",
            "robot_name:=",
            LaunchConfiguration("robot_name"),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Create required nodes
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("rviz_pkg")),
                    LaunchConfiguration("rviz_config"),
                ]
            ),
        ],
    )

    return LaunchDescription(
        launch_args + [joint_state_publisher_node, robot_state_publisher_node, rviz]
    )
