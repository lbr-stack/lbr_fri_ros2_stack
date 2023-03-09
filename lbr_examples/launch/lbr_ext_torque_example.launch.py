import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("lbr_storz_tilt_endoscope_description"),
                    "urdf/lbr_storz_tilt_endoscope.urdf.xacro",
                ]
            ),
        ]
    )

    urdf_path = os.path.join(
        get_package_share_directory("lbr_storz_tilt_endoscope_description"),
        "urdf/lbr_storz_tilt_endoscope.urdf.xacro",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
        arguments=[urdf_path],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lbr_examples"), "rviz_config/ext_torque.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            urdf_path,
        ],
        arguments=["-d", rviz_config],
    )

    # Joint state publisher
    lbr_joint_state_publisher = Node(
        package="lbr_examples",
        executable="lbr_joint_state_publisher.py",
    )

    # Example script
    lbr_ext_torque_example = Node(
        package="lbr_examples",
        executable="lbr_ext_torque_example.py",
        parameters=[{"lbr_description": robot_description_content}],
    )

    return LaunchDescription(
        [rviz, robot_state_publisher, lbr_joint_state_publisher, lbr_ext_torque_example]
    )
