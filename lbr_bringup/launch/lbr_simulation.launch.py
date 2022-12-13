from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
        description="Set robot name."
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), 
                "launch", 
                "gazebo.launch.py"
            ])
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", LaunchConfiguration("robot_name")
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_name_arg,
        gazebo,
        spawn_entity
    ])
