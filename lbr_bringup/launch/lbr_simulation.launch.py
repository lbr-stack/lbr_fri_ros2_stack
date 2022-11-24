from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
        description="Set robot name."
    ))

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

    # Note: Environment variable GAZEBO_MODEL_PATH is extended as in 
    # ROS2 control demos via environment hook https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot_description
    # Also see https://colcon.readthedocs.io/en/released/developer/environment.html#dsv-files
    # Gazebo launch scripts append GAZEBO_MODEL_PATH with known paths, see https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ab1ae5c05eda62674b36df74eb3be8c93cdc8761/gazebo_ros/launch/gzclient.launch.py#L26
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", LaunchConfiguration("robot_name")
        ],
        output="screen"
    )

    return LaunchDescription(
        launch_args + [
            gazebo,
            spawn_entity
    ])
