# launch gazebo

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions.execute_process import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

from gazebo_msgs.srv import SpawnEntity
# from rclpy.node import Node

# read launch description files med7/...

def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(DeclareLaunchArgument(
        name="description_package",
        default_value="lbr_description",
        description="Description package."
    ))

    launch_args.append(DeclareLaunchArgument(
        name="description_file",
        default_value="urdf/iiwa7/iiwa7.urdf.xacro",
        description="Path to URDF file, relative to description_package."
    ))

    launch_args.append(DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
        description="Set robot name."
    ))

    launch_args.append(DeclareLaunchArgument(
        name="origin_xyz",
        default_value="'0 0 0'",
        description="Set position origin of robot."
    ))

    launch_args.append(DeclareLaunchArgument(
        name="origin_rpy",
        default_value="'0 0 0'",
        description="Set orientation origin of robot."
    ))

    launch_args.append(
        DeclareLaunchArgument(
            name="controller_configurations_package",
            default_value="lbr_bringup",
            description="Package that contains controller configurations."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller_configurations",
            default_value="config/lbr_controllers.yml",
            description="Relative path to controller configurations YAML file."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller",
            default_value="forward_position_controller",
            description="Robot controller."
        )
    )


    # # make gazebo aware of files
    # install_dir = get_package_prefix("lbr_description")

    # if "GAZEBO_MODEL_PATH" in os.environ:
    #     os.environ["GAZEBO_MODEL_PATH"] =  os.environ["GAZEBO_MODEL_PATH"] + ":" + install_dir + "/share"
    # else:
    #     os.environ["GAZEBO_MODEL_PATH"] =  install_dir + "/share"


    # # see /opt/ros/foxy/share/gazebo_ros/launch
    # # spawns through node @ /opt/ros/foxy/lib/gazebo_ros

    # # load gazebo launch description
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("gazebo_ros"), 
    #             "launch", 
    #             "gazebo.launch.py"
    #         ])
    #     )
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "gazebo_dec.launch.py"
            ])
        ), launch_arguments=[
           ("entity", "lbr")
        ]
    )


    # file = xacro.process_file()
    # file = file.toxml()


    # load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare(LaunchConfiguration("description_package")), LaunchConfiguration("description_file")]
            ), " ",
            "origin_xyz:=", LaunchConfiguration("origin_xyz"), " ",
            "origin_rpy:=", LaunchConfiguration("origin_rpy"), " ",
            "robot_name:=", LaunchConfiguration("robot_name")
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # # better way to spawn robot into gazebo? https://jeffzzq.medium.com/designing-a-ros2-robot-7c31a62c535a
    # # spawn robot
    # spawn_entity = Node(package="gazebo_ros",
    # executable="spawn_entity.py",
    # arguments=[
    #     "-topic", "robot_description",
    #     "-entity", "some_name"],
    # output="screen")

    # spawner_node = Node(
    #     package="lbr_description",
    #     executable="spawn_lbr.py",
    #     output="screen",
    #     parameters=[robot_description]
    # )




    # launch controllers https://bitbucket.org/theconstructcore/box_car/src/foxy/box_car_description/launch/steering_control.launch.py
    # https://github.com/ros-controls/ros2_control_demos/tree/ead1af151d92b104774e9061684cf23b52f67507

    # robot state publisher http://wiki.ros.org/robot_state_publisher
    # [...] The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links


    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "control.launch.py"
            ])
        ), launch_arguments=[
            ("controller_configurations_package", LaunchConfiguration("controller_configurations_package")),
            ("controller_configurations", LaunchConfiguration("controller_configurations")),
            ("robot_description", robot_description_content),
            ("controller", LaunchConfiguration("controller"))
        ]
    )

    return LaunchDescription(
        launch_args +
        [
            gazebo,
            control,
            # spawner_node
        ]
    )
