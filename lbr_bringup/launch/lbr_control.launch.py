from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch arguments
    launch_args = []

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
            name="robot_description",
            description="Robot description XML file."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller",
            default_value="position_trajectory_controller",
            description="Robot controller."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Launch robot in simulation or on real setup."
        )
    )

    # Configure robot_description
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}

    # Load controllers from YAML configuration file
    controller_configurations = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("controller_configurations_package")),
        LaunchConfiguration("controller_configurations")
    ])

    # Prepare controller manager and other required nodes
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_configurations],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("sim")),
        prefix="chrt -rr 99"  # launch with realtime priority, requries to set rtprio in /etc/security/limits.conf, e.g. <user> - rtprio 99
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[LaunchConfiguration("controller"), "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        launch_args + [
            controller_manager,
            robot_state_publisher,
            joint_state_broadcaster,
            controller
    ])
