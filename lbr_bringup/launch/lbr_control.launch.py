from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        package="lbr_hardware",
        executable="fri_ros2_control_node",
        parameters=[robot_description, controller_configurations],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    lbr_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lbr_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(LaunchConfiguration("sim"))
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("controller"), "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        launch_args + [
            controller_manager,
            robot_state_publisher,
            joint_state_broadcaster,
            lbr_state_broadcaster,
            controller
        ])
