from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_configurations_package_arg = DeclareLaunchArgument(
        name="controller_configurations_package",
        default_value="lbr_bringup",
        description="Package that contains controller configurations."
    )

    controller_configurations_arg = DeclareLaunchArgument(
        name="controller_configurations",
        default_value="config/lbr_controllers.yml",
        description="Relative path to controller configurations YAML file."
    )

    robot_description_arg =  DeclareLaunchArgument(
        name="robot_description",
        description="Robot description XML file."
    )

    controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value="position_trajectory_controller",
        description="Robot controller."
    )

    sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Launch robot in simulation or on real setup."
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
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("controller"), "--controller-manager", "/controller_manager"],
    )

    controllers_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster, controller]
        )
    )

    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description]
            )
    
    robot_state_publisher_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot_state_publisher]
        )
    )
    
    return LaunchDescription([
        controller_configurations_package_arg,
        controller_configurations_arg,
        robot_description_arg,
        controller_arg,
        sim_arg, 
        controller_manager,
        controllers_event_handler,
        robot_state_publisher_event_handler
    ])
