import sys
import warnings

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Configure robot_description
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}

    # Load controllers from YAML configuration file
    controller_configurations = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("controller_configurations_package")),
        LaunchConfiguration("controller_configurations")
    ])

    # Prepare controller manager and other required nodes
    real_time = LaunchConfiguration("real_time").perform(context)
    valid_real_time_arguments = ["true", "false", "1", "0"]
    if not real_time in valid_real_time_arguments:
        raise ValueError(f"Invalid real_time launch argument. Expected one of {valid_real_time_arguments}, got {real_time}.")

    controller_manager_prefix = ""
    if real_time in ["true", "1"]:
        if sys.platform.startswith("win"):
            # currently not supported. To be implemented with ExecuteCommand after fri_ros2_control_node launch
            # https://superuser.com/questions/620724/changing-windows-process-priority-via-command-line 
            warnings.warn("Windows currently not supported for real-time priority. Defaulting to non-real-time.")
        elif sys.platform.startswith("lin"):
            # launch with realtime priority, requries to set rtprio in /etc/security/limits.conf, e.g. <user> - rtprio 99
            controller_manager_prefix = "chrt -rr 99"
        else:
            raise RuntimeError(f"Encountered unhandled platform {sys.platform}")

    controller_manager = Node(
        package="lbr_hardware",
        executable="fri_ros2_control_node",
        parameters=[robot_description, controller_configurations],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("sim")),
        prefix=controller_manager_prefix
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

    return [
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster,
        lbr_state_broadcaster,
        controller
    ]


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

    launch_args.append(
        DeclareLaunchArgument(
            name="real_time",
            default_value="false",
            description=
                "Will launch ros2_control_node with real-time priority.\n"
                "\tCurrently only supported on Linux. Requires user to set rtprio\n"
                "\tin /etc/security/limits.conf, see https://linux.die.net/man/5/limits.conf.\n"
                "\tE.g. <user> - rtprio 99."
        )
    )

    return LaunchDescription(
        launch_args + [
            OpaqueFunction(function=launch_setup)
    ])
