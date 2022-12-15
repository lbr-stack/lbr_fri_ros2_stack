from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from lbr_bringup.launch_bundles import lbr_with_moveit_launch


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="iiwa7",
        description="The LBR model in use.",
        choices=["iiwa7", "iiwa14", "med7", "med14"],
    )

    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
        description="The robot's name."
    )

    sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Whether to launch simulation or real robot.",
        choices=["True", "true", "False", "false"],
    )

    controller_package_arg = DeclareLaunchArgument(
        name="controller_package",
        default_value="lbr_bringup",
        description="The package containing the controller_file.",
    )

    controller_file_arg = DeclareLaunchArgument(
        name="controller_file",
        default_value="config/lbr_controllers.yml",
        description="Relative path from controller_package to the controllers.",
    )

    controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value="position_trajectory_controller",
        description="Desired default controller. One of specified in controller_file_arg.",
    )

    return LaunchDescription(
        [
            model_arg,
            robot_name_arg,
            sim_arg,
            controller_package_arg,
            controller_file_arg,
            controller_arg,
            OpaqueFunction(function=lbr_with_moveit_launch),
        ]
    )
