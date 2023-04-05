from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from lbr_bringup import LBRBringup


def configure_lbr(context):
    model = LaunchConfiguration("model").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    if sim in ["True", "true"]:
        sim = True
    elif sim in ["False", "false"]:
        sim = False
    else:
        raise ValueError("Received sim of invalid value.")
    controller_package = LaunchConfiguration("controller_package").perform(context)
    controller_file = LaunchConfiguration("controller_file").perform(context)
    controller = LaunchConfiguration("controller").perform(context)

    lbr_bringup = LBRBringup(sim=sim)
    lbr_bringup.add_robot_description(
        package="lbr_description", xacro_file=f"urdf/{model}/{model}.urdf.xacro"
    ).add_controller_manager(
        package=controller_package, controller_configurations_file=controller_file
    ).add_robot().add_controller(
        "joint_state_broadcaster"
    ).add_controller(
        controller
    ).add_robot_state_publisher().add_rviz2()

    return lbr_bringup.launch_description.entities


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="iiwa7",
        description="The LBR model in use.",
        choices=["iiwa7", "iiwa14", "med7", "med14"],
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
            sim_arg,
            controller_package_arg,
            controller_file_arg,
            controller_arg,
            OpaqueFunction(function=configure_lbr),
        ]
    )
