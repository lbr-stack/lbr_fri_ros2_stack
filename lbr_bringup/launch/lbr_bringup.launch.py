from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from lbr_bringup import LBRBringUp


def configure_lbr(context):
    model = LaunchConfiguration("model").perform(context)

    # Load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("lbr_description"), "urdf/{}/{}.urdf.xacro".format(model, model)]
            ), " ",
            "robot_name:=", LaunchConfiguration("robot_name"), " ",
            "sim:=", LaunchConfiguration("sim")
        ]
    )

    # Load controls
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "lbr_control.launch.py"
            ])
        ), launch_arguments=[
            ("robot_description", robot_description_content),
            ("controller_configurations_package", LaunchConfiguration("controller_configurations_package")),
            ("controller_configurations", LaunchConfiguration("controller_configurations")),
            ("controller", LaunchConfiguration("controller")),
            ("sim", LaunchConfiguration("sim"))
        ]
    )

    # Gazebo simulation 
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "lbr_simulation.launch.py"
            ])
        ), 
        launch_arguments=[
           ("robot_name", LaunchConfiguration("robot_name"))
        ],
        condition=IfCondition(LaunchConfiguration("sim"))
    )

    # Move group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_moveit"),
                "launch",
                "lbr_move_group.launch.py"
            ])
        ), 
        launch_arguments=[
            ("robot_description", robot_description_content),
            ("moveit_controller_configurations_package", LaunchConfiguration("moveit_controller_configurations_package")),
            ("moveit_controller_configurations", LaunchConfiguration("moveit_controller_configurations")),
            ("model", LaunchConfiguration("model")),
            ("sim", LaunchConfiguration("sim"))
        ]
    )

    return [
        simulation,
        control,
        # move_group
    ]


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
