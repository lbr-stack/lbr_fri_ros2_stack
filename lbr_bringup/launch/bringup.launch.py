from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        name='rviz_config',
        default_value='config/config.rviz',
        description='Rviz configuration relative to description_package.'
    ))

    launch_args.append(DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
        description="Set robot name."
    ))

    launch_args.append(DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Launch robot in simulation or on real setup."
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

    # Load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare(LaunchConfiguration("description_package")), LaunchConfiguration("description_file")]
            ), " ",
            "origin_xyz:=", LaunchConfiguration("origin_xyz"), " ",
            "origin_rpy:=", LaunchConfiguration("origin_rpy"), " ",
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
                "control.launch.py"
            ])
        ), launch_arguments=[
            ("controller_configurations_package", LaunchConfiguration("controller_configurations_package")),
            ("controller_configurations", LaunchConfiguration("controller_configurations")),
            ("robot_description", robot_description_content),
            ("controller", LaunchConfiguration("controller")),
            ("sim", LaunchConfiguration("sim"))
        ]
    )

    # Gazebo simulation or real robot    
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "simulation.launch.py"
            ])
        ), 
        launch_arguments=[
           ("entity", "lbr") # TODO move name to declare launch arguments
        ],
        condition=IfCondition(LaunchConfiguration("sim"))
    )

    # real = IncludeLaunchDescription(
    #     ...
    #     condition=UnlessCondition(LaunchConfiguration("sim"))
    # )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'config': PathJoinSubstitution(
            [FindPackageShare(LaunchConfiguration('description_package')), LaunchConfiguration('rviz_config')]
        )}]
    )

    return LaunchDescription(
        launch_args+ [
            simulation,
            # real,
            control,
            # rviz
    ])
