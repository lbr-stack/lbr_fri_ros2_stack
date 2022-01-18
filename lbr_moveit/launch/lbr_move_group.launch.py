import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def load_file(package_name: str, file_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolut_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolut_file_path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):

    # Evaluate frequently used variables
    model = LaunchConfiguration("model").perform(context)

    # Configure robot_description
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}

    # Robot semantics SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file("lbr_moveit", "srdf/{}.srdf".format(model))
    }

    # Kinematics
    kinematics_yaml = load_yaml("lbr_moveit", "config/kinematics.yml")
    
    # Update group name
    kinematics_yaml["{}_arm".format(model)] = kinematics_yaml["group_name"]
    del kinematics_yaml["group_name"]

    # Joint limits
    robot_description_planning = {
        "robot_description_planning": PathJoinSubstitution(
            [
                FindPackageShare("lbr_moveit"),
                "config/joint_limits.yml"
            ]
        )
    }

    # Planning
    ompl_yaml = load_yaml("lbr_moveit", "config/ompl_planning.yml")

    planning = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1
        }
    }

    # Trajectory execution
    trajectory_execution = {"allow_trajectory_execution": True,
                            "moveit_manage_controllers": True}

    # Controllers
    controllers_yaml = load_yaml(
        LaunchConfiguration("moveit_controller_configurations_package").perform(context),
        LaunchConfiguration("moveit_controller_configurations").perform(context)
    )

    moveit_controllers = {"moveit_simple_controller_manager": controllers_yaml,
                          "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}

    # Planning scene
    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    # Time configuration
    use_sim_time = {"use_sim_time": LaunchConfiguration("sim")}

    # Prepare move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args"],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            robot_description_planning,
            ompl_yaml,
            planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time
        ]
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare("lbr_description"), "config/config.rviz"])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning,
            use_sim_time
        ],
        arguments=[
            '-d', rviz_config
        ]
    )

    return [
        move_group_node,
        rviz
    ]

def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            name="robot_description",
            description="Robot description XML file."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="Desired LBR model. Use model:=iiwa7/iiwa14/med7/med14."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations_package",
            default_value="lbr_moveit",
            description="Package that contains MoveIt! controller configurations."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations",
            default_value="config/lbr_controllers.yml",
            description="Relative path to MoveIt! controller configurations YAML file. Note that the joints in the controllers must be named according to the robot_name."
        )
    )

    launch_args.append(DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Launch robot in simulation or on real setup."
    ))

    return LaunchDescription(
        launch_args + [
        OpaqueFunction(function=launch_setup)
    ])
