import os
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from .lbr_bringup import LBRBringUp
from .utilities import string_to_bool


def lbr_launch(context: LaunchContext):
    model = LaunchConfiguration("model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    sim = string_to_bool(LaunchConfiguration("sim").perform(context))

    controller_package = LaunchConfiguration("controller_package").perform(context)
    controller_file = LaunchConfiguration("controller_file").perform(context)
    controller = LaunchConfiguration("controller").perform(context)

    lbr_bringup = LBRBringUp(robot_name=robot_name, sim=sim)
    lbr_bringup.add_robot_description(
        package="lbr_description", xacro_file=f"urdf/{model}/{model}.urdf.xacro"
    ).add_robot().add_controller_manager(
        package=controller_package, controller_configurations_file=controller_file
    ).add_controller(
        "joint_state_broadcaster"
    ).add_controller(
        controller
    ).add_robot_state_publisher().add_rviz2(
        parameters=[lbr_bringup.robot_description]
    )

    return lbr_bringup.launch_description.entities


def lbr_with_moveit_launch(context: LaunchContext):
    model = LaunchConfiguration("model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    sim = string_to_bool(LaunchConfiguration("sim").perform(context))

    controller_package = LaunchConfiguration("controller_package").perform(context)
    controller_file = LaunchConfiguration("controller_file").perform(context)
    controller = LaunchConfiguration("controller").perform(context)

    lbr_bringup = LBRBringUp(robot_name=robot_name, sim=sim)
    lbr_bringup.add_robot_description(
        package="lbr_description", xacro_file=f"urdf/{model}/{model}.urdf.xacro"
    ).add_robot().add_controller_manager(
        package=controller_package, controller_configurations_file=controller_file
    ).add_controller(
        "joint_state_broadcaster"
    ).add_controller(
        controller
    ).add_robot_state_publisher()

    moveit_config_builder = MoveItConfigsBuilder(
        robot_name="lbr",
    )
    moveit_config_builder.robot_description(
        os.path.join(
            get_package_share_directory("lbr_description"),
            f"urdf/{model}/{model}.urdf.xacro",
        ),
        mappings={
            "robot_name": lbr_bringup.robot_name,
            "sim": str(lbr_bringup.sim),
        },
    ).robot_description_semantic(
        os.path.join(
            get_package_share_directory("lbr_moveit_config"), "config/lbr.srdf"
        )
    ).robot_description_kinematics(
        os.path.join(
            get_package_share_directory("lbr_moveit_config"), "config/kinematics.yaml"
        )
    ).trajectory_execution(
        os.path.join(
            get_package_share_directory("lbr_moveit_config"),
            "config/moveit_controllers.yaml",
        )
    ).joint_limits(
        os.path.join(
            get_package_share_directory("lbr_moveit_config"),
            "config/joint_limits.yaml",
        )
    ).pilz_cartesian_limits(
        os.path.join(
            get_package_share_directory("lbr_moveit_config"),
            "config/pilz_cartesian_limits.yaml",
        )
    )
    moveit_config = moveit_config_builder.to_moveit_configs()
    moveit_launch_description = generate_move_group_launch(moveit_config=moveit_config)

    lbr_bringup.add_rviz2(
        package="lbr_bringup",
        rviz2_config_file="config/config_with_moveit.rviz",
        parameters=[
            lbr_bringup.robot_description,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )

    return lbr_bringup.launch_description.entities + moveit_launch_description.entities
