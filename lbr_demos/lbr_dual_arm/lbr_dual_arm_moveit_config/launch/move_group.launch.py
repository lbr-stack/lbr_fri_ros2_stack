import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("mode", default_value="mock"))

    moveit_config = (
        MoveItConfigsBuilder(
            "lbr_dual_arm",
            package_name="lbr_dual_arm_moveit_config",
        )
        .robot_description(
            os.path.join(
                get_package_share_directory("lbr_dual_arm_description"),
                "urdf/lbr_dual_arm.xacro",
            ),
            mappings={"mode": LaunchConfiguration("mode")},
        )
        .to_moveit_configs()
    )
    for entity in generate_move_group_launch(moveit_config).entities:
        ld.add_action(entity)
    return ld
