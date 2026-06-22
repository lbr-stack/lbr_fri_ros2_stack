from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("mode", default_value="mock"))
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_x",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_y",
            default_value="0.5",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_z",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_roll",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_pitch",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_one_yaw",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_x",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_y",
            default_value="-0.5",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_z",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_roll",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_pitch",
            default_value="0.0",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="lbr_two_yaw",
            default_value="0.0",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "lbr_dual_arm",
            package_name="lbr_dual_arm_moveit_config",
        )
        .robot_description(
            Path(get_package_share_directory("lbr_dual_arm"))
            / "urdf"
            / "lbr_dual_arm.xacro",
            mappings={
                "mode": LaunchConfiguration("mode"),
                # manual pose setting until load from robot description topic properly supported: https://github.com/moveit/moveit2/issues/2291#issuecomment-4769692822
                "lbr_one_x": LaunchConfiguration("lbr_one_x"),
                "lbr_one_y": LaunchConfiguration("lbr_one_y"),
                "lbr_one_z": LaunchConfiguration("lbr_one_z"),
                "lbr_one_roll": LaunchConfiguration("lbr_one_roll"),
                "lbr_one_pitch": LaunchConfiguration("lbr_one_pitch"),
                "lbr_one_yaw": LaunchConfiguration("lbr_one_yaw"),
                "lbr_two_x": LaunchConfiguration("lbr_two_x"),
                "lbr_two_y": LaunchConfiguration("lbr_two_y"),
                "lbr_two_z": LaunchConfiguration("lbr_two_z"),
                "lbr_two_roll": LaunchConfiguration("lbr_two_roll"),
                "lbr_two_pitch": LaunchConfiguration("lbr_two_pitch"),
                "lbr_two_yaw": LaunchConfiguration("lbr_two_yaw"),
            },
        )
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
