import os
from typing import List

from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.moveit import LBRMoveGroupMixin
from lbr_bringup.rviz import RVizMixin
from moveit_configs_utils import MoveItConfigsBuilder


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    moveit_configs_builder = (
        MoveItConfigsBuilder(
            robot_name="lbr_dual_arm",
            package_name="lbr_dual_arm_moveit_config",
        )
        .robot_description(
            os.path.join(
                get_package_share_directory("lbr_dual_arm_description"),
                "urdf/lbr_dual_arm.xacro",
            ),
            mappings={"mode": LaunchConfiguration("mode")},
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
    )
    move_group_params = LBRMoveGroupMixin.params_move_group()
    moveit_configs = moveit_configs_builder.to_moveit_configs()
    robot_name = LaunchConfiguration("robot_name")

    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                move_group_params,
                {"use_sim_time": False},
            ],
            namespace=robot_name,
        )
    )

    ld.add_action(
        RVizMixin.node_rviz(
            rviz_cfg_pkg="lbr_dual_arm_moveit_config",
            rviz_cfg="config/moveit.rviz",
            parameters=LBRMoveGroupMixin.params_rviz(moveit_configs=moveit_configs)
            + [{"use_sim_time": False}],
            remappings=[
                (
                    "display_planned_path",
                    PathJoinSubstitution([robot_name, "display_planned_path"]),
                ),
                ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
                (
                    "monitored_planning_scene",
                    PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
                ),
                ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
                (
                    "planning_scene_world",
                    PathJoinSubstitution([robot_name, "planning_scene_world"]),
                ),
                (
                    "robot_description",
                    PathJoinSubstitution([robot_name, "robot_description"]),
                ),
                (
                    "robot_description_semantic",
                    PathJoinSubstitution([robot_name, "robot_description_semantic"]),
                ),
                (
                    "recognized_object_array",
                    PathJoinSubstitution([robot_name, "recognized_object_array"]),
                ),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="mode",
            default_value="mock",
            description="Dual-arm description mode.",
            choices=["mock", "hardware"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm",
            description="Namespace for MoveIt and RViz remappings.",
        )
    )
    ld.add_action(RVizMixin.arg_rviz(default_value="true"))
    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())
    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
