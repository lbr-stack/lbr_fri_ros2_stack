from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from lbr_bringup import LBRMoveGroupMixin
from lbr_description import LBRDescriptionMixin, RVizMixin


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())

    model = LaunchConfiguration("model").perform(context)
    moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        base_frame=LaunchConfiguration("base_frame"),
        package_name=f"{model}_moveit_config",
    )
    movegroup_params = LBRMoveGroupMixin.params_move_group()

    # MoveGroup:
    # - requires world frame
    # - maps link robot_name/base_frame -> base_frame
    # These two transform need publishing
    robot_name = LaunchConfiguration("robot_name").perform(context)
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=[0, 0, 0, 0, 0, 0],
            parent="world",
            child=LaunchConfiguration("base_frame"),
        )
    )
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=[0, 0, 0, 0, 0, 0],  # keep zero
            parent=LaunchConfiguration("base_frame"),
            child=PathJoinSubstitution(
                [
                    LaunchConfiguration("robot_name"),
                    LaunchConfiguration("base_frame"),
                ]  # results in robot_name/base_frame
            ),
        )
    )

    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                movegroup_params,
                {"use_sim_time": LaunchConfiguration("sim")},
            ],
            namespace=robot_name,
        )
    )

    # RViz
    rviz = RVizMixin.node_rviz(
        rviz_config_pkg=f"{model}_moveit_config",
        rviz_config="config/moveit.rviz",
        parameters=LBRMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        ),
        remappings=[
            ("robot_description", robot_name + "/robot_description"),
            ("robot_description_semantic", robot_name + "/robot_description_semantic"),
            ("display_planned_path", robot_name + "/display_planned_path"),
            ("monitored_planning_scene", robot_name + "/monitored_planning_scene"),
        ],
    )

    ld.add_action(rviz)

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRDescriptionMixin.arg_base_frame())
    ld.add_action(LBRDescriptionMixin.arg_sim())

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
