from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_mixins.lbr_bringup import LBRMoveGroupMixin
from launch_mixins.lbr_description import LBRDescriptionMixin
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    model = LaunchConfiguration("model").perform(context)

    # generate moveit configs
    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )

    # launch demo node
    ld.add_action(
        Node(
            package="lbr_moveit_cpp",
            executable="hello_moveit",
            parameters=[
                moveit_configs.to_dict(),
                {"use_sim_time": LaunchConfiguration("sim")},
                LBRDescriptionMixin.param_robot_name(),
            ],
        )
    )
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRDescriptionMixin.arg_sim())

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
