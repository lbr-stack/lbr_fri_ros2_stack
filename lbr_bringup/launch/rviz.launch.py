from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.rviz import RVizMixin


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()
    mode = LaunchConfiguration("mode").perform(context)

    # nodes
    ld.add_action(
        RVizMixin.node_rviz(
            rviz_config_pkg="lbr_bringup",
            rviz_config=f"config/{mode}.rviz",
        )
    )
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_mode())

    # rviz with mode-specific configuration
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
