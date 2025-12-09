from launch import LaunchDescription
from lbr_bringup.rviz import RVizMixin


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [RVizMixin.arg_rviz_cfg(), RVizMixin.arg_rviz_cfg_pkg(), RVizMixin.node_rviz()]
    )
