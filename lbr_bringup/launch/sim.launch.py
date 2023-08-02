from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from lbr_description import GazeboMixin, LBRDescriptionMixin, RVizMixin
from lbr_hardware_interface import LBRHardwareInterfaceMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    robot_description = LBRDescriptionMixin.description(sim=True)
    ld.add_action(GazeboMixin.include_gazebo())  # Gazebo has its own controller manager
    spawn_entity = GazeboMixin.node_spawn_entity()
    ld.add_action(spawn_entity)
    ld.add_action(
        LBRHardwareInterfaceMixin.arg_ctrl()
    )  # Gazebo loads controller configuration through lbr_description/gazebo/*.xacro from lbr_description/config/lbr_controllers.yml
    joint_state_broadcaster = LBRHardwareInterfaceMixin.node_joint_state_broadcaster()
    ld.add_action(joint_state_broadcaster)
    robot_state_publisher = LBRHardwareInterfaceMixin.node_robot_state_publisher(
        robot_description=robot_description
    )
    ld.add_action(
        robot_state_publisher
    )  # Do not condition robot state publisher on joint state broadcaster as Gazebo uses robot state publisher to retrieve robot description
    ld.add_action(LBRHardwareInterfaceMixin.node_controller())

    # RViz
    ld.add_action(RVizMixin.arg_rviz_config_pkg())
    ld.add_action(RVizMixin.arg_rviz_config())
    rviz = RVizMixin.node_rviz()
    rviz_event_handler = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[rviz])
    )
    ld.add_action(rviz_event_handler)

    return ld
