from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LBRDescriptionMixin.arg_model(),
            LBRDescriptionMixin.arg_robot_name(),
            LBRROS2ControlMixin.arg_init_jnt_pos(),
            LBRROS2ControlMixin.arg_ctrl(),  # Gazebo loads controller configuration through lbr_description/gazebo/*.xacro from lbr_description/ros2_control/lbr_controllers.yaml
            LBRROS2ControlMixin.node_robot_state_publisher(
                robot_description=LBRDescriptionMixin.param_robot_description(
                    mode="gazebo"
                ),
                use_sim_time=True,
            ),
            GazeboMixin.include_gazebo(),  # Gazebo has its own controller manager
            GazeboMixin.node_clock_bridge(),
            GazeboMixin.node_create(),  # spawns robot in Gazebo through robot_description topic of robot_state_publisher
            LBRROS2ControlMixin.node_controller_spawner(
                controllers=["joint_state_broadcaster", LaunchConfiguration("ctrl")]
            ),
        ]
    )
