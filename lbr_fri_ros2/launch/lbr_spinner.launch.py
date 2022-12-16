from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    lbr_spinner_node = Node(
        package="lbr_fri_ros2",
        executable="lbr_spinner",
        emulate_tty=True,
        output="screen",
    )

    lbr_state_smoothing_node = Node(
        package="lbr_fri_ros2",
        executable="lbr_state_smoothing_node.py",
        output="screen",
    )

    lbr_state_smoothing_node_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=lbr_spinner_node, on_start=[lbr_state_smoothing_node]
        )
    )

    return LaunchDescription([lbr_spinner_node, lbr_state_smoothing_node_event_handler])
