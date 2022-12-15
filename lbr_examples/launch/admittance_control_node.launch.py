from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    admittance_control_node = Node(
        package="lbr_examples",
        executable="admittance_control_node.py"
    )

    return LaunchDescription(
        [
            admittance_control_node
        ]
    )
