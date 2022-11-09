from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    base = get_package_share_directory('lbr_description')
    urdf_filename = base + '/urdf/med7/med7.urdf.xacro'
    
    return LaunchDescription([
        Node(
            package="safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"robot_description": ParameterValue(
                    Command(['xacro', ' ', str(urdf_filename)]), value_type=str)}
            ]
        )
    ])
