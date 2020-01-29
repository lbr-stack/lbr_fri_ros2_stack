import os
import sys
import argparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def get_args():
    parser = argparse.ArgumentParser(description='Argument parser for ROS2 package lbr_gazebo.')
    parser.add_argument('-m', '--model', type=str, default='med7', help='Available models are iiwa7, iiwa14, med7, and med 14.')
    
    return parser.parse_args(sys.argv[4:])

def generate_launch_description():
    args = get_args()
    model = args.model

    urdf_file_name = model + '.urdf'
    urdf = os.path.join(get_package_share_directory('lbr_description'), 'urdf', urdf_file_name)

    return LaunchDescription([
        Node(
            package='lbr_fri_ros',
            node_executable='lbr_fri_ros'
        ),
        
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            arguments=[urdf])
    ])
