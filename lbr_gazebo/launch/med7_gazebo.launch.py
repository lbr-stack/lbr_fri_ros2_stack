#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    model_dir = os.path.join(get_package_share_directory('lbr_gazebo'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + model_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  model_dir

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'med7.world'
    world = os.path.join(get_package_share_directory('lbr_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('lbr_gazebo'), 'launch')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
    ])


