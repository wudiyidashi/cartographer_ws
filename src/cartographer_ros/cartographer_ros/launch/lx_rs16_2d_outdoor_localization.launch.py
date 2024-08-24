"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""
import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown


def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filename',default_value=os.path.join(os.path.expandvars('$HOME'),"bagfiles/rslidar-outdoor-gps-notf.db3"))
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='true')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/lx_2d.rviz')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files')
    configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames', default_value = 'lx_rs16_2d_outdoor_localization.lua')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')
    load_state_filename_arg = DeclareLaunchArgument('load_state_filename',default_value = os.path.join(os.path.expandvars('$HOME'),"carto_ws/map/2d-1.pbstream"))
       ## ***** Nodes *****
    cartographer_node = Node(
    package = 'cartographer_ros',
    executable = 'cartographer_node',
    parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    arguments = [
        '-configuration_directory', LaunchConfiguration('configuration_directory'),
        '-configuration_basename', LaunchConfiguration('configuration_basenames'),
        '-load_state_filename', LaunchConfiguration('load_state_filename'),
        ],
    remappings = [
        ('points2', 'rslidar_points'),
        ('imu', 'imu'),
        ('scan', 'front_scan'),
        ('odom', 'odom_scout'),
       ],
    output = 'screen'
    )

    cartographer_occupancy_grid_node = Node(
    package = 'cartographer_ros',
    executable = 'cartographer_occupancy_grid_node',
    parameters = [
        {'use_sim_time': True},
        {'resolution': 0.05}],
    )
    
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d',  LaunchConfiguration('rviz_config'),],
        parameters = [{'use_sim_time': True}],
    )

    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
        name = 'rosbag_play',
    )

    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        use_sim_time_arg,
        load_state_filename_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,

        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        # rviz_node,
        ros2_bag_play_cmd
    ])

    
