# Copyright 2020-2022, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    
    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    lat_control_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lateral_controller.param.yaml')
    lon_control_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/longitudinal_controller.param.yaml')
    latlon_muxer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/latlon_muxer.param.yaml')

    

    # Arguments

    
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    lat_control_param = DeclareLaunchArgument(
        'lat_control_param_file',
        default_value=lat_control_param_file,
        description='Path to config file for lateral controller'
    )
    lon_control_param = DeclareLaunchArgument(
        'lon_control_param_file',
        default_value=lon_control_param_file,
        description='Path to config file for longitudinal controller'
    )
    latlon_muxer_param = DeclareLaunchArgument(
        'latlon_muxer_param_file',
        default_value=latlon_muxer_param_file,
        description='Path to config file for lateral and longitudinal control commands muxer'
    )

    # Nodes

    
    lat_control = Node(
        package='trajectory_follower_nodes',
        executable='lateral_controller_node_exe',
        name='lateral_controller_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('lat_control_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
           ("input/reference_trajectory", "/planning/trajectory"),#/planning/scenario_planning/lane_driving/trajectory
           ("input/current_kinematic_state", "/vehicle/vehicle_kinematic_state"),
           ("input/tf", "/tf"),
           ("input/tf_static", "/tf_static")
        ],
    )
    lon_control = Node(
        package='trajectory_follower_nodes',
        executable='longitudinal_controller_node_exe',
        name='longitudinal_controller_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('lon_control_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
           ("input/current_trajectory", "/planning/trajectory"),#/planning/scenario_planning/lane_driving/trajectory
           ("input/current_state", "/vehicle/vehicle_kinematic_state"),
           ("input/tf", "/tf"),
           ("input/tf_static", "/tf_static"),
        ],
    )
    latlon_muxer = Node(
        package='trajectory_follower_nodes',
        executable='latlon_muxer_node_exe',
        name='latlon_muxer_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('latlon_muxer_param_file'),
        ],
        remappings=[
           ("input/lateral/control_cmd", "output/lateral/control_cmd"),
           ("input/longitudinal/control_cmd", "output/longitudinal/control_cmd"),
           ("output/control_cmd", "/vehicle/ackermann_vehicle_command"),
        ],
    )

    

    return LaunchDescription([
        vehicle_characteristics_param,
        lat_control_param,
        lon_control_param,
        latlon_muxer_param,
        lat_control,
        lon_control,
        latlon_muxer
    ])
