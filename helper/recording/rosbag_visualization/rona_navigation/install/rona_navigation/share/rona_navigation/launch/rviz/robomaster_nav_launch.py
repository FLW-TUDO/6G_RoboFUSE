# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Get the launch directory
    bringup_share = get_package_share_directory('rona_navigation')

    prefix = 'ep04'

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=prefix,
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_share, 'config', 'robomaster_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        # default_value=os.path.join(
        #     get_package_share_directory('nav2_bt_navigator'),
        #     'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        default_value=os.path.join(bringup_share, 'config', 'navigate_w_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        description='Whether run a SLAM (uses AMCL instead if false)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'map.yaml'),

        description='Full path to map yaml file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_share, 'config', 'rviz', 'nav2_robomaster_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')


    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch','rviz','rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': prefix,
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    namespaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': str(prefix + '/')})

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            namespace=prefix),
        Node(
            parameters=[
                namespaced_params_file,
                {"scan_topic" : "/" + prefix + "/scan"},
                {"odom_frame" : prefix + "/odom"},
                {"map_frame" : "map"},
                {"base_frame" : prefix + "/base_footprint"},
            ],
            package='slam_toolbox',
            executable='lifelong_slam_toolbox_node',
            name='Slam_Toolbox_Mapping',
            output='screen',		
            # remappings=[("/map", str("/" + prefix + "/map")),
            #             ("/map_metadata", str("/" + prefix + "/map_metadata"))]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch','rviz','amcl_launch.py')),

            # condition=IfCondition(PythonExpression(['not ', use_slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': namespaced_params_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz','navigation_launch.py')),
            launch_arguments={'namespace': prefix,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': namespaced_params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items())
     ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)
    
    ld.add_action(bringup_cmd_group)

    return ld
