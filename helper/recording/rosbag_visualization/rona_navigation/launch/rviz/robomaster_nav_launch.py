import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

def launch_setup(context, *args, **kwargs):
    # ✅ Extract evaluated namespace value
    namespace = LaunchConfiguration('namespace').perform(context)

    bringup_share = get_package_share_directory('rona_navigation')

    # ✅ Ensure topic names are properly formatted
    odom = f"{namespace}/odom"
    base_footprint = f"{namespace}/base_footprint"

    tf_remapping = [('tf', f"{namespace}/tf"), ('tf_static', f"{namespace}/tf_static")]

    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        
        # Static TF publisher (Map -> Odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link1_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '/map', odom],
            output='screen',
            remappings=tf_remapping
        ),

        # Static TF publisher (Odom -> Base Footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link1_broadcaster_ba',
            arguments=['0', '0', '0', '0', '0', '0', odom, base_footprint],
            output='screen',
            remappings=tf_remapping
        ),

        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz', 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'params_file': ReplaceString(
                    source_file=LaunchConfiguration('params_file'),
                    replacements={'<robot_namespace>': f"{namespace}/"}
                ),
                'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true'
            }.items()
        )
    ])

    return [bringup_cmd_group]

def generate_launch_description():
    bringup_share = get_package_share_directory('rona_navigation')

    # ✅ Declare namespace argument
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='ep03'),  # Default namespace
        description='Namespace for the robot (e.g., ep03, ep05)'
    )

    # ✅ Declare other necessary launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_share, 'config', 'robomaster_nav2_params.yaml'),
        description='Path to the ROS2 parameters file'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(bringup_share, 'config', 'navigate_w_recovery.xml'),
        description='Behavior Tree XML file'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam', default_value='False', description='Run SLAM instead of AMCL'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'map.yaml'),
        description='Path to map YAML file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically start Nav2 stack'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_share, 'config', 'rviz', 'nav2_robomaster_view.rviz'),
        description='Path to RViz configuration file'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Start RViz'
    )

    # ✅ Define other included launch files
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz', 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': 'True',
            'rviz_config': LaunchConfiguration('rviz_config_file')
        }.items()
    )

    vicon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rona_physical'), 'launch', 'vicon_tf_converter.launch.py')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    robo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_robomaster_description'), 'launch', 'ros_robomaster_description.launch.py'))
    )

    # ✅ Add everything to the launch description
    ld = LaunchDescription()

    # ✅ Add declared launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # ✅ Add the main launch commands
    ld.add_action(vicon_cmd)
    ld.add_action(robo_description)
    ld.add_action(rviz_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))  # Ensures correct evaluation

    return ld

# import os
# import socket

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node, PushRosNamespace
# from nav2_common.launch import ReplaceString

# def generate_launch_description():
#     bringup_share = get_package_share_directory('rona_navigation')
#     ns= 'ep03'
#     # nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

#     # Get the default hostname of the computer

#     # New launch argument for prefix
#     declare_prefix_cmd = DeclareLaunchArgument(
#         'prefix',
#         default_value= ns,
#         description='Namespace prefix')

#     prefix = LaunchConfiguration('prefix')

#     namespace = LaunchConfiguration('namespace')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     params_file = LaunchConfiguration('params_file')
#     default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
#     map_yaml_file = LaunchConfiguration('map')
#     use_slam = LaunchConfiguration('use_slam')
#     autostart = LaunchConfiguration('autostart')
#     rviz_config_file = LaunchConfiguration('rviz_config_file')
#     use_rviz = LaunchConfiguration('use_rviz')

#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace',
#         default_value=prefix,
#         description='Top-level namespace')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation (Gazebo) clock if true')

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(bringup_share, 'config', 'robomaster_nav2_params.yaml'),# write down nav2_params for a slow velocity 
#         description='Full path to the ROS2 parameters file to use for all launched nodes')

#     declare_bt_xml_cmd = DeclareLaunchArgument(
#         'default_bt_xml_filename',
#         default_value=os.path.join(
#             bringup_share, 'config', 'navigate_w_recovery.xml'
#         ),
#         description='Full path to the behavior tree xml file to use')
    
#     declare_slam_cmd = DeclareLaunchArgument(
#         'use_slam',
#         default_value='False',
#         description='Whether run a SLAM (uses AMCL instead if false)')

#     declare_map_yaml_cmd = DeclareLaunchArgument(
#         'map',
#         default_value=os.path.join(bringup_share, 'maps', 'map.yaml'),
#         description='Full path to map yaml file to load')

#     declare_autostart_cmd = DeclareLaunchArgument(
#         'autostart',
#         default_value='true',
#         description='Automatically startup the nav2 stack')

#     declare_rviz_config_file_cmd = DeclareLaunchArgument(
#         'rviz_config_file',
#         default_value=os.path.join(bringup_share, 'config', 'rviz', 'nav2_robomaster_view.rviz'),
#         description='Full path to the RVIZ config file to use')

#     declare_use_rviz_cmd = DeclareLaunchArgument(
#         'use_rviz',
#         default_value='False',
#         description='Whether to start RVIZ')

#     map_dir = LaunchConfiguration(
#         'map',
#         default=os.path.join(get_package_share_directory('rona_navigation'), 'maps', 'map.yaml'))

#     rviz_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch','rviz','rviz_launch.py')),
#         condition=IfCondition(use_rviz),
#         launch_arguments={'namespace': prefix,
#                           'use_namespace': 'True',
#                           'rviz_config': rviz_config_file}.items())
    
# #    robocore_cmd = IncludeLaunchDescription(
# #        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robomaster_bringup'), 'launch','minimal.launch.py')))
#     vicon_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rona_physical'), 'launch','vicon_tf_converter.launch.py')),
#         launch_arguments={
#                 'namespace': prefix}.items())

#     robo_description = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_robomaster_description'), 'launch','ros_robomaster_description.launch.py'))
#     )

#     namespaced_params_file = ReplaceString(
#         source_file=params_file,
#         replacements={'<robot_namespace>': [prefix, '/']})

#     tf_remapping = [('tf', [namespace, '/tf']), ('tf_static', [namespace, '/tf_static'])]
#     print(f"namespace: {namespace}")

#     odom = ns + '/odom'
#     base_footprint = ns + '/base_footprint'

#     bringup_cmd_group = GroupAction([
#         PushRosNamespace(namespace=prefix),
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='link1_broadcaster',
#             arguments=['0', '0', '0', '0', '0', '0','/map', [namespace, '/odom']],
#             output='screen',
#             remappings= tf_remapping),
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='link1_broadcaster_ba',
#             arguments=['0', '0', '0', '0', '0', '0', odom, base_footprint], #change this name
#             #arguments=['0', '0', '0', '0', '0', '0','ep03/odom', 'ep03/base_footprint'], #change this name
#             output='screen',
#             remappings= tf_remapping),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz','navigation_launch.py')),
#             launch_arguments={
#                 'namespace': prefix,
#                 'use_sim_time': use_sim_time,
#                 'autostart': autostart,
#                 'params_file': namespaced_params_file,
#                 'default_bt_xml_filename': default_bt_xml_filename,
#                 'use_lifecycle_mgr': 'false',
#                 'map_subscribe_transient_local': 'true'}.items())
#      ])

#     ld = LaunchDescription()

#     # Add the new prefix launch argument
#    # ld.add_action(robo_description)
#     ld.add_action(declare_prefix_cmd)
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_slam_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(vicon_cmd)
#     ld.add_action(robo_description)
#  #   ld.add_action(robocore_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(declare_bt_xml_cmd)
#     ld.add_action(declare_map_yaml_cmd)
#     ld.add_action(declare_autostart_cmd)
#     ld.add_action(declare_rviz_config_file_cmd)
#     ld.add_action(declare_use_rviz_cmd)
#     ld.add_action(rviz_cmd)
#     ld.add_action(bringup_cmd_group)

#     return ld
