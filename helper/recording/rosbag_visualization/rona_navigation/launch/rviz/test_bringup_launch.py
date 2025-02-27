import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

def generate_launch_description():
    bringup_share = get_package_share_directory('rona_navigation')

    # Declare speed argument as numerical input
    declare_speed_cmd = DeclareLaunchArgument(
        'speed',
        default_value='2',  # Default to 'normal'
        description='Set navigation speed: 1 (fast), 2 (normal), 3 (slow)'
    )

    speed = LaunchConfiguration('speed')

    # Dynamically select the params file based on the speed input
    params_file_text = PythonExpression([
        "'",
        os.path.join(bringup_share, 'config', 'robomaster_nav2_'),
        "' + ('fast' if int(", speed, ") == 1 else 'normal' if int(", speed, ") == 2 else 'slow') + '.yaml'"
    ])

    # Declare the prefix argument
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value=socket.gethostname(),
        description='Namespace prefix'
    )

    prefix = LaunchConfiguration('prefix')

    # Other configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the rest of the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=prefix,
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_text,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            bringup_share, 'config', 'navigate_w_recovery.xml'
        ),
        description='Full path to the behavior tree XML file to use'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        description='Whether to run a SLAM (uses AMCL instead if false)'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'map.yaml'),
        description='Full path to map YAML file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start up the nav2 stack'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_share, 'config', 'rviz', 'nav2_robomaster_view.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ'
    )

    # Include RVIZ if enabled
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': prefix,
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items()
    )
    
    robocore_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robomaster_bringup'), 'launch', 'minimal.launch.py'))
    )

    vicon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rona_physical'), 'launch', 'vicon_tf_converter.launch.py')),
        launch_arguments={
                'namespace': prefix}.items()
    )

    namespaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': [prefix, '/']}
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=prefix),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link1_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '/map', [namespace, '/odom']],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rviz', 'navigation_launch.py')),
            launch_arguments={
                'namespace': prefix,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': namespaced_params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true'}.items()
        )
    ])

    # Launch description
    ld = LaunchDescription()

    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_speed_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robocore_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(vicon_cmd)

    return ld
