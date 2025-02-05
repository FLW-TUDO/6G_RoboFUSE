import os
from datetime import datetime

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import sys


def generate_launch_description():
    
    server_ip_arg = DeclareLaunchArgument('server_ip', default_value='127.0.0.1')
    server_port_arg = DeclareLaunchArgument('server_port', default_value='65432')
    
    
    command_port_arg = DeclareLaunchArgument('command_port', default_value='/dev/ttyUSB1')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200')

    data_port_arg = DeclareLaunchArgument('data_port', default_value='/dev/ttyUSB2')
    data_rate_arg = DeclareLaunchArgument('data_rate', default_value='921600')
    
    # validate_ports(LaunchConfiguration('command_port'),LaunchConfiguration('data_port'))
    # Generate unique directory for rosbag files based on current timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Save bags in the current working directory.
    # rosbag_dir = os.getcwd()  # Get the current working directory
    # Ensure rosbag_logs directory exists
    rosbag_dir = os.path.join(os.getcwd(), 'rosbag_logs')
    os.makedirs(rosbag_dir, exist_ok=True)
    
    # Create a unique filename for the rosbag file.
    rosbag_file_path = os.path.join(rosbag_dir, f'rosbag_logs/recorded_data_{timestamp}')  # Full path for recorded data

    # Configuration file for mmWave sensor
    cfg_file = "6843ISK_3d_0512.cfg"

    # Path to configuration file
    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    cfg_file_path = os.path.join(pkg_dir_path, 'cfg', cfg_file)

    # Node for configuring the mmWave sensor
    mmwave_quick_config = Node(
        package='ti_mmwave_ros2_pkg',
        executable='mmWaveQuickConfig',
        name='mmwave_quick_config',
        output='screen',
        arguments=[cfg_file_path],
        parameters=[{
            "mmWaveCLI_name": "/mmWaveCLI",
        }],
    )

    # Node for managing communication with the mmWave sensor
    mmwave_comm_srv_node = Node(
        package='ti_mmwave_ros2_pkg',
        executable='mmwave_comm_srv_node',
        name='mmWaveCommSrvNode',
        output='screen',
        parameters=[{
            "command_port": LaunchConfiguration('command_port'),
            "command_rate": LaunchConfiguration('baud_rate'),
            "mmWaveCLI_name": "/mmWaveCLI",
        }],
    )

    # Command to start recording rosbag
    rosbag_record_cmd = [
        'ros2', 'bag', 'record', '-o', rosbag_file_path, 
        '--all'  # Use '--all' to record all topics
    ]

    # Execute process for rosbag recording.
    rosbag_record_process = ExecuteProcess(
        cmd=rosbag_record_cmd,
        output='screen'
    )

    # Container with the updated mmWaveDataHdl node
    container = ComposableNodeContainer(
        name='my_container',
        namespace='ep03',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ti_mmwave_ros2_pkg',
                plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                name='mmWaveDataHdl',
                parameters=[{
                    "data_port": LaunchConfiguration('data_port'),
                    "data_rate": LaunchConfiguration('data_rate'),
                    "frame_id": "ep03/wave_sensor_link",
                    "max_allowed_elevation_angle_deg": 90,
                    "max_allowed_azimuth_angle_deg": 90,
                    "server_ip": LaunchConfiguration('server_ip'),  
                    "server_port": LaunchConfiguration('server_port'),      
                    "enable_socket": True      
                }]
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        command_port_arg,
        baud_rate_arg,
        data_port_arg,
        data_rate_arg,
        mmwave_comm_srv_node,
        mmwave_quick_config,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=mmwave_quick_config,
                on_exit=[container,rosbag_record_process],  
            )
        ),
         
    ])
