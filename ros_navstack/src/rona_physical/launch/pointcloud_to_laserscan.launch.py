from launch import LaunchDescription
from launch_ros.actions import Node
from math import pi 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {
                    'target_frame': 'ep05/wave_sensor_link',  # Frame ID of your point cloud data
                    'transform_tolerance': 0.01,
                    'min_height': -0.2,
                    'max_height': 1.0,
                    'angle_min': pi/2 - 1.0472,  # -60 degrees in radians
                    'angle_max': pi/2 + 1.0472,   # 60 degrees in radians
                    'angle_increment': 0.05,
                    'scan_time': 0.05,
                    'range_min': 0.2,
                    'range_max': 9.04,
                    'use_inf': True,
                    'inf_epsilon': 0.5,
                }
            ],
            remappings=[
                ('cloud_in', 'ti_mmwave/radar_scan_pcl'),  # Input topic for PointCloud2 data
                ('scan', 'ep05/scan')              # Output topic for LaserScan data
            ]
        )
    ])
