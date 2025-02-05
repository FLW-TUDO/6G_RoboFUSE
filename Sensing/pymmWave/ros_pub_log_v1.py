import json
import pandas as pd
import re
import numpy as np

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import struct
from std_msgs.msg import Float32MultiArray

#input_file = '\\home\\robot_5\\py_mmwave_dev\\py_mmwave_plot\\log\\test_robomaster_5_mmwave_sensor_7_cfar_20240920_182103.txt'
input_file = '/home/robot_3/6G_RoboFUSE/Sensing/log/test_robomaster_5_mmwave_sensor_7_cfar_20240927_160454.txt'

match = re.search(r'sensor_(\d{8}_\d{6})', input_file)
if match:
    timestamp_part = match.group(1)
else:
    raise ValueError("No timestamp found in the input file path")

#output_file = f'{timestamp_part}_parsed_data.csv'


data_list = []

with open(input_file, 'r') as file:
    for line in file:
        data = json.loads(line.strip())
        timestamp = data['timestamp']
        numObj = data['numObj']

        for i in range(numObj):
            row = {
                'timestamp': timestamp,
                'range': data['range'][i],
                'azimuth': data['azimuth'][i],
                'elevation': data['elevation'][i],
                'x': data['x'][i],
                'y': data['y'][i],
                'z': data['z'][i],
                'v': data['v'][i],
                'snr': data['snr'][i],
                'rangeProfile': data['rangeProfile'],
            }
            data_list.append(row)

#list of dictionaries to pandas DataFrame
df = pd.DataFrame(data_list)

print(df.x)


### Initial simple publisher (works)
# #Publisher for ROS2
# class MmwavePublisher(Node):
#     def __init__(self):
#         super().__init__('mmwave_pub')
#         qos_profile = QoSProfile(depth=10)
#         self.publisher_ = self.create_publisher(Float32MultiArray, 'detected_obj', qos_profile)

#     def publish_data(self, x, y, z, snr):
#         msg = Float32MultiArray()
#         msg.data = [x, y, z, snr]  # list of values
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published data: {msg.data}')

### Publisher and Visualizer in RViz
class MmwavePublisher(Node):
    def __init__(self):
        super().__init__('mmwave_pub')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', qos_profile)

    def publish_data(self, x, y, z, snr):
        points = np.array([[x, y, z]], dtype=np.float32)

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        pc_data = struct.pack('fff', x, y, z)

        point_cloud = PointCloud2()
        point_cloud.header = header
        point_cloud.height = 1
        point_cloud.width = 1
        point_cloud.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12  # 4 bytes per float * 3 fields
        point_cloud.row_step = point_cloud.point_step * point_cloud.width
        point_cloud.is_dense = True
        point_cloud.data = pc_data

        self.publisher_.publish(point_cloud)
        self.get_logger().info(f'Published PointCloud2: {points}')


rclpy.init(args=None)
mmwave_publisher = MmwavePublisher()

for index, row in df.iterrows():
    x = row['x']
    y = row['y']
    z = row['z']
    snr = row['snr']

    mmwave_publisher.publish_data(x, y, z, snr)
    time.sleep(0.3)

rclpy.shutdown()

