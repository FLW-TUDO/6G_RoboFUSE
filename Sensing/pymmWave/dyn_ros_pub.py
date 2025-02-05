import json
import re
import time
import os
import glob
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

log_dir = '/home/robot_3/6G_RoboFUSE/Sensing/log/'

# Filename pattern (prefix) to match the log files
file_pattern = 'test_robomaster_5_mmwave_sensor_7_cfar*.txt'

# Find the latest file based on the pattern
def find_latest_file(directory, pattern):
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError(f"No files found with pattern {pattern}")
    
    latest_file = max(files, key=os.path.getmtime)
    return latest_file

# Tail the file and read new lines dynamically
def tail_file(file, read_from_start=True):
    """ Continuously read new lines as they are added to the file.
        If `read_from_start` is True, start reading from the beginning of the file.
    """
    #reading and parsing data
    with open(file, 'r') as f:
        if not read_from_start:
            f.seek(0, os.SEEK_END)  # Move the pointer to the end of the file

        while True:
            line = f.readline()
            if not line:  # No new line yet, wait a bit and try again
                time.sleep(0.1)
                continue
            yield line.strip()

#ROS2 publiser deifinition
class MmwavePublisher(Node):
    def __init__(self):
        super().__init__('mmwave_pub')
        qos_profile = QoSProfile(depth=10)  # adjust depth if needed
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', qos_profile)

    def publish_data(self, x, y, z):
        #PointCloud2 message header
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()  # Timestamp
        header.frame_id = 'base_link'

        #binary string of points
        pc_data = struct.pack('fff', x, y, z)

        #PointCloud2 message that will be published
        point_cloud = PointCloud2()
        point_cloud.header = header
        point_cloud.height = 1
        point_cloud.width = 1
        point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12  # 4 bytes per float * 3 fields
        point_cloud.row_step = point_cloud.point_step * point_cloud.width
        point_cloud.is_dense = True
        point_cloud.data = pc_data

        # Debugging: Log the point cloud data being published
        self.get_logger().info(f'Publishing PointCloud2: x={x}, y={y}, z={z}, raw={pc_data}')

        self.publisher_.publish(point_cloud)

#ROS2 node and publisher init
rclpy.init(args=None)
mmwave_publisher = MmwavePublisher()

# Debugging: Find the latest file
input_file = find_latest_file(log_dir, file_pattern)
print(f"Using latest file: {input_file}")

# Test with a pre-existing file (debugging)
print(f"Reading from the pre-existing file: {input_file}")

# Read from the file starting at the beginning
try:
    for line in tail_file(input_file, read_from_start=True):  # Change here
        # Debugging: Print every line being read
        print(f"New line from file: {line}")

        try:
            # Parse the JSON data from each new line
            data = json.loads(line)

            # Debugging: Print parsed data to ensure it's valid
            print(f"Parsed JSON data: {data}")

            numObj = data.get('numObj', 0)

            # Publish each object's data as it's received
            for i in range(numObj):
                x = data['x'][i]
                y = data['y'][i]
                z = data['z'][i]

                # Debugging: Print coordinates before publishing
                print(f"Publishing data - x: {x}, y: {y}, z: {z}")

                # Publish point cloud data for this point
                mmwave_publisher.publish_data(x, y, z)
                
                # Delay between publishes (can adjust or remove for faster publish)
                time.sleep(0.3)

        except json.JSONDecodeError:
            mmwave_publisher.get_logger().warn(f"Failed to decode JSON from log line: {line}")
        except KeyError as e:
            mmwave_publisher.get_logger().warn(f"Missing key in JSON data: {e}")
        except IndexError as e:
            mmwave_publisher.get_logger().warn(f"Index error while accessing object data: {e}")

except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()
