#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import sys

class TimestampPublisher(Node):
    def __init__(self, namespace):
        super().__init__('timestamp_publisher')

        # Construct topic names dynamically
        pose_topic = f"/{namespace}/vicon/pose"
        marker_topic = f"/{namespace}/timestamp_marker"

        self.get_logger().info(f"Listening to: {pose_topic}")
        self.get_logger().info(f"Publishing markers to: {marker_topic}")

        # Subscriber for pose data
        self.subscription = self.create_subscription(
            TransformStamped,
            pose_topic,
            self.pose_callback,
            10)

        # Publisher for timestamp marker
        self.publisher = self.create_publisher(Marker, marker_topic, 10)

    def pose_callback(self, msg):
        # Extract timestamp
        timestamp_sec = msg.header.stamp.sec
        timestamp_nsec = msg.header.stamp.nanosec
        timestamp_str = f"{timestamp_sec}.{timestamp_nsec:09d}"

        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # Use the frame from the message
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "timestamp_marker"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Place marker near the object
        marker.pose.position.x = msg.transform.translation.x
        marker.pose.position.y = msg.transform.translation.y
        marker.pose.position.z = msg.transform.translation.z + 0.5  # Slightly above

        marker.scale.z = 0.5  # Adjust text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set text to show the timestamp
        marker.text = f"Timestamp: {timestamp_str}"

        # Publish the marker
        self.publisher.publish(marker)
        self.get_logger().info(f"[{marker.header.frame_id}] Published timestamp: {timestamp_str}")

def main(args=None):
    rclpy.init(args=args)

    # Get namespace from command-line arguments (default: "ep03")
    namespace = sys.argv[1] if len(sys.argv) > 1 else "ep03"

    node = TimestampPublisher(namespace)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
