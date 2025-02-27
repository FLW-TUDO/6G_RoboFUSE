#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import numpy as np

class CostmapGenerator(Node):
    def __init__(self):
        super().__init__('costmap_generator')

        # Get namespace for dynamic topic names
        namespace = self.get_namespace().strip("/")
        ns_prefix = f"/{namespace}" if namespace else ""

        # Get costmap topic from navigation system
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.costmap_topic = f"{ns_prefix}{self.costmap_topic}"

        # Subscribe to costmap to get map dimensions
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)

        # Subscribe to all Vicon obstacle topics
        self.vicon_topics = [
            f"{ns_prefix}/AS_1_neu/vicon/pose",
            f"{ns_prefix}/AS_2_neu/vicon/pose",
            f"{ns_prefix}/AS_3_neu/vicon/pose",
            f"{ns_prefix}/AS_4_neu/vicon/pose",
            f"{ns_prefix}/AS_5_neu/vicon/pose",
            f"{ns_prefix}/AS_6_neu/vicon/pose"
        ]

        self.subscribers = [
            self.create_subscription(
                TransformStamped, topic, self.obstacle_callback, 10
            ) for topic in self.vicon_topics
        ]

        # Costmap publisher
        self.publisher = self.create_publisher(OccupancyGrid, f"{ns_prefix}/updated_costmap", 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize empty costmap until map data is received
        self.costmap = None
        self.grid_width = 0
        self.grid_height = 0
        self.resolution = 0.05  # Default, will update from costmap

    def costmap_callback(self, msg):
        """Update costmap dimensions from the navigation system."""
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height
        self.resolution = msg.info.resolution
        self.costmap_frame = msg.header.frame_id

        # Initialize costmap with zeros
        self.costmap = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

    def obstacle_callback(self, msg):
        """Handles Vicon data and updates the costmap with obstacles."""
        if self.costmap is None:
            self.get_logger().warn("Waiting for costmap data...")
            return

        try:
            # Transform obstacle position to costmap frame
            transform = self.tf_buffer.lookup_transform(
                self.costmap_frame, msg.header.frame_id, rclpy.time.Time()
            )

            x = msg.transform.translation.x + transform.transform.translation.x
            y = msg.transform.translation.y + transform.transform.translation.y

            # Convert to grid coordinates
            grid_x = int(x / self.resolution)
            grid_y = int(y / self.resolution)

            # Mark obstacle in costmap
            self.mark_obstacle(grid_x, grid_y)

            # Publish updated costmap
            self.publish_costmap()

        except tf2_ros.LookupException:
            self.get_logger().warn("TF lookup failed")

    def mark_obstacle(self, grid_x, grid_y):
        """Marks a 1.3m x 0.9m rectangle in the costmap."""
        rect_width = int(1.3 / self.resolution)
        rect_height = int(0.9 / self.resolution)

        for i in range(-rect_width // 2, rect_width // 2):
            for j in range(-rect_height // 2, rect_height // 2):
                x, y = grid_x + i, grid_y + j
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    self.costmap[y, x] = 100  # Mark as occupied

    def publish_costmap(self):
        """Publishes the updated costmap as an OccupancyGrid message."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.costmap_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0

        # Convert numpy array to list for ROS
        msg.data = self.costmap.flatten().tolist()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
