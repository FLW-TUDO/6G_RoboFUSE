#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

class JsonPrinterNode(Node):
    def __init__(self):
        super().__init__('json_printer_node')
        self.subscription1 = self.create_subscription(TransformStamped, '/ep03/vicon/pose', self.callback1, 10)
        self.subscription2 = self.create_subscription(TransformStamped, '/ep04/vicon/pose', self.callback2, 10)
        #self.subscription1 = self.create_subscription(TransformStamped, '/station2/vicon/pose', self.callback1, 10)
        #self.subscription2 = self.create_subscription(TransformStamped, '/station4/vicon/pose', self.callback2, 10)
        self.translation_topic1 = None
        self.translation_topic2 = None
        self.publisher = self.create_publisher(Float32, '/relative_distance', 10)
        
        # Create a timer that triggers every second
        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback1(self, msg):
        self.translation_topic1 = msg.transform.translation

    def callback2(self, msg):
        self.translation_topic2 = msg.transform.translation

    def timer_callback(self):
        if self.translation_topic1 is not None and self.translation_topic2 is not None:
            self.calculate_and_publish_distance()

    def calculate_and_publish_distance(self):
        distance = self.calculate_distance(self.translation_topic1, self.translation_topic2)
        self.publish_distance(distance)

    def publish_distance(self, distance):
        msg = Float32()
        msg.data = distance
        self.publisher.publish(msg)

    def calculate_distance(self, translation1, translation2):
        dx = translation1.x - translation2.x
        dy = translation1.y - translation2.y
        dz = translation1.z - translation2.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance

def main(args=None):
    rclpy.init(args=args)
    node = JsonPrinterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#ros2 launch vicon_bridge_ros2 vicon_bridge.launch.py
#python3 Working_topics.py
