#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import tf_transformations
from tf2_ros import Buffer, TransformListener


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Subscribe to the master's pose 
        self.leader_pose_sub = self.create_subscription(
            PoseStamped,
            '/ep05/dummy/pose',
            self.leader_pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /ep05/dummy/pose topic.")  # Log subscription info

        # Subscribe to the /relative_distance topic to get the distance between robots
        self.distance_sub = self.create_subscription(
            # Float32,  # Assuming the message is a float, Twist can hold linear distance info
            # '/relative_distance',
            # self.distance_callback,
            String,
            '/serial_raw_data',
            self.catch_value_callback,
            10
        )
        self.get_logger().info("Subscribed to /relative_distance topic.")  # Log subscription info

        # Action client for follower
        self.follower_client = ActionClient(self, NavigateToPose, '/ep03/navigate_to_pose')
        self.get_logger().info("Action client for /ep03/navigate_to_pose initialized.")

        # Set up tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.leader_pose = None
        self.distance = 0.60  # Set the threshold distance (60 cm)
        self.current_distance = float('inf')  # Initialize the current distance to a large value
    
    def leader_pose_callback(self, msg):

        self.leader_pose = msg.pose
        # Send waypoints if distance is greater than 60 cm
        if self.current_distance > self.distance:
            # self.get_logger().info(f"la distancia ahora es:{self.current_distance}")
            self.send_waypoints()
        else:
            self.stop_follower()

    def distance_callback(self, msg):
        
        self.current_distance = msg.data 
        self.get_logger().info(f"Received relative distance: {self.current_distance} meters")

        # Check if the distance is less than 60 cm
        if self.current_distance < self.distance:
            self.get_logger().info("Distance is below 60 cm. Stopping the robot.")
            self.stop_follower()
    
    def catch_value_callback(self, msg):
        
        self.current_value = msg.data 
        self.get_logger().info(f"Received message: {self.current_value} ")

        # Check if the distance is less than 60 cm
        if self.current_value == '0':
            self.get_logger().info("Distance is below 60 cm. Stopping the robot.")
            self.stop_follower()

    def stop_follower(self):
        # Stop sending waypoints and publish something 

        self.get_logger().info("Stopping the follower robot.")
        
        # here the action when the trheshold is broken 

    def compute_follower_pose(self, leader_pose):
        try:
            # Lookup transform from 'map' to 'base_link' (or base_footprint) of ep05
            transform = self.tf_buffer.lookup_transform('map', 'ep05/base_footprint', rclpy.time.Time())

            # Get the orientation in Euler angles
            euler = tf_transformations.euler_from_quaternion([
                leader_pose.orientation.x,
                leader_pose.orientation.y,
                leader_pose.orientation.z,
                leader_pose.orientation.w
            ])
            
            yaw = euler[2]  # Extract yaw (rotation around z-axis)

            # Apply a 10 cm offset in the local x-direction of the leader robot
            follower_pose = PoseStamped()
            follower_pose.pose.position.x = leader_pose.position.x - self.distance * math.cos(yaw)
            follower_pose.pose.position.y = leader_pose.position.y - self.distance * math.sin(yaw)
            follower_pose.pose.position.z = leader_pose.position.z  # Assuming z stays the same

            # Keep the same orientation as the leader
            follower_pose.pose.orientation = leader_pose.orientation
            
            return follower_pose
        
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            return None

    def send_waypoints(self):
        if self.leader_pose is None:
            self.get_logger().warn("Leader pose is not available yet.")
            return
        
        pose_cal = self.compute_follower_pose(self.leader_pose)
        if pose_cal is None:
            return
        
        follower_goal = NavigateToPose.Goal()
        follower_goal.pose = pose_cal
        follower_goal.pose.header.frame_id = 'map'
        
        self.follower_client.send_goal_async(follower_goal)
        # self.get_logger().info(f"Sending follower robot to Position (x: {follower_goal.pose.pose.position.x}, y: {follower_goal.pose.pose.position.y})")

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

