import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import socket
from time import sleep

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circular_trajectory_with_nav_goal')
        self.namespace = socket.gethostname()

        # Declare and get parameters
        self.declare_parameter('radius', 1.0)  
        self.declare_parameter('angular_velocity', 0.8)  
        self.declare_parameter('initial_x', 4.0)  
        self.declare_parameter('initial_y', 1.0)  
        self.declare_parameter('duration', 60.0)  
        
        self.radius = self.get_parameter('radius').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.duration = self.get_parameter('duration').value

        # Calculate linear velocity based on radius and angular velocity
        self.linear_velocity = self.angular_velocity * self.radius

        self.get_logger().info(f'Radius: {self.radius}, Angular Velocity: {self.angular_velocity}')
        self.get_logger().info(f'Calculated Linear Velocity: {self.linear_velocity}')

        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, self.namespace + '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, self.namespace + '/cmd_vel', 10)

        # State management
        self.initial_goal_sent = False
        self.start_circular_motion = False
        self.start_time = None

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def send_initial_goal(self):
        """Send the initial navigation goal."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = self.initial_x
        goal_msg.pose.position.y = self.initial_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No rotation
        
        self.get_logger().info(f'Sending initial goal to ({self.initial_x}, {self.initial_y})...')
        self.goal_publisher.publish(goal_msg)
        self.initial_goal_sent = True

    def start_circular_trajectory(self):
        """Start the circular trajectory motion."""
        self.get_logger().info('Starting circular trajectory...')
        self.start_circular_motion = True
        self.start_time = self.get_clock().now()

    def control_loop(self):
        """Control loop to manage initial positioning and circular motion."""
        if not self.initial_goal_sent:
            # Send the initial navigation goal
            self.send_initial_goal()
            sleep(5)  # Allow some time for the navigation stack to process

        elif self.initial_goal_sent and not self.start_circular_motion:
            # Assume the robot reached the goal (or implement proper feedback logic)
            self.start_circular_trajectory()

        elif self.start_circular_motion:
            # Publish cmd_vel messages for circular trajectory
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed_time < self.duration:
                twist_msg = Twist()
                twist_msg.linear.x = self.linear_velocity
                twist_msg.angular.z = self.angular_velocity
                self.cmd_vel_publisher.publish(twist_msg)
            else:
                # Stop the robot after completing the circular motion
                twist_msg = Twist()
                self.cmd_vel_publisher.publish(twist_msg)  # Stop robot
                self.get_logger().info('Completed circular trajectory. Shutting down.')
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
