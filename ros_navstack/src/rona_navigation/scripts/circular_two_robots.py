import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import math
import socket
from argparse import ArgumentParser

class DualCircularTrajectoryPublisher(Node):
    def __init__(self, linear_velocity):
        super().__init__('dual_circular_trajectory_publisher')

        # Set the linear velocity from user input
        self.linear_velocity = linear_velocity

        # Declare parameters for the first robot (ep02)
        self.namespace = socket.gethostname()
        self.radii_1 = [1.0, 2.0, 3.0]
        self.start_positions_1 = [(0, 1), (0, 2), (0, 3)]

        # Declare parameters for the second robot (ep02)
        self.namespace2 = "ep02"
        self.radii_2 = [1.0, 2.0, 3.0]
        self.start_positions_2 = [(2, -1), (2, -2), (2, -3)]

        # Action clients for NavigateToPose
        self.action_client_1 = ActionClient(self, NavigateToPose, f'{self.namespace}/navigate_to_pose')
        self.action_client_2 = ActionClient(self, NavigateToPose, f'{self.namespace2}/navigate_to_pose')

        # Publisher for cmd_vel
        self.cmd_vel_publisher_1 = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)
        self.cmd_vel_publisher_2 = self.create_publisher(Twist, f'{self.namespace2}/cmd_vel', 10)

        # State management
        self.initial_goal_reached_1 = False
        self.initial_goal_reached_2 = False
        self.start_circular_motion_1 = False
        self.start_circular_motion_2 = False

        self.start_time_1 = None
        self.start_time_2 = None

        # Timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Set robot states for different radii
        self.current_radius_index = 0  # Start with the first radius
        self.num_radii = len(self.radii_1)

    def send_initial_goal(self, namespace, position, action_client):
        """Send the initial goal to the NavigateToPose action server."""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(position[0])
        goal_msg.pose.pose.position.y = float(position[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation

        self.get_logger().info(f'Sending goal for {namespace} to position {position}...')
        action_client.wait_for_server()
        future = action_client.send_goal_async(goal_msg)
        if namespace == self.namespace:
            future.add_done_callback(self.goal_response_callback_1)
        else:
            future.add_done_callback(self.goal_response_callback_2)

    def goal_response_callback_1(self, future):
        """Handle the response for robot 1."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Robot 1 goal was rejected!')
            return

        self.get_logger().info('Robot 1 goal accepted!')
        goal_handle.get_result_async().add_done_callback(self.result_callback_1)

    def goal_response_callback_2(self, future):
        """Handle the response for robot 2."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Robot 2 goal was rejected!')
            return

        self.get_logger().info('Robot 2 goal accepted!')
        goal_handle.get_result_async().add_done_callback(self.result_callback_2)

    def result_callback_1(self, future):
        """Handle the result for robot 1."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Robot 1 reached goal position!')
            self.initial_goal_reached_1 = True
        else:
            self.get_logger().info(f'Robot 1 goal failed with status: {result.status}')

    def result_callback_2(self, future):
        """Handle the result for robot 2."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Robot 2 reached goal position!')
            self.initial_goal_reached_2 = True
        else:
            self.get_logger().info(f'Robot 2 goal failed with status: {result.status}')

    def generate_circular_motion(self, radius, linear_velocity, clockwise):
        """Calculate angular velocity and return a Twist message."""
        angular_velocity = linear_velocity / radius
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = -angular_velocity if clockwise else angular_velocity
        return twist_msg

    def control_loop(self):
        """Control loop for managing both robots."""
        if self.current_radius_index < self.num_radii:
            radius_1 = self.radii_1[self.current_radius_index]
            position_1 = self.start_positions_1[self.current_radius_index]
            radius_2 = self.radii_2[self.current_radius_index]
            position_2 = self.start_positions_2[self.current_radius_index]

            # Robot 1: Send initial goal if not reached
            if not self.initial_goal_reached_1:
                self.send_initial_goal(self.namespace, position_1, self.action_client_1)

            # Robot 2: Send initial goal if not reached
            if not self.initial_goal_reached_2:
                self.send_initial_goal(self.namespace2, position_2, self.action_client_2)

            # Wait until both robots reach their goals
            if self.initial_goal_reached_1 and self.initial_goal_reached_2:

                elapsed_time_1 = (self.get_clock().now() - self.start_time_1).nanoseconds * 1e-9 if self.start_time_1 else 0.0
                elapsed_time_2 = (self.get_clock().now() - self.start_time_2).nanoseconds * 1e-9 if self.start_time_2 else 0.0

                if not self.start_circular_motion_1:
                    self.start_circular_motion_1 = True
                    self.start_time_1 = self.get_clock().now()
                elif elapsed_time_1 < 20.0:
                    twist_msg_1 = self.generate_circular_motion(radius_1, self.linear_velocity, clockwise=True)
                    self.cmd_vel_publisher_1.publish(twist_msg_1)
                else:
                    self.cmd_vel_publisher_1.publish(Twist())  # Stop robot 1

                if not self.start_circular_motion_2:
                    self.start_circular_motion_2 = True
                    self.start_time_2 = self.get_clock().now()
                elif elapsed_time_2 < 20.0:
                    twist_msg_2 = self.generate_circular_motion(radius_2, self.linear_velocity, clockwise=False)
                    self.cmd_vel_publisher_2.publish(twist_msg_2)
                else:
                    self.cmd_vel_publisher_2.publish(Twist())  # Stop robot 2

                if elapsed_time_1 >= 20.0 and elapsed_time_2 >= 20.0:
                    self.get_logger().info(f'Completed radius {radius_1} for both robots.')
                    self.current_radius_index += 1

                    # Reset flags for the next radius
                    self.initial_goal_reached_1 = False
                    self.start_circular_motion_1 = False
                    self.initial_goal_reached_2 = False
                    self.start_circular_motion_2 = False
                    self.start_time_1 = None
                    self.start_time_2 = None

            else:
                self.get_logger().info('Waiting for both robots to reach initial positions...')

        else:
            self.get_logger().info('Completed all circles for both robots. Shutting down.')
            self.send_initial_goal(self.namespace, (0, 0), self.action_client_1)
            self.send_initial_goal(self.namespace2, (2, 0), self.action_client_2)
            self.timer.cancel()

def main():
    parser = ArgumentParser(description='Dual circular trajectory controller.')
    parser.add_argument('vel', type=float, help='Set the constant linear velocity (e.g., 0.8, 1.0, 1.5 m/s)')
    args = parser.parse_args()

    rclpy.init()  # Initialize without passing argparse arguments

    node = DualCircularTrajectoryPublisher(linear_velocity=args.vel)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
