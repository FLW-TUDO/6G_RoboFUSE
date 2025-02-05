import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
from tf_transformations import quaternion_from_euler
import socket


class GridActionNavigator(Node):
    def __init__(self):
        super().__init__('grid_action_navigator')

        self.namespace = socket.gethostname()
        self.action_client = ActionClient(self, NavigateToPose, self.namespace + '/navigate_to_pose')

        self.waypoints = self.generate_grid_waypoints(x_start=4.0, x_end=1.0, y_start=2.0, y_end=-2.9, step=0.7)
        self.current_waypoint_index = 0

        # Variable to track distance status
        self.robot_stopped = False
        self.last_distance = float('inf')
        self.current_goal_handle = None

        # Subscription for distance topic
        self.distance_sub = self.create_subscription(
            Float32,
            'ep02_ep03/relative_distance',
            self.distance_callback,
            QoSProfile(depth=10)
        )

        self.navigate_to_next_waypoint()

    def generate_grid_waypoints(self, x_start, x_end, y_start, y_end, step):
        waypoints = []
        y = float(y_start)
        direction = -1
        while y >= y_end:
            if direction == -1:
                waypoints.append({'x': float(x_start), 'y': y})
                waypoints.append({'x': float(x_end), 'y': y})
            else:
                waypoints.append({'x': float(x_end), 'y': y})
                waypoints.append({'x': float(x_start), 'y': y})
            y -= step
            direction *= -1
        return waypoints

    def distance_callback(self, msg):
        """Handle updates from the distance sensor."""
        self.last_distance = msg.data
        if self.last_distance < 1.0:  # Distance 
            if not self.robot_stopped:
                self.robot_stopped = True
                self.get_logger().info("Stopping robot: distance below 50 cm.")
                self.cancel_current_goal()
        else:
            if self.robot_stopped:
                self.robot_stopped = False
                self.get_logger().info("Resuming robot: distance above 50 cm.")
                self.navigate_to_next_waypoint()

    def cancel_current_goal(self):
        """Cancel the current goal and delete the current waypoint."""
        if self.current_goal_handle:
            self.get_logger().info("Canceling current goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.current_waypoint_index -= 1

    def cancel_done_callback(self, future):
        """Callback after canceling the current goal."""
        cancel_response = future.result()
        if cancel_response:
            self.get_logger().info("Goal successfully canceled.")
            # Remove the current waypoint
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f"Deleting waypoint {self.current_waypoint_index + 1}.")
                self.waypoints.pop(self.current_waypoint_index)

    def navigate_to_next_waypoint(self):
        """Send the next waypoint as a goal if not stopped."""
        if self.robot_stopped:
            self.get_logger().info("Robot is stopped due to proximity alert.")
            return

        if self.current_waypoint_index < len(self.waypoints):
            waypoint_data = self.waypoints[self.current_waypoint_index]

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = waypoint_data['x']
            goal_msg.pose.pose.position.y = waypoint_data['y']
            goal_msg.pose.pose.position.z = 0.0

            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_waypoint = self.waypoints[self.current_waypoint_index + 1]
                quaternion = calculate_orientation(
                    waypoint_data['x'], waypoint_data['y'],
                    next_waypoint['x'], next_waypoint['y']
                )
                goal_msg.pose.pose.orientation.x = quaternion[0]
                goal_msg.pose.pose.orientation.y = quaternion[1]
                goal_msg.pose.pose.orientation.z = quaternion[2]
                goal_msg.pose.pose.orientation.w = quaternion[3]
            else:
                goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(f"Sending goal to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: {waypoint_data}")

            self.action_client.wait_for_server()
            self._send_goal_future = self.action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("All waypoints completed!")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server")
            return

        self.get_logger().info("Goal accepted by the server")
        self.current_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Goal reached for waypoint {self.current_waypoint_index + 1}")
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error("Goal failed or was canceled")

def calculate_orientation(from_x, from_y, to_x, to_y):
    delta_x = to_x - from_x
    delta_y = to_y - from_y
    yaw = math.atan2(delta_y, delta_x)
    quaternion = quaternion_from_euler(0, 0, yaw)
    return quaternion

def main(args=None):
    rclpy.init(args=args)
    node = GridActionNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
