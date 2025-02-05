import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import quaternion_from_euler
import socket

class GridActionNavigator(Node):
    def __init__(self):
        super().__init__('grid_action_navigator')

        self.namespace = socket.gethostname()
        # Create an action client for the NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose, self.namespace + '/navigate_to_pose')

        # Generate grid waypoints
        self.waypoints = self.generate_grid_waypoints(x_start=7, x_end=-8, y_start=5.5, y_end=-3.5, step=0.5)
        self.current_waypoint_index = 0

        # Start the navigation process
        self.navigate_to_next_waypoint()

    def generate_grid_waypoints(self, x_start, x_end, y_start, y_end, step):
        """Generate waypoints for a grid-like path."""
        waypoints = []
        y = float(y_start)  # Start at the top (y_start)
        direction = -1  # Start moving from +x to -x
        while y >= y_end:
            if direction == -1:  # Moving from +x to -x
                waypoints.append({'x': float(x_start), 'y': y})
                waypoints.append({'x': float(x_end), 'y': y})
            else:  # Moving from -x to +x
                waypoints.append({'x': float(x_end), 'y': y})
                waypoints.append({'x': float(x_start), 'y': y})
            # Move to the next y-level
            y -= step
            # Switch direction for the next row
            direction *= -1
        return waypoints
    


    def navigate_to_next_waypoint(self):
        """Send the next waypoint as a goal."""
        if self.current_waypoint_index < len(self.waypoints):
            waypoint_data = self.waypoints[self.current_waypoint_index]

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = waypoint_data['x']
            goal_msg.pose.pose.position.y = waypoint_data['y']
            goal_msg.pose.pose.position.z = 0.0

            # Calculate orientation if there's a next waypoint
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
                # Default orientation for the last waypoint
                goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(f"Sending goal to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: {waypoint_data}")
            
            # Wait for the action server to be ready
            self.action_client.wait_for_server()
            
            # Send the goal and attach callbacks
            self._send_goal_future = self.action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("All waypoints completed!")

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback received: {feedback}")
    


    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server")
            return
        
        self.get_logger().info("Goal accepted by the server")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Goal reached for waypoint {self.current_waypoint_index + 1}")
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error("Goal failed or was canceled")

def calculate_orientation(from_x, from_y, to_x, to_y):
    """Calculate quaternion for orientation based on the direction of movement."""
    delta_x = to_x - from_x
    delta_y = to_y - from_y
    yaw = math.atan2(delta_y, delta_x)  # Calculate the yaw angle
    quaternion = quaternion_from_euler(0, 0, yaw)  # Convert yaw to quaternion
    return quaternion

def main(args=None):
    rclpy.init(args=args)
    node = GridActionNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
