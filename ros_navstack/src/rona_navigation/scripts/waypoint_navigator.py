import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose  # Use appropriate action type
import yaml
import os
from tf_transformations import quaternion_from_euler
import socket

class WaypointActionNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_action_navigator')
        
        self.namespace = socket.gethostname()
        # Create an action client for the NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose,  self.namespace + '/navigate_to_pose')
        
        # Load waypoints from the YAML file
        waypoints_file = os.path.join(os.getcwd(), 'waypoints.yaml')  # Update with actual path
        self.waypoints = self.load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded. Check the YAML file.")
            return
        
        self.current_waypoint_index = 0
        self.navigate_to_next_waypoint()

    def load_waypoints(self, file_path):
        """Load waypoints from a YAML file."""
        try:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints file: {e}")
            return []

    def navigate_to_next_waypoint(self):
        """Send the next waypoint as a goal."""
        if self.current_waypoint_index < len(self.waypoints):
            waypoint_data = self.waypoints[self.current_waypoint_index]
            
            # Calculate orientation if there's a next waypoint
            orientation = self.calculate_orientation(waypoint_data)
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'  # Use appropriate frame
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = waypoint_data['x']
            goal_msg.pose.pose.position.y = waypoint_data['y']
            goal_msg.pose.pose.position.z = 0.0
            # Replace the fixed orientation with calculated quaternion
            goal_msg.pose.pose.orientation.x = orientation[0]
            goal_msg.pose.pose.orientation.y = orientation[1]
            goal_msg.pose.pose.orientation.z = orientation[2]
            goal_msg.pose.pose.orientation.w = orientation[3]
            
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
        self.get_logger().info(f"Feedback: {feedback}")

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

    def calculate_orientation(self, current_waypoint):
        """Calculate orientation quaternion based on movement direction."""
        import math
        from tf_transformations import quaternion_from_euler
        
        # Get next waypoint if available
        next_index = self.current_waypoint_index + 1
        if next_index < len(self.waypoints):
            next_waypoint = self.waypoints[next_index]
            # Calculate angle between current and next waypoint
            dx = next_waypoint['x'] - current_waypoint['x']
            dy = next_waypoint['y'] - current_waypoint['y']
            yaw = math.atan2(dy, dx)
        else:
            # For the last waypoint, use the orientation from the previous movement
            if self.current_waypoint_index > 0:
                prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
                dx = current_waypoint['x'] - prev_waypoint['x']
                dy = current_waypoint['y'] - prev_waypoint['y']
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0  # Default orientation for single waypoint
        
        # Convert euler angle to quaternion (roll=0, pitch=0)
        return quaternion_from_euler(0.0, 0.0, yaw)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointActionNavigator()
    if node.waypoints:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
