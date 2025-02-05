#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import quaternion_from_euler
import socket
from matplotlib import pyplot as plt

class GridActionNavigator(Node):
    def __init__(self):
        super().__init__('grid_action_navigator_test')

        self.namespace = socket.gethostname()
        self.sec_robot = "ep03"
        # Create an action client for the NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose, self.namespace + '/navigate_to_pose')
        self.action_client2 = ActionClient(self, NavigateToPose, self.sec_robot + '/navigate_to_pose')
        # Generate grid waypoints
        self.waypoints = self.generate_grid_waypoints(x_start=7, x_end=-7, y_start=5.5, y_end=-2.5, step=0.5)
        plt.plot(self.waypoints)
        plt.title('waypoints')
        plt.show()
        # self.waypoints2 = self.generate_separated_waypoints(self.waypoints, y_offset=0.7 )
        # self.current_waypoint_index = 0

        # # Start the navigation process
        # self.navigate_to_next_waypoint()

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
    
    def generate_separated_waypoints(self, base_waypoints, y_offset=0.5):
        """Generate waypoints for the second robot with a fixed y-axis offset."""
        separated_waypoints = []
        for waypoint in base_waypoints:
            separated_waypoints.append({
                'x': waypoint['x'],
                'y': waypoint['y'] - y_offset  # Apply a fixed offset in the y-axis
            })
        return separated_waypoints

    def navigate_to_next_waypoint(self):
        """Send the next waypoint as a goal to both robots."""
        if self.current_waypoint_index < len(self.waypoints):
            # Waypoints for both robots
            waypoint_data1 = self.waypoints[self.current_waypoint_index]
            waypoint_data2 = self.waypoints2[self.current_waypoint_index]

            # Goal messages for both robots
            goal_msg1 = NavigateToPose.Goal()
            goal_msg1.pose.header.frame_id = 'map'
            goal_msg1.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg1.pose.pose.position.x = waypoint_data1['x']
            goal_msg1.pose.pose.position.y = waypoint_data1['y']
            goal_msg1.pose.pose.position.z = 0.0

            goal_msg2 = NavigateToPose.Goal()
            goal_msg2.pose.header.frame_id = 'map'
            goal_msg2.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg2.pose.pose.position.x = waypoint_data2['x']
            goal_msg2.pose.pose.position.y = waypoint_data2['y']
            goal_msg2.pose.pose.position.z = 0.0

            # Calculate orientation if there is a next waypoint
            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_wp1 = self.waypoints[self.current_waypoint_index + 1]
                quaternion1 = calculate_orientation(
                    waypoint_data1['x'], waypoint_data1['y'],
                    next_wp1['x'], next_wp1['y']
                )
                goal_msg1.pose.pose.orientation.x = quaternion1[0]
                goal_msg1.pose.pose.orientation.y = quaternion1[1]
                goal_msg1.pose.pose.orientation.z = quaternion1[2]
                goal_msg1.pose.pose.orientation.w = quaternion1[3]

                next_wp2 = self.waypoints2[self.current_waypoint_index + 1]
                quaternion2 = calculate_orientation(
                    waypoint_data2['x'], waypoint_data2['y'],
                    next_wp2['x'], next_wp2['y']
                )
                goal_msg2.pose.pose.orientation.x = quaternion2[0]
                goal_msg2.pose.pose.orientation.y = quaternion2[1]
                goal_msg2.pose.pose.orientation.z = quaternion2[2]
                goal_msg2.pose.pose.orientation.w = quaternion2[3]
            else:
                # Default orientation for the last waypoint
                goal_msg1.pose.pose.orientation.w = 1.0
                goal_msg2.pose.pose.orientation.w = 1.0

            # Logging and sending goals
            self.get_logger().info(f"Sending goals to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            self.action_client.wait_for_server()
            self.action_client2.wait_for_server()
            self._send_goal_future1 = self.action_client.send_goal_async(
                goal_msg1, feedback_callback=self.feedback_callback
            )
            self._send_goal_future2 = self.action_client2.send_goal_async(
                goal_msg2, feedback_callback=self.feedback_callback
            )
            self._send_goal_future1.add_done_callback(self.goal_response_callback)
            self._send_goal_future2.add_done_callback(self.goal_response_callback) ### this one can be used for a new callback 
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
