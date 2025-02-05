#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
import math
from tf_transformations import quaternion_from_euler
import socket
from matplotlib import pyplot as plt
import numpy as np
from numpy import typing as npt
from copy import deepcopy
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_srvs.srv import SetBool
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
from typing import Optional


ROBOT1 = "ep02" # socket.gethostname()
ROBOT2 = "ep03"

class GridActionNavigator(Node):
    def __init__(self):
        super().__init__('grid_action_navigator_test')

        self.robot1_waypoints_traj_pub = self.create_publisher(Path, ROBOT1+"/waypoints_traj", 10)
        self.robot2_waypoints_traj_pub = self.create_publisher(Path, ROBOT2+"/waypoints_traj", 10)
        self.robot1_waypoints_traj_msg: Path = Path()
        self.robot2_waypoints_traj_msg: Path = Path()
        # Create an action client for the NavigateToPose action
        self.robot1_action_client = ActionClient(self, NavigateToPose, ROBOT1 + '/navigate_to_pose')
        self.robot2_action_client = ActionClient(self, NavigateToPose, ROBOT2 + '/navigate_to_pose')
        # TODO (Sachin): Add while loop that will continously check the action servers and print a warning message
        self.get_logger().info("waiting for action server")
        self.robot1_action_client.wait_for_server()
        self.robot2_action_client.wait_for_server()
        self.get_logger().info("action servers found")

        # Generate grid waypoints
        self.robot1_waypoints: npt.NDArray = self.generate_grid_waypoints(x_start=7, x_end=-7, y_start=5.5, y_end=-2.5, step=0.5)
        self.robot2_waypoints: npt.NDArray = deepcopy(self.robot1_waypoints)
        y_offset = 0.8
        self.robot2_waypoints.T[1] -= y_offset
        self.current_waypoint_index: int = 0
        self.robot1_index: int = 0
        self.robot2_index: int = 0

        self.prepare_trajectory_path()
        self.create_timer(1.0, self.publish_waypoint_traj)
        
        # Goal messages for both robots
        self.robot1_goal_msg: NavigateToPose.Goal = NavigateToPose.Goal()
        self.robot1_goal_msg.pose.header.frame_id = 'map'
        self.robot2_goal_msg = NavigateToPose.Goal()
        self.robot2_goal_msg.pose.header.frame_id = 'map'

        self._robot1_goal_handle : Optional[ClientGoalHandle] = None
        self._robot2_goal_handle : Optional[ClientGoalHandle] = None

        self.create_service(SetBool, "start_nav", self.start_nav_cb)
        self.get_logger().info("service started")

        # Start the navigation process
        # self.navigate_to_next_waypoint()
        # self.robot1_send_goal_with_index(self.robot1_index)
        # self.robot2_send_goal_with_index(self.robot2_index)

    def start_nav_cb(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.robot1_send_goal_with_index(self.robot1_index)
            self.robot2_send_goal_with_index(self.robot2_index)
            response.message = "Navigation started"
            response.success = True
        else:
            if self._robot1_goal_handle is None and self._robot2_goal_handle is None:
                response.message = "There is no goal to cancel"
                response.success = True
            else:
                self.stop_robots()

                # TODO (Sachin): stop the nav
                response.message = "Navigation stopped"
                response.success = False
            pass
        return response
    
    def stop_robots(self):
        r1_future: Future = self._robot1_goal_handle.cancel_goal_async()
        r2_future: Future = self._robot2_goal_handle.cancel_goal_async()
        try:
            result1 = r1_future.result()
            result2 = r2_future.result()
            self.get_logger().info(f"result from cancel {result1}")
            self.get_logger().info(f"result from cancel {result2}")
        except:
            self.get_logger().error("Nav2 goal was not able to cancel")


    def prepare_trajectory_path(self):
        self.robot1_waypoints_traj_msg.header.frame_id = "map"
        self.robot2_waypoints_traj_msg.header.frame_id = "map" # TODO (Sachin) : Change this to robot2
        for waypoint in self.robot1_waypoints:
            pose1 = PoseStamped()
            pose1.header.frame_id = "map"
            pose1.pose.position.x = float(waypoint[0])
            pose1.pose.position.y = float(waypoint[1])
            pose1.pose.orientation.w = 1.0
            self.robot1_waypoints_traj_msg.poses.append(pose1)

        for waypoint in self.robot2_waypoints:
            pose2 = PoseStamped()
            pose2.header.frame_id = "map"
            pose2.pose.position.x = float(waypoint[0])
            pose2.pose.position.y = float(waypoint[1])
            pose2.pose.orientation.w = 1.0
            self.robot2_waypoints_traj_msg.poses.append(pose2)

    def publish_waypoint_traj(self):
        self.robot1_waypoints_traj_pub.publish(self.robot1_waypoints_traj_msg)
        self.robot2_waypoints_traj_pub.publish(self.robot2_waypoints_traj_msg)
        # self.get_logger().info("publishing waypoints")

    def generate_grid_waypoints(self, x_start: float, x_end: float, y_start: float, y_end: float, step: float):
        """Generate waypoints for a grid-like path."""
        x_pts = []
        y_pts = []
        y = float(y_start)  # Start at the top (y_start)
        direction = -1  # Start moving from +x to -x
        while y >= y_end:
            if direction == -1:  # Moving from +x to -x
                x_pts.append(x_start)
                y_pts.append(y)
                x_pts.append(x_end)
                y_pts.append(y)
                # waypoints.append({'x': float(x_start), 'y': y})
                # waypoints.append({'x': float(x_end), 'y': y})
            else:  # Moving from -x to +x
                x_pts.append(x_end)
                y_pts.append(y)
                x_pts.append(x_start)
                y_pts.append(y)
                # waypoints.append({'x': float(x_end), 'y': y})
                # waypoints.append({'x': float(x_start), 'y': y})
            # Move to the next y-level
            y -= step
            # Switch direction for the next row
            direction *= -1
        return np.array([x_pts, y_pts], dtype=np.float32).T
    
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
        if self.current_waypoint_index < len(self.robot1_waypoints):
            # Waypoints for both robots

            # Goal messages for both robots
            self.robot1_goal_msg.pose.pose.position.x = float(self.robot1_waypoints[self.current_waypoint_index][0])
            self.robot1_goal_msg.pose.pose.position.y = float(self.robot1_waypoints[self.current_waypoint_index][1])
            # goal_msg1.pose.header.stamp = self.get_clock().now().to_msg() # TODO (Sachin): add the timestamp before calling

            self.robot2_goal_msg.pose.pose.position.x = float(self.robot2_waypoints[self.current_waypoint_index][0])
            self.robot2_goal_msg.pose.pose.position.y = float(self.robot2_waypoints[self.current_waypoint_index][1])

            # Calculate orientation if there is a next waypoint
            if self.current_waypoint_index < len(self.robot1_waypoints) - 1:
                next_wp1 = self.robot1_waypoints[self.current_waypoint_index + 1]
                waypoint_data1 = self.robot1_waypoints[self.current_waypoint_index]
                quaternion1 = calculate_orientation(
                    waypoint_data1[0], waypoint_data1[1],
                    next_wp1[0], next_wp1[1]
                )
                self.robot1_goal_msg.pose.pose.orientation.x = quaternion1[0]
                self.robot1_goal_msg.pose.pose.orientation.y = quaternion1[1]
                self.robot1_goal_msg.pose.pose.orientation.z = quaternion1[2]
                self.robot1_goal_msg.pose.pose.orientation.w = quaternion1[3]

                next_wp2 = self.robot2_waypoints[self.current_waypoint_index + 1]
                waypoint_data2 = self.robot1_waypoints[self.current_waypoint_index]
                quaternion2 = calculate_orientation(
                    waypoint_data2[0], waypoint_data2[1],
                    next_wp2[0], next_wp2[1]
                )
                self.robot2_goal_msg.pose.pose.orientation.x = quaternion2[0]
                self.robot2_goal_msg.pose.pose.orientation.y = quaternion2[1]
                self.robot2_goal_msg.pose.pose.orientation.z = quaternion2[2]
                self.robot2_goal_msg.pose.pose.orientation.w = quaternion2[3]
            else:
                # Default orientation for the last waypoint
                self.robot1_goal_msg.pose.pose.orientation.w = 1.0
                self.robot2_goal_msg.pose.pose.orientation.w = 1.0

            # add timestamp
            self.robot1_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            self.robot2_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            # Logging and sending goals
            self.get_logger().info(f"Sending goals to waypoint {self.current_waypoint_index + 1}/{len(self.robot1_waypoints)}")
            self._send_goal_future1 = self.robot1_action_client.send_goal_async(
                self.robot1_goal_msg, feedback_callback=self.robot1_feedback_callback
            )
            self._send_goal_future2 = self.robot2_action_client.send_goal_async(
                self.robot2_goal_msg, feedback_callback=self.robot2_feedback_callback
            )
            self._send_goal_future1.add_done_callback(self.robot1_goal_response_callback)
            self._send_goal_future2.add_done_callback(self.robot2_goal_response_callback) ### this one can be used for a new callback 
        else:
            self.get_logger().info("All waypoints completed!")

    def robot1_feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        
        # self.get_logger().info(f"Robot1 Feedback received: {feedback}")

    def robot1_goal_response_callback(self, future: Future):
        """Handle the response from the action server."""
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Robot1 Goal was rejected by the server")
            # TODO (Sachin): If goal was rejected try to send the goal again
            return
        
        self._robot1_goal_handle = goal_handle
        
        self.get_logger().info("Goal accepted by the server")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot1_result_callback)

    def robot1_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Robot1 Goal reached for waypoint {self.robot1_index+ 1}")
            # self.current_waypoint_index += 1
            # self.navigate_to_next_waypoint()
            self.robot1_index += 1
            if self.robot1_index < len(self.robot1_waypoints):
                self.robot1_send_goal_with_index(self.robot1_index)
            else:
                self.get_logger().info("robot2 reached all goals")
        else:
            self.get_logger().error("Robot1 Goal failed or was canceled")

    def robot2_feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Robot2 Feedback received: {feedback}")

    def robot2_goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Robot2 Goal was rejected by the server")
            return
        
        self._robot2_goal_handle = goal_handle
        
        
        self.get_logger().info("Robot2 Goal accepted by the server")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot2_result_callback)

    def robot2_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Robot2 Goal reached for waypoint {self.robot2_index+ 1}")
            # TODO (Sachin) : Uncomment this later if need to use the older version
            # self.current_waypoint_index += 1
            # self.navigate_to_next_waypoint()
            self.robot2_index += 1
            if self.robot2_index < len(self.robot2_waypoints):
                self.robot2_send_goal_with_index(self.robot2_index)
            else:
                self.get_logger().info("robot2 reached all goals")
        else:
            self.get_logger().error("Robot2 Goal failed or was canceled")

    def robot1_send_goal_with_index(self, index: int):
        # Goal messages for both robots
        self.robot1_goal_msg.pose.pose.position.x = float(self.robot1_waypoints[index][0])
        self.robot1_goal_msg.pose.pose.position.y = float(self.robot1_waypoints[index][1])

        # Calculate orientation if there is a next waypoint
        if index < len(self.robot1_waypoints) - 1:
            next_wp1 = self.robot1_waypoints[index + 1]
            waypoint_data1 = self.robot1_waypoints[index]
            quaternion1 = calculate_orientation(
                waypoint_data1[0], waypoint_data1[1],
                next_wp1[0], next_wp1[1]
            )
            self.robot1_goal_msg.pose.pose.orientation.x = quaternion1[0]
            self.robot1_goal_msg.pose.pose.orientation.y = quaternion1[1]
            self.robot1_goal_msg.pose.pose.orientation.z = quaternion1[2]
            self.robot1_goal_msg.pose.pose.orientation.w = quaternion1[3]

        else:
            # Default orientation for the last waypoint
            self.robot1_goal_msg.pose.pose.orientation.w = 1.0

        # add timestamp
        self.robot1_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Logging and sending goals
        self.get_logger().info(f"Sending goals to waypoint {index + 1}/{len(self.robot1_waypoints)}")
        self._send_goal_future1 = self.robot1_action_client.send_goal_async(
            self.robot1_goal_msg, feedback_callback=self.robot1_feedback_callback
        )
        self._send_goal_future1.add_done_callback(self.robot1_goal_response_callback)

    def robot2_send_goal_with_index(self, index: int):
        # Goal messages for both robots
        self.robot2_goal_msg.pose.pose.position.x = float(self.robot2_waypoints[index][0])
        self.robot2_goal_msg.pose.pose.position.y = float(self.robot2_waypoints[index][1])

        # Calculate orientation if there is a next waypoint
        if index < len(self.robot1_waypoints) - 1:
            next_wp2 = self.robot2_waypoints[index + 1]
            waypoint_data2 = self.robot1_waypoints[index]
            quaternion2 = calculate_orientation(
                waypoint_data2[0], waypoint_data2[1],
                next_wp2[0], next_wp2[1]
            )
            self.robot2_goal_msg.pose.pose.orientation.x = quaternion2[0]
            self.robot2_goal_msg.pose.pose.orientation.y = quaternion2[1]
            self.robot2_goal_msg.pose.pose.orientation.z = quaternion2[2]
            self.robot2_goal_msg.pose.pose.orientation.w = quaternion2[3]
        else:
            # Default orientation for the last waypoint
            self.robot2_goal_msg.pose.pose.orientation.w = 1.0

        # add timestamp
        self.robot2_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        # Logging and sending goals
        self.get_logger().info(f"Sending goals to waypoint {index + 1}/{len(self.robot1_waypoints)}")
        self._send_goal_future2 = self.robot2_action_client.send_goal_async(
            self.robot2_goal_msg, feedback_callback=self.robot2_feedback_callback
        )
        self._send_goal_future2.add_done_callback(self.robot2_goal_response_callback) ### this one can be used for a new callback 


@staticmethod
def calculate_orientation(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Calculate quaternion for orientation based on the direction of movement."""
    delta_x = to_x - from_x
    delta_y = to_y - from_y
    yaw = math.atan2(delta_y, delta_x)  # Calculate the yaw angle
    quaternion = quaternion_from_euler(0, 0, yaw)  # Convert yaw to quaternion
    return quaternion

def main(args=None):
    rclpy.init(args=args)
    node = GridActionNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
