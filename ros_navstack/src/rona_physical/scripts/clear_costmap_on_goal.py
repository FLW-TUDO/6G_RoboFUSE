#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray  
from nav2_msgs.srv import ClearEntireCostmap
import time  

class ClearCostmapOnGoal(Node):
    def __init__(self):
        super().__init__('clear_costmap_on_goal')

        # Subscribe to goal status updates from navigation
        self.goal_sub = self.create_subscription(
            GoalStatusArray, '/ep03/follow_path/_action/status', self.goal_callback, 10)

        # Create service clients for clearing costmaps
        self.global_costmap_client = self.create_client(ClearEntireCostmap, '/ep03/global_costmap/clear_entirely_global_costmap')
        self.local_costmap_client = self.create_client(ClearEntireCostmap, '/ep03/local_costmap/clear_entirely_local_costmap')

    def goal_callback(self, msg):
        if msg.status_list and msg.status_list[-1].status == 4:  # Status 4 = Goal Reached
            self.get_logger().info(" Goal reached")
            
            # time.sleep(1) 

            # Clear both global and local costmaps
            self.clear_costmap(self.global_costmap_client)
            self.clear_costmap(self.local_costmap_client)

    def clear_costmap(self, client):
        if client.wait_for_service(timeout_sec=1.0):
            request = ClearEntireCostmap.Request()
            future = client.call_async(request)
            self.get_logger().info("🧹 Costmap cleared!")
        else:
            self.get_logger().warn("⚠️ Costmap clear service not available!")

def main(args=None):
    rclpy.init(args=args)
    node = ClearCostmapOnGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
