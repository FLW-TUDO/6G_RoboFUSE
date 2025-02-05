#### If want to use the waypoint follower ###
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import tf_transformations
import json
import paho.mqtt.client as mqtt
import tf2_ros
from math import atan2
import socket

class MQTTWaypointReceiver(Node):
    def __init__(self):
        super().__init__('mqtt_waypoint_receiver')
        self.get_logger().info("Initializing MQTTWaypointReceiver node...")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client_ptr = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.declare_parameter('namespace', '')
        self.namespace_ = self.get_parameter('namespace').get_parameter_value().string_value
        self.get_logger().info(f"Namespace parameter: {self.namespace_}")
        hostname =  socket.gethostname()

        MQTT_BROKER_ADDRESS = "192.168.2.68" 
        MQTT_BROKER_PORT = 1883
        MQTT_CLIENT_ID = f"RM_{hostname}"
        self._topic = self.namespace_+"/waypoints"

        self.mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        try:
            self.mqtt_client.connect(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT)
            self.get_logger().info("Connected to MQTT Broker")
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT Broker: {e}")

        self.poses_ = []

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker successfully")
            self.get_logger().info(f"TOPIC 1: {self._topic}")
            self.mqtt_client.subscribe(self._topic)
            self.get_logger().info(f"TOPIC 2: {self._topic}") 
            self.get_logger().info(f"Subscribed to topic: {self._topic}")
        else:
            self.get_logger().error(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"Received message from topic {msg.topic}: {msg.payload.decode()}")
        try:
            data = json.loads(msg.payload.decode())
            self.mqtt_callback(data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON message: {e}")

    def mqtt_callback(self, data):
        self.get_logger().info("MQTT callback")

        try:
            transformStamped = self.tf_buffer.lookup_transform("map", self.namespace_+"/base_footprint", rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"TransformException: {ex}")
            return

        last_pose = PoseStamped()
        last_pose.header.frame_id = "map"
        last_pose.pose.position.x = transformStamped.transform.translation.x
        last_pose.pose.position.y = transformStamped.transform.translation.y
        last_pose.pose.position.z = transformStamped.transform.translation.z
        last_pose.pose.orientation = transformStamped.transform.rotation

        self.poses_.clear()
        for element in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = element["x"]
            pose.pose.position.y = element["y"]
            pose.pose.position.z = element["z"]

            heading = atan2(pose.pose.position.y - last_pose.pose.position.y, pose.pose.position.x - last_pose.pose.position.x)
            quaternion = tf_transformations.quaternion_about_axis(heading, (0, 0, 1))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            last_pose = pose

            self.poses_.append(pose)

        self.publish_waypoints()

    def publish_waypoints(self):
        self.get_logger().info("Publishing waypoints")
        
        if not self.poses_:
            self.get_logger().error("No waypoints to publish")
            return

        for pose in self.poses_:
            self.get_logger().info(f"Waypoint: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")

        if not self.client_ptr.wait_for_server(timeout_sec=10):
            self.get_logger().error("Action server not available after waiting")
            rclpy.shutdown()
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.poses_

        self.get_logger().info(f"Sending goal with {len(goal_msg.poses)} poses")
        for i, pose in enumerate(goal_msg.poses):
            self.get_logger().info(f"Pose {i}: Frame ID({pose.header.frame_id}), Position({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}), Orientation({pose.pose.orientation.x}, {pose.pose.orientation.y}, {pose.pose.orientation.z}, {pose.pose.orientation.w})")

        self.client_ptr.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by server")
            else:
                self.get_logger().info("Goal accepted by server, waiting for result")
                goal_handle.result().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal response failed: {e}")

    # def feedback_callback(self, goal_handle, feedback):
    #     self.get_logger().info(f"Received feedback: {feedback.navigation_time.sec}")

    def result_callback(self, future):
        try:
            result = future.result()
            if result.result.code == rclpy.action.ResultCode.SUCCEEDED:
                self.get_logger().info("Navigation finished")
            elif result.result.code == rclpy.action.ResultCode.ABORTED:
                self.get_logger().error("Goal was aborted")
            elif result.result.code == rclpy.action.ResultCode.CANCELED:
                self.get_logger().error("Goal was canceled")
            else:
                self.get_logger().error("Unknown result code")
        except Exception as e:
            self.get_logger().error(f"Result callback failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MQTTWaypointReceiver()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger("mqtt_waypoint_receiver").error(f"Exception in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
### For the navigation throgh-poses__ fail sometimes
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateThroughPoses
# import tf_transformations
# import json
# import paho.mqtt.client as mqtt
# import tf2_ros
# from math import atan2
# import socket

# class MQTTWaypointReceiver(Node):
#     def __init__(self):
#         super().__init__('mqtt_waypoint_receiver')
#         self.get_logger().info("Initializing MQTTWaypointReceiver node...")
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.client_ptr = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

#         self.declare_parameter('namespace', '')
#         self.namespace_ = self.get_parameter('namespace').get_parameter_value().string_value
#         self.get_logger().info(f"Namespace parameter: {self.namespace_}")
#         hostname =  socket.gethostname()

#         MQTT_BROKER_ADDRESS = "192.168.2.187" 
#         MQTT_BROKER_PORT = 1883
#         MQTT_CLIENT_ID = f"RM_{hostname}"
#         self._topic = self.namespace_+"/waypoints"

#         self.mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)
#         self.mqtt_client.on_connect = self.on_connect
#         self.mqtt_client.on_message = self.on_message

#         try:
#             self.mqtt_client.connect(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT)
#             self.get_logger().info("Connected to MQTT Broker")
#             self.mqtt_client.loop_start()
#         except Exception as e:
#             self.get_logger().error(f"Failed to connect to MQTT Broker: {e}")

#         self.poses_ = []

#     def on_connect(self, client, userdata, flags, rc):
#         if rc == 0:
#             self.get_logger().info("Connected to MQTT Broker successfully")
#             self.mqtt_client.subscribe(self._topic)
#             self.get_logger().info(f"Subscribed to topic: {self._topic}")
#         else:
#             self.get_logger().error(f"Failed to connect, return code {rc}")

#     def on_message(self, client, userdata, msg):
#         self.get_logger().info(f"Received message from topic {msg.topic}: {msg.payload.decode()}")
#         try:
#             data = json.loads(msg.payload.decode())
#             self.mqtt_callback(data)
#         except json.JSONDecodeError as e:
#             self.get_logger().error(f"Failed to decode JSON message: {e}")

#     def mqtt_callback(self, data):
#         self.get_logger().info("MQTT callback")

#         try:
#             transformStamped = self.tf_buffer.lookup_transform("map", self.namespace_+"/base_footprint", rclpy.time.Time())
#         except tf2_ros.TransformException as ex:
#             self.get_logger().error(f"TransformException: {ex}")
#             return

#         last_pose = PoseStamped()
#         last_pose.header.frame_id = "map"
#         last_pose.pose.position.x = transformStamped.transform.translation.x
#         last_pose.pose.position.y = transformStamped.transform.translation.y
#         last_pose.pose.position.z = transformStamped.transform.translation.z
#         last_pose.pose.orientation = transformStamped.transform.rotation

#         self.poses_.clear()
#         for element in data:
#             pose = PoseStamped()
#             pose.header.frame_id = "map"
#             pose.pose.position.x = element["x"]
#             pose.pose.position.y = element["y"]
#             pose.pose.position.z = element["z"]

#             heading = atan2(pose.pose.position.y - last_pose.pose.position.y, pose.pose.position.x - last_pose.pose.position.x)
#             quaternion = tf_transformations.quaternion_about_axis(heading, (0, 0, 1))
#             pose.pose.orientation.x = quaternion[0]
#             pose.pose.orientation.y = quaternion[1]
#             pose.pose.orientation.z = quaternion[2]
#             pose.pose.orientation.w = quaternion[3]

#             last_pose = pose

#             self.poses_.append(pose)

#         self.publish_waypoints()

#     def publish_waypoints(self):
#         self.get_logger().info("Publishing waypoints")
        
#         if not self.poses_:
#             self.get_logger().error("No waypoints to publish")
#             return

#         for pose in self.poses_:
#             self.get_logger().info(f"Waypoint: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")

#         if not self.client_ptr.wait_for_server(timeout_sec=10):
#             self.get_logger().error("Action server not available after waiting")
#             rclpy.shutdown()
#             return

#         goal_msg = NavigateThroughPoses.Goal()
#         goal_msg.poses = self.poses_

#         self.get_logger().info(f"Sending goal with {len(goal_msg.poses)} poses")
#         for i, pose in enumerate(goal_msg.poses):
#             self.get_logger().info(f"Pose {i}: Frame ID({pose.header.frame_id}), Position({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}), Orientation({pose.pose.orientation.x}, {pose.pose.orientation.y}, {pose.pose.orientation.z}, {pose.pose.orientation.w})")

#         self.client_ptr.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
#     def goal_response_callback(self, future):
#         try:
#             goal_handle = future.result()
#             if not goal_handle.accepted:
#                 self.get_logger().error("Goal was rejected by server")
#             else:
#                 self.get_logger().info("Goal accepted by server, waiting for result")
#                 goal_handle.result().add_done_callback(self.result_callback)
#         except Exception as e:
#             self.get_logger().error(f"Goal response failed: {e}")

#     # def feedback_callback(self, goal_handle, feedback):
#     #     self.get_logger().info(f"Received feedback: {feedback.navigation_time.sec}")

#     def result_callback(self, future):
#         try:
#             result = future.result()
#             if result.result.code == rclpy.action.ResultCode.SUCCEEDED:
#                 self.get_logger().info("Navigation finished")
#             elif result.result.code == rclpy.action.ResultCode.ABORTED:
#                 self.get_logger().error("Goal was aborted")
#             elif result.result.code == rclpy.action.ResultCode.CANCELED:
#                 self.get_logger().error("Goal was canceled")
#             else:
#                 self.get_logger().error("Unknown result code")
#         except Exception as e:
#             self.get_logger().error(f"Result callback failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = MQTTWaypointReceiver()
#         rclpy.spin(node)
#     except Exception as e:
#         rclpy.logging.get_logger("mqtt_waypoint_receiver").error(f"Exception in main: {e}")
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
