#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    
    def __init__(self, namespace=''):
        super().__init__('webcam_publisher', namespace=namespace)
        
        # Create publishers for compressed images and camera info
        self.image_publisher = self.create_publisher(CompressedImage, f'{namespace}/ep03/image_raw/compressed', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, f'{namespace}/ep03/image_raw/camera_info', 10)
        
        # Initialize the OpenCV bridge for ROS2 image conversion
        self.bridge = CvBridge()
        
        # Set the publishing rate (5 Hz)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # Open the webcam (default is /dev/video0; change if necessary)
        self.capture = cv2.VideoCapture(0)
        
        # Check if the webcam opened successfully
        if not self.capture.isOpened():
            self.get_logger().error("Could not open webcam.")
            raise RuntimeError("Webcam not accessible.")
        
        self.get_logger().info("Webcam publisher initialized and streaming.")

        # Define default camera parameters
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.camera_info_msg.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [500.0, 0.0, self.camera_info_msg.width / 2.0,  # fx, 0, cx
                                  0.0, 500.0, self.camera_info_msg.height / 2.0,  # 0, fy, cy
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.r = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.p = [500.0, 0.0, self.camera_info_msg.width / 2.0, 0.0,  # fx, 0, cx, 0
                                  0.0, 500.0, self.camera_info_msg.height / 2.0, 0.0,  # 0, fy, cy, 0
                                  0.0, 0.0, 1.0, 0.0]

    def timer_callback(self):
        # Capture a frame from the webcam
        ret, frame = self.capture.read()
        
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return
        
        # Convert the OpenCV frame to a ROS2 CompressedImage message
        image_msg = CompressedImage()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "ep03/wave_sensor_link"  # Attach the frame to 'camera_link'
        image_msg.format = "jpeg"
        image_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()  # Encode image to JPEG format
        
        # Publish the compressed image
        self.image_publisher.publish(image_msg)

        # Update the CameraInfo message header
        self.camera_info_msg.header.stamp = image_msg.header.stamp
        
        # Publish the camera info
        self.camera_info_publisher.publish(self.camera_info_msg)
        self.get_logger().debug("Published a compressed frame and CameraInfo.")

    def __del__(self):
        # Release the webcam resource on shutdown
        self.capture.release()
        self.get_logger().info("Webcam resource released.")

def main(args=None):
    rclpy.init(args=args)
    
    # Get the namespace from the command line arguments or use default
    namespace = ''
    if args and len(args) > 1:
        namespace = args[1]
    
    node = WebcamPublisher(namespace=namespace)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Webcam Publisher.")
    finally:
        # Ensure resources are released
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
