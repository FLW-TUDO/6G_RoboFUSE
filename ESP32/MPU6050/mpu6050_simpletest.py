import time
import board
import adafruit_mpu6050
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class IMUPublisher(Node):
    CALIBRATION_SAMPLES = 100  # Number of samples to collect for calibration

    def __init__(self):
        super().__init__('imu_publisher')  # Initialize the ROS2 node with the name 'imu_publisher'
        # Create a publisher for Float32MultiArray messages on the 'ros2_imu' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ros2_imu', 20)
        # Set up a timer to call publish_imu_data every second
        self.timer = self.create_timer(0.05, self.publish_imu_data)
        # Initialize I2C communication and create an instance of the MPU6050 sensor
        self.mpu = adafruit_mpu6050.MPU6050(board.I2C())
        # Initialize offset arrays for calibration (acceleration and gyroscope)
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)
        # Perform calibration of the sensor
        self.calibrate_sensor()
        # Flag to control whether publishing is active
        self.running = True  

    def calibrate_sensor(self):
        # Arrays to accumulate readings for calibration
        accel_total = np.zeros(3)
        gyro_total = np.zeros(3)
        self.get_logger().info('Calibrating sensor...')  # Log the start of calibration

        # Collect multiple samples for calibration
        for _ in range(self.CALIBRATION_SAMPLES):
            try:
                # Read acceleration and gyro data from the sensor
                accel = self.mpu.acceleration
                gyro = self.mpu.gyro
                # Accumulate the readings
                accel_total += accel
                gyro_total += gyro
                time.sleep(0.005)  # Short delay between samples
            except Exception as e:
                # Log any errors that occur during calibration
                self.get_logger().error(f'Error during calibration: {e}')
                return

        # Calculate average offsets for acceleration and gyro
        self.accel_offset = accel_total / self.CALIBRATION_SAMPLES
        # Adjust for gravity on the Z-axis
        self.accel_offset[2] -= 9.81  # Gravity in m/s²
        self.get_logger().info(f'Calibration complete. Accel offset: {self.accel_offset}, Gyro offset: {self.gyro_offset}')

    def publish_imu_data(self):
        # Check if the publishing should continue
        if not self.running:  
            return  # Exit the function if the flag is False

        try:
            # Retrieve current sensor readings
            accel = self.mpu.acceleration
            gyro = self.mpu.gyro
            #temperature = self.mpu.temperature
            
            # Apply calibration offsets to the readings
            accel_calibrated = accel - self.accel_offset
            gyro_calibrated = gyro - self.gyro_offset

            # Create a message with the calibrated data
            imu_msg = Float32MultiArray(data=np.concatenate((accel_calibrated, gyro_calibrated)))
            # Publish the message
            self.publisher_.publish(imu_msg)
            # Log the published data for monitoring
            self.get_logger().info(f'Publishing IMU data: Accel({accel_calibrated}), Gyro({gyro_calibrated})')
        except Exception as e:
            # Log any errors encountered while reading or publishing data
            self.get_logger().error(f'Error reading from IMU: {e}')

    def stop_publishing(self):
        # Method to stop publishing data
        self.running = False  # Set the flag to False to stop publishing

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    imu_publisher = IMUPublisher()  # Create an instance of the IMUPublisher

    try:
        rclpy.spin(imu_publisher)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        # Handle Ctrl+C (KeyboardInterrupt) to stop publishing 
        imu_publisher.stop_publishing()  # Call the method to stop publishing
        imu_publisher.get_logger().info('Shutting down IMU publisher...')  # Log shutdown message

    # Clean up and shutdown
    imu_publisher.destroy_node()  # Destroy the publisher node
    rclpy.shutdown()  # Shutdown the ROS2 client library

if __name__ == '__main__':
    main()  # Execute the main function to start the node

