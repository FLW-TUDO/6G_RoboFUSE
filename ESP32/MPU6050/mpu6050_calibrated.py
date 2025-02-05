import time
import board
import adafruit_mpu6050
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ros2_imu', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        i2c = board.I2C()  # Initialize I2C connection
        self.mpu = adafruit_mpu6050.MPU6050(i2c)

        # Calibration variables
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]

        # Calibrate the sensor
        self.calibrate_sensor()

    def calibrate_sensor(self, calibration_samples=100):
        # Collect multiple samples for calibration
        accel_total = [0.0, 0.0, 0.0]
        gyro_total = [0.0, 0.0, 0.0]

        self.get_logger().info('Calibrating sensor...')

        for _ in range(calibration_samples):
            accel = self.mpu.acceleration
            gyro = self.mpu.gyro

            # Accumulate the readings
            for i in range(3):
                accel_total[i] += accel[i]
                gyro_total[i] += gyro[i]

            time.sleep(0.01)  # Small delay between samples

        # Calculate average offset
        self.accel_offset = [x / calibration_samples for x in accel_total]
        self.gyro_offset = [x / calibration_samples for x in gyro_total]

        # Adjust for gravity in Z-axis (assume sensor is static during calibration)
        self.accel_offset[2] -= 9.81  # Gravity in m/s²

        self.get_logger().info(f'Calibration complete. Accel offset: {self.accel_offset}, Gyro offset: {self.gyro_offset}')

    def publish_imu_data(self):
        # Retrieve acceleration and gyro data
        accel = self.mpu.acceleration
        gyro = self.mpu.gyro
        temperature = self.mpu.temperature

        # Apply calibration offsets
        accel_calibrated = [accel[i] - self.accel_offset[i] for i in range(3)]
        gyro_calibrated = [gyro[i] - self.gyro_offset[i] for i in range(3)]

        # Create message
        imu_msg = Float32MultiArray()
        imu_msg.data = [
            accel_calibrated[0], accel_calibrated[1], accel_calibrated[2],  # Acceleration data (X, Y, Z)
            gyro_calibrated[0], gyro_calibrated[1], gyro_calibrated[2],     # Gyro data (X, Y, Z)
            temperature                                                    # Temperature
        ]

        # Publish the data
        self.publisher_.publish(imu_msg)

        # Log to console (optional)
        self.get_logger().info(f'Publishing IMU data: Accel({accel_calibrated}), Gyro({gyro_calibrated}), Temp({temperature:.2f} C)')


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass

    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

