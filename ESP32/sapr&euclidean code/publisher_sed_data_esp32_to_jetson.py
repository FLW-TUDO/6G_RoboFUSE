import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

# Configure the serial connection with error handling
try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',  
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1  # Timeout for reading
    )
    if ser.isOpen():
        print(f"Serial port {ser.portstr} is open")
    else:
        raise serial.SerialException(f"Failed to open serial port {ser.portstr}")
except serial.SerialException as e:
    print(f"Serial connection error: {e}")
    exit(1)

class SerialDataPublisher(Node):
    def __init__(self):
        super().__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(String, 'serial_raw_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Check if there is data waiting in the buffer
            if ser.inWaiting() > 0:
                # Attempt to read data from the serial port
                raw_data = ser.read(ser.inWaiting()).decode('utf-8').strip()

                # Filter out non-printable characters or junk values (empty strings, newlines, or carriage returns)
                if raw_data and raw_data.isprintable():  # Check if data is printable
                    # Optionally use regex to ensure data matches your expected pattern
                    if re.match(r"^[a-zA-Z0-9\s]+$", raw_data):  # Example: allow only alphanumeric and spaces
                        self.get_logger().info(f"Valid data: {raw_data}")  # Log the valid data
                        
                        # Publish the valid data to the ROS 2 topic
                        msg = String()
                        msg.data = raw_data
                        self.publisher_.publish(msg)
                    else:
                        self.get_logger().warn(f"Filtered out data due to invalid characters: {raw_data}")
                else:
                    self.get_logger().warn("Received non-printable or empty data, ignoring")
        except serial.SerialTimeoutException:
            self.get_logger().error("Timeout occurred while reading serial data")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Decoding error: {e} - Invalid data received, ignoring")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
    
    def shutdown(self):
        try:
            ser.close()  # Close the serial port safely
            self.get_logger().info("Serial port closed")
        except serial.SerialException as e:
            self.get_logger().error(f"Error closing serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_data_publisher = SerialDataPublisher()

    try:
        rclpy.spin(serial_data_publisher)
    except KeyboardInterrupt:
        serial_data_publisher.get_logger().info("Program interrupted by user")
    finally:
        serial_data_publisher.shutdown()  # Gracefully shutdown
        serial_data_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



