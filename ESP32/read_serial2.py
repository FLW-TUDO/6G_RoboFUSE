import serial
import json
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configure the serial connection
ser = serial.Serial(
    port='/dev/ttyUSB2',  
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1  # Timeout for reading
)

# Ensure the serial port is opened
if ser.isOpen():
    print(f"Serial port {ser.portstr} is open")
else:
    print(f"Failed to open serial port {ser.portstr}")

# Directory and file setup
workspace_directory = os.path.expanduser("~/Received_data/serial_data")
os.makedirs(workspace_directory, exist_ok=True)
json_file_path = os.path.join(workspace_directory, "data.json")

# Initialize the data list
data_list = []

class SerialDataPublisher(Node):
    def __init__(self):
        super().__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(String, 'serial_raw_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if ser.inWaiting() > 0:
            # Read data from the serial port
            raw_data = ser.read(ser.inWaiting()).decode('utf-8').strip()
            self.get_logger().info(f"Raw data: {raw_data}")  # Log the raw data
            
            # Publish the raw data to the ROS 2 topic
            msg = String()
            msg.data = raw_data
            self.publisher_.publish(msg)
            
            # Parse the JSON data
            try:
                json_data = json.loads(raw_data)
                message_value = json_data.get("message")
                self.get_logger().info(f"Parsed message value: {message_value}")  # Log the parsed message value
                
                # Create a data entry with a timestamp
                data_entry = {
                    #"timestamp": datetime.now().isoformat(),
                    "value": message_value
                }
                data_list.append(data_entry)
                
                # Write the data to the JSON file
                with open(json_file_path, 'w') as json_file:
                    json.dump(data_list, json_file, indent=4)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSONDecodeError: {e} - Invalid JSON data: {raw_data}")

def main(args=None):
    rclpy.init(args=args)
    serial_data_publisher = SerialDataPublisher()

    try:
        rclpy.spin(serial_data_publisher)
    except KeyboardInterrupt:
        serial_data_publisher.get_logger().info("Program interrupted by user")
    finally:
        ser.close()
        serial_data_publisher.get_logger().info("Serial port closed")
        serial_data_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

