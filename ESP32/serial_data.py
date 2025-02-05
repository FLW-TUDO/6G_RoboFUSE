#!/usr/bin/env python
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Thread, Lock
import signal
import json, os

#Open the serial port
ser = serial.Serial('/dev/ttyTHS1')
sapr_file_path = os.path.join(os.getcwd(), '..', 'sapr_data.json')  # Path to SAPR JSON file

def signal_handler(sig,frame):
    print('You pressed Ctrl+C!')
    if ser.is_open:
        print('Closing serial port')
        ser.close()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

class UARTSender(Node):
    def __init__(self, serial_port):
        super().__init__('uart_sender_node')
        self.subscription = self.create_subscription(
            Float32,
            '/relative_distance',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.serial_port = serial_port
        self.lock = Lock()
        self.last_sent_time = 0  # Track the last time data was sent

    def send_relevant_sapr_data(self):
        """
        Select 'timestamp', 'received_distance', and 'range' from 'radar' in sapr_data.json 
        and send them over the serial port.
        """
        if os.path.exists(sapr_file_path):
            # Read the SAPR data from the JSON file
            with open(sapr_file_path, 'r') as file:
                sapr_data = json.load(file)

                # Create a subset of the SAPR data with only relevant keys
                relevant_sapr_data = {}

                # Extract 'timestamp' and 'received_distance' if they exist
                if 'timestamp' in sapr_data:
                    relevant_sapr_data['timestamp'] = sapr_data['timestamp']
                if 'received_distance' in sapr_data:
                    relevant_sapr_data['received_distance'] = sapr_data['received_distance']

                # Extract the 'range' key from the 'radar' object if it exists
                if 'radar' in sapr_data and 'range' in sapr_data['radar']:
                    relevant_sapr_data['range'] = sapr_data['radar']['range']

                # Convert the subset to a JSON string
                sapr_message = json.dumps(relevant_sapr_data)

                # Send the filtered SAPR data over the serial port if it's open
                if self.serial_port.is_open:
                    self.serial_port.write(sapr_message.encode())
                    print(f"Sent relevant SAPR data: {sapr_message}")
                else:
                    print("Serial port is not open, unable to send relevant SAPR data.")
        else:
            print("SAPR data file not found.")
    
    def listener_callback(self, msg):
        distance = msg.data
        timestamp = time.time()
        message = f"{timestamp:.3f},{distance:.2f}\n"  # Include timestamp and distance
        with self.lock:
            self.serial_port.write(message.encode())
        print(f"Listener Callback Sent: {message.strip()}")

    
    # def listener_callback(self, msg):
    #     distance = msg.data
    #     timestamp = time.time()
    #     message = f"{timestamp:.3f},{distance:.2f}\n"  # Include timestamp and distance
        # with self.lock:
        #     if os.path.exists(sapr_file_path):
        #         # Read the SAPR data from the JSON file
        #         with open(sapr_file_path, 'r') as file:
        #             sapr_data = json.load(file)

        #             # Create a subset of the SAPR data with only relevant keys
        #             relevant_sapr_data = {}

        #             # Extract 'timestamp' and 'received_distance' if they exist
        #             if 'timestamp' in sapr_data:
        #                 relevant_sapr_data['timestamp'] = sapr_data['timestamp']
        #             if 'received_distance' in sapr_data:
        #                 relevant_sapr_data['received_distance'] = sapr_data['received_distance']

        #             # Extract the 'range' key from the 'radar' object if it exists
        #             if 'radar' in sapr_data and 'range' in sapr_data['radar']:
        #                 relevant_sapr_data['range'] = sapr_data['radar']['range']

        #             # Convert the subset to a JSON string
        #             sapr_message = json.dumps(relevant_sapr_data)

        #             # Send the filtered SAPR data over the serial port if it's open
        #             if self.serial_port.is_open:
        #                 self.serial_port.write(sapr_message.encode())
        #                 print(f"Sent relevant SAPR data: {sapr_message}")
        #             else:
        #                 print("Serial port is not open, unable to send relevant SAPR data.")
        #     else:
        #         print("SAPR data file not found.")

    def periodic_send(self):
        # This method is called periodically every 100ms
        while rclpy.ok():
            with self.lock:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        try:
                            timestamp, received_distance = data.split(',', 1)
                            print(f"{time.time()} Received: Timestamp: {timestamp}, Distance: {received_distance}")
                        except ValueError:
                            print(f"{time.time()} Received malformed data: {data}")
            time.sleep(0.05)  # Sleep

def ros2_thread():
    rclpy.init()
    uart_sender = UARTSender(esp)
    rclpy.spin(uart_sender)
    uart_sender.destroy_node()
    rclpy.shutdown()

# Initialize the serial connection
esp = serial.Serial(
    port="/dev/ttyTHS1",  # Use the ttyTHS1 port
    baudrate=115200,      # Baud rate should match the external device's
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,
    write_timeout=2,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False
)

# Wait a second to let the port initialize
time.sleep(1)

# Start the ROS2 thread
ros2_thread_instance = Thread(target=ros2_thread)
ros2_thread_instance.start()

# Start the periodic send thread
periodic_thread = Thread(target=UARTSender(esp).periodic_send)
periodic_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("Exiting Program")
finally:
    if esp.is_open:
        esp.write(b"TERMINATE\n")  # Custom termination message
        time.sleep(0.5)  # Allow time for the message to be sent
        esp.close()

