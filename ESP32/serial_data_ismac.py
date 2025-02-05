#!/usr/bin/env python
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Thread, Lock
import signal
import json
import os, sys
import numpy as np
from datetime import datetime

# Path for saving received distance data
received_distance_file_path = 'received_distance.json'
current_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
log_file = 'log/' + 'ESP32_data_' + current_datetime + '.txt'
sapr_file_path = os.path.join(os.getcwd(), '..', 'sapr_data.json')  # Path to SAPR JSON file

# Initialize the serial connection
ser = serial.Serial(
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

# Signal handler to safely close the serial port
def signal_handler(sig, frame):
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
        self.subscription  # Prevent unused variable warning
        self.serial_port = serial_port
        self.lock = Lock()
        self.last_sent_time = 0  # Track the last time data was sent
        self.received_distance = 0.0  # Store the distance received from another robot


    # TODO: need to be checked
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

                size = sys.getsizeof(relevant_sapr_data)

                relevant_sapr_data["bs"] = size

                # Convert the subset to a JSON string
                sapr_message = json.dumps(relevant_sapr_data)


                # Append the data to the text current_time_slot, radar_slot, esp32_slot file, one JSON object per line
                with open(log_file, 'a') as file:
                    file.write(json.dumps(sapr_message) + '\n')
                print(f"Time sync data saved to {log_file}")

                # Send the filtered SAPR data over the serial port if it's open
                if self.serial_port.is_open:
                    self.serial_port.write(sapr_message.encode())
                    print(f"Sent relevant SAPR data: {sapr_message}")
                else:
                    print("Serial port is not open, unable to send relevant SAPR data.")
        else:
            print("SAPR data file not found.")


    def listener_callback(self, msg):
        """
        Callback when new relative distance is published. This will be the SAPR data that
        we receive from the ISMAC script via ROS2.
        """
        distance = msg.data
        timestamp = time.time()
        message = f"{timestamp:.3f},{distance:.2f}\n"  # Include timestamp and distance
        # Save received distance to JSON file
        self.received_distance = np.round(distance, 4)  # Store distance in the object
        with open(received_distance_file_path, 'w') as file:
            json.dump({"d": self.received_distance}, file)
        #print(f"Sent: {message.strip()}")  
        print(f"Saved received distance {self.received_distance}")
        
        # with self.lock:
        #     self.serial_port.write(message.encode())  # Send to ESP32
        # print(f"Sent: {message.strip()}")
        with self.lock:
            # Transmission Mode: Read the SAPR data and send to ESP32
            try:
                if os.path.exists(sapr_file_path):
                    # Read the SAPR data from the JSON file
                    with open(sapr_file_path, 'r') as file:
                        #timestamp = time.time()
                        sapr_data = json.load(file)
                        #sapr_data["ch_time"] = timestamp

                        sapr_message = json.dumps(sapr_data)  # Convert the SAPR data to JSON string

                        # Send SAPR data over the serial port if it's open
                        if self.serial_port and self.serial_port.is_open:
                            
                            self.serial_port.write(sapr_message.encode())
                            print(f"Callback - Sent SAPR to ESP32")

                            # Log the timestamped SAPR data to the log file
                            with open(log_file, 'a') as log:
                                log_entry = {
                                    "timestamp": timestamp,
                                    "sapr_message": sapr_data
                                }
                                log.write(json.dumps(log_entry) + '\n')

                            # Append the data to the text current_time_slot, radar_slot, esp32_slot file, one JSON object per line
                            # with open(log_file, 'a') as file:
                            #     file.write(json.dumps(sapr_message) + '\n')
                            # print(f"Time sync data saved to {log_file}")

                            #print(f"Callback - Sent SAPR to ESP32: {sapr_message.strip()}")
                        else:
                            print(f"Serial port not open. Simulated sending SAPR: {sapr_message.strip()}")
                else:
                    print("SAPR data file not found for transmission.")

            except Exception as e:
                print(f"Error during SAPR transmission: {e}")

    def periodic_send(self):
        """
        Periodically switches between transmission and receiving modes.

        Transmission Mode:
        - Read SAPR data from the JSON file and send it to ESP32.

        Receiving Mode:
        - Check for incoming data from ESP32, including timestamp, radar data, and received_distance.
        """
        while rclpy.ok():
            with self.lock:

                # Receiving Mode: Check for incoming data from ESP32

                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        try:
                            # Assuming the received data is in the format: timestamp, radar_data, received_distance
                            parts = data.split(',')
                            if len(parts) == 3:
                                timestamp, radar_data, received_distance = parts
                                print(f"Received Data - Timestamp: {timestamp}, Radar: {radar_data}, Distance: {received_distance}")

                                # Process or store the received data as needed
                                # Save received distance to JSON file
                                self.received_distance = float(received_distance)  # Update the received distance
                                with open(received_distance_file_path, 'w') as file:
                                    json.dump({"received_distance": self.received_distance}, file)
                                    print(f"Saved received distance to {received_distance_file_path}")

                            else:
                                print(f"Malformed incoming data: {data}")
                        except ValueError as e:
                            print(f"Error processing received data: {e}")


            # Sleep for a short interval between checks to avoid CPU overload
            time.sleep(0.05)
    
    # def periodic_send(self):
    #     """
    #     Periodically checks for SAPR data in the JSON file and sends it to ESP32.
    #     Also checks for incoming data from the serial port and processes it.
    #     """
    #     sapr_file_path = os.path.join(os.getcwd(), '..', 'sapr_data.json')  # Path to SAPR JSON file

    #     while rclpy.ok():
    #         with self.lock:
    #             # 1. Send SAPR data to ESP32
    #             if os.path.exists(sapr_file_path):
    #                 # Read the SAPR data from the JSON file
    #                 with open(sapr_file_path, 'r') as file:
    #                     sapr_data = json.load(file)
    #                     sapr_message = json.dumps(sapr_data)  # Convert the SAPR data to JSON string

    #                     # Send SAPR data over the serial port if it's open
    #                     if self.serial_port and self.serial_port.is_open:
    #                         self.serial_port.write(sapr_message.encode())
    #                         print(f"Sent SAPR to ESP32: {sapr_message.strip()}")
    #                     else:
    #                         print(f"Simulated sending SAPR: {sapr_message.strip()}")
    #             else:
    #                 print("SAPR data file not found.")

    #             # 2. Check for incoming data from ESP32
    #             if self.serial_port.in_waiting > 0:
    #                 data = self.serial_port.readline().decode().strip()
    #                 if data:
    #                     try:
    #                         # Assuming data format: timestamp, distance
    #                         timestamp, received_distance = data.split(',', 1)
    #                         self.received_distance = float(received_distance)  # Store the received distance
    #                         print(f"{time.time()} Received: Timestamp: {timestamp}, Distance: {received_distance}")

    #                         # Save received distance to JSON file
    #                         with open(received_distance_file_path, 'w') as file:
    #                             json.dump({"received_distance": self.received_distance}, file)
    #                             print(f"Saved received distance to {received_distance_file_path}")

    #                     except ValueError:
    #                         print(f"{time.time()} Received malformed data: {data}")
            
    #         # Sleep for a short interval between checks to avoid CPU overload
    #         time.sleep(0.05)


def ros2_thread():
    """
    This is the ROS2 node that listens for SAPR data (relative distance) published by ISMAC.
    """
    rclpy.init()
    uart_sender = UARTSender(ser)
    rclpy.spin(uart_sender)  # Keep the ROS2 node running
    uart_sender.destroy_node()
    rclpy.shutdown()

# Initialize the serial connection
ser = serial.Serial(
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


# Wait a second to let the serial port initialize
time.sleep(1)

# Start the ROS2 thread
ros2_thread_instance = Thread(target=ros2_thread)
ros2_thread_instance.start()

# Start the periodic send thread (this checks for incoming serial data)
periodic_thread = Thread(target=UARTSender(ser).periodic_send)
periodic_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("Exiting Program")
finally:
    if ser.is_open:
        ser.write(b"TERMINATE\n")  # Custom termination message
        time.sleep(0.5)  # Allow time for the message to be sent
        ser.close()
