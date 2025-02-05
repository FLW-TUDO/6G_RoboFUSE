#!/usr/bin/env python

import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Thread, Lock

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

    def listener_callback(self, msg):
        distance = msg.data
        message = f"{distance:.2f}\n"  # Only include distance
        with self.lock:
            self.serial_port.write(message.encode())
        print(f"Sent: {message.strip()}")  # Print the sent message without extra timestamp

    def periodic_send(self):
        # This method is called periodically every 100ms
        while rclpy.ok():
            with self.lock:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        print(f"Received: {data}")  # You can process data here if needed
            time.sleep(0.1)  # Sleep for 100ms

class USBReceiver(Node):
    def __init__(self, usb_port):
        super().__init__('usb_receiver_node')
        self.publisher_ = self.create_publisher(Float32, '/usb_data', 10)
        self.usb_port = usb_port
        self.lock = Lock()

    def receive_and_publish(self):
        # This method reads data from USB port and publishes it on the /usb_data topic
        while rclpy.ok():
            with self.lock:
                if self.usb_port.in_waiting > 0:
                    try:
                        data = self.usb_port.readline().decode().strip()
                        if data:
                            received_value = float(data)  # Assuming the incoming data is float
                            msg = Float32()
                            msg.data = received_value
                            self.publisher_.publish(msg)
                            print(f"Published: {received_value}")
                    except ValueError:
                        print("Received invalid data, could not convert to float")
            time.sleep(0.1)  # Sleep for 100ms

def ros2_sender_thread():
    rclpy.init()
    uart_sender = UARTSender(esp)
    rclpy.spin(uart_sender)
    uart_sender.destroy_node()
    rclpy.shutdown()

def ros2_receiver_thread():
    rclpy.init()
    usb_receiver = USBReceiver(usb)
    usb_receiver.receive_and_publish()
    usb_receiver.destroy_node()
    rclpy.shutdown()

# Initialize the serial connection for UART (TTYTHS1)
esp = serial.Serial(
    port="/dev/ttyUSB1",  # Use the ttyTHS1 port
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

# Initialize the serial connection for USB (TTYUSB0)
usb = serial.Serial(
    port="/dev/ttyUSB0",  # Use the ttyUSB0 port
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

# Wait a second to let the ports initialize
time.sleep(1)

# Start the ROS2 sender thread (handles sending data over UART)
ros2_sender_thread_instance = Thread(target=ros2_sender_thread)
ros2_sender_thread_instance.start()

# Start the ROS2 receiver thread (handles receiving data from USB and publishing to ROS2 topic)
ros2_receiver_thread_instance = Thread(target=ros2_receiver_thread)
ros2_receiver_thread_instance.start()

# Start the periodic send thread for UART
periodic_thread = Thread(target=UARTSender(esp).periodic_send)
periodic_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main thread alive
except KeyboardInterrupt:
    print("Exiting Program")
finally:
    if esp.is_open:
        esp.close()
    if usb.is_open:
        usb.close()

