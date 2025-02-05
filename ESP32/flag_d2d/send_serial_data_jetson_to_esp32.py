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
                    #if data:
                        #print(f"{time.time()} Received: {data}")
            time.sleep(0.1)  # Sleep for 100ms

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
        esp.close()


