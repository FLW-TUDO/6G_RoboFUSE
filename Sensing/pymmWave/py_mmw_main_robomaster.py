import serial
import time
import numpy as np
import os
import sys
import signal
import csv, json, time
from datetime import datetime
import atexit
import threading
from queue import Queue
import socket

#from PyQt5 import QtWidgets, QtCore
#import pyqtgraph as pg
#import pyqtgraph.opengl as gl
#from pyqtgraph.Qt import QtGui

from mss.x8_handler import *
from lib.utility import *
from lib.serial_comm import *
from lib.probe import *
from lib.logger import *
from parser_scripts.config_parser import *
from parser_scripts.data_parser import *

# Global running flag
running = True

# Change the configuration file name
#configFileName = 'config/xwr68xxconfig.cfg' #xwr68xx_profile_range.cfg xwr68xxconfig.cfg

config_file = 'config/py_mmw_setup.json'  # Path to your JSON config file
pymmw_setup = read_config(config_file)
#filename = f'Node1_mmw_demo_output_{current_datetime}.txt'

configFileName = pymmw_setup["configFileName"]
visualizer = pymmw_setup["visualizer"]
controlPort_ = pymmw_setup["controlPort"]
dataPort_ = pymmw_setup["dataPort"]
current_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = pymmw_setup["fileName"] + '_' + current_datetime + '.txt'


print("Config File Name:", pymmw_setup["configFileName"])
print("Visualizer:", pymmw_setup["visualizer"])
print("Control Port:", pymmw_setup["controlPort"])
print("Data Port:", pymmw_setup["dataPort"])
#print("File Name:", filename)

# Change the debug variable to use print()
DEBUG = False

# global CLIport, Dataport
# CLIport = None
# Dataport = None
CLIport = {}
Dataport = {}
detObj = {}  
frameData = {}    
currentIndex = 0

#Threading data queue
data_queue = Queue()

#time interval frame in ms
interval = 30

def send_reset_command(prt):
    """Send the resetSystem command to the control port."""
    try:
        prt.write(b'resetSystem\n')
        time.sleep(0.1)
        print("Reset command sent successfully.")
    except Exception as e:
        print(f"Failed to send reset command: {e}")

##--------------- update threading ----------------------------------#
def read_serial_data():
    """Read data from the serial port and put it into the queue."""
    while running:
        try:
            if Dataport and Dataport.in_waiting > 0:
                data = Dataport.read(Dataport.in_waiting)
                if data:
                    data_queue.put(data)
                    #print(f"data_queue: {data}")
        except Exception as e:
            print(f"Error reading serial data: {e}")

#when visualizer is disabled
def read_parse_data(data_parser):
    global detObj
    dataOk = 0

    try:
        while not data_queue.empty():
            data = data_queue.get()
            dataOk, frameNumber, detObj = data_parser.readAndParseData68xx(data)
            if dataOk and data_parser.configParameters["detectedObjects"] and len(detObj["x"]) > 0:
                x = detObj["x"]
                y = detObj["y"]
                z = detObj["z"]
                snr = detObj["snr"]
                #print(f"read_data: {detObj}")
                'TODO: either ADD PUBLISHER TO ROS2 NODE here or in data_parser, please create new class or script under lib folder'
                return dataOk, detObj
    except Exception as e:
        print(f"Error parsing data: {e}")

    return 0, {}

#parse data if visualizer is active
def parse_data(data_parser, data):
    dataOk, frameNumber, detObj = data_parser.readAndParseData68xx(data)
    if dataOk and data_parser.configParameters["detectedObjects"] and len(detObj["x"]) > 0:
        'TODO: either ADD PUBLISHER TO ROS2 NODE here or in data_parser, please create new class or script under lib folder'
        return dataOk, detObj
    return 0, {}

def send_detObj_to_ISMAC(detObj):
    try:
        # Create a socket connection to the ISMAC controller
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('localhost', 65432))  # Connect to ISMAC server on port 65432
            data = json.dumps(detObj) + '\n'
            message = data.encode('utf-8')  # Encode the message as JSON
            s.sendall(message)  # Send the detObj data
            print(f"detObj sent to client: {detObj}")
            #time.sleep(0.01)
    except ConnectionRefusedError:
        print("Could not connect to the clients. Is it running?")

# def send_detObj_to_ISMAC(detObj):
#     """
#     Sends radar data to the ROS2 receiver via socket. Retries if the connection fails.
#     """
#     retry_count = 0
#     max_retries = 10  # Configure maximum retries

#     while retry_count < max_retries:
#         try:
#             with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#                 s.connect(('localhost', 65432))  # Connect to ROS2 client
#                 data = json.dumps(detObj) + '\n'  # Add newline to signal end of message
#                 s.sendall(data.encode('utf-8'))  # Send the radar data
#                 print(f"Sent radar data: {detObj}")
#             break  # Exit the loop after successful transmission

#         except ConnectionRefusedError:
#             retry_count += 1
#             print(f"Client node not available. Retrying {retry_count}/{max_retries}...")
#             time.sleep(1)  # Wait 1 second before retrying

#         except Exception as e:
#             print(f"Error while sending data: {e}")
#             break

##----------------end of update -------------------------------------#

def signal_handler(sig, frame):
    global running
    running = False
    print("Ctrl+C pressed. Exiting...")
    if CLIport.is_open:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        print("signal handler: Send Sensor Stop and CLIPort is closed!")
    if Dataport.is_open:
        Dataport.close()
        print("signal handler: Data Port is closed!")
    #if visualizer:
        #QtWidgets.QApplication.quit()
    sys.exit(0)

def close_ports():
    global CLIport, Dataport 
    if isinstance(CLIport, serial.Serial) and CLIport.is_open:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
    if isinstance(Dataport, serial.Serial) and Dataport.is_open:
        Dataport.close()

atexit.register(close_ports)

def main():
    global configParameters
    global CLIport, Dataport
    # CLIport = None
    # Dataport = None
    signal.signal(signal.SIGINT, signal_handler)  # Set up the signal handler
    nrst = True
    
    frameData = {}    
    currentIndex = 0
    # Main loop 
    try:
        print(f"Serial: {controlPort_}, {dataPort_}, {configFileName}")
        # Configurate the serial port
        CLIport, Dataport = serialConfig(configFileName, controlPort_, dataPort_)

        # Get the configuration parameters from the configuration file
        configParameters = parseConfigFile(configFileName)

        data_parser = DataParser(configParameters, filename)
        
        #Reset for the first time
        # ---
        nrst = True
        print("nrst mode 2: ", nrst)

        if nrst:
            send_reset_command(CLIport)
        else:
            print('\nwaiting for reset (NRST) of the device', file=sys.stderr, flush=True)
        
        time.sleep(1.0)
        print("wait after reset")

        ## new update with thread
        read_thread = threading.Thread(target=read_serial_data)
        read_thread.start()

        while running:
            # Update the data and check if the data is okay
            dataOk, detObj = read_parse_data(data_parser)
            if dataOk:
                send_detObj_to_ISMAC(detObj)
            #print("detObj main: ", detObj)
            time.sleep(interval/1000) # Sampling frequency of 30 Hz

        read_thread.join()

    except Exception as e: 
        #KeyboardInterrupt:
        #if isinstance(CLIport, serial.Serial) and CLIport.is_open:
        if CLIport.is_open:
            CLIport.write(('sensorStop\n').encode())
            CLIport.close()
            print("Exception: Send Sensor Stop and CLIPort is closed!")
        #if isinstance(Dataport, serial.Serial) and Dataport.is_open:
        if Dataport.is_open:
            Dataport.close()
        print("Exception: Data Port is closed!")

    finally:
        if 'CLIport' in globals() and CLIport.is_open:
        #if isinstance(CLIport, serial.Serial) and CLIport.is_open:
            CLIport.write(('sensorStop\n').encode())
            print("finally Send Sensor Stop!")
            CLIport.close()
        #if isinstance(Dataport, serial.Serial) and Dataport.is_open:
        if 'Dataport' in globals() and Dataport.is_open:
            print("finally Dataport Stop!")
            Dataport.close()   


if __name__ == "__main__":
    main()