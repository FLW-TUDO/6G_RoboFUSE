import time
import json
import socket
import os, sys
import numpy as np
from datetime import datetime
#import matplotlib.pyplot as plt

# Initialize lists to store time slots for sensing and communication
time_slots = []
sensing_slots = []
communication_slots = []

# Path for the JSON file to transfer data
#sapr_file_path = os.path.join(os.getcwd(), 'sapr_data.json')
sapr_file_path = 'sapr_data.json'  # SAPR data saved here
received_distance_file_path = 'ESP32/received_distance.json'  # Distance data from ESP32 saved here
current_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
time_sync_file = 'ISMAC/measurement/' + 'ISMAC_data_' + current_datetime + '.txt'

def save_time_sync_data(radar_timestamp, communication_timestamp, current_time_slot, sensing_slot, communication_slot, radar_freq, esp32_freq, sapr_data):
    """
    Save the time synchronization data (slots, frequencies, timestamps) to a text file.
    """
    sync_data = {
        "s_time": radar_timestamp,
        "c_time": communication_timestamp,
        "ts": current_time_slot,
        "s_slot": sensing_slot,
        "c_slot": communication_slot,
        "s_freq": radar_freq,
        "c_freq": esp32_freq,
        "sapr": sapr_data
    }

    # Append the data to the text current_time_slot, radar_slot, esp32_slot file, one JSON object per line
    with open(time_sync_file, 'a') as file:
        file.write(json.dumps(sync_data) + '\n')
    print(f"Time sync data saved to {time_sync_file}")

def start_socket_server():
    """
    Set up the socket server to receive data from the radar script.
    Handles large data by reading until the message is complete (uses newline delimiter).
    Also handles socket reuse to avoid 'Address already in use' errors.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse of the socket address
        s.bind(('localhost', 65432))  # Bind to localhost on port 65432
        s.listen()  # Start listening for incoming connections
        print("Waiting for radar data...")

        while True:
            conn, addr = s.accept()  # Accept an incoming connection
            with conn:
                print(f"Connected by {addr}")
                buffer = ''
                while True:
                    data = conn.recv(8192).decode('utf-8') # Keep receiving data in chunks
                    #data = conn.recv(128).decode('utf-8') 
                    #print(data)  
                    if not data:
                        break  # Connection closed by client
                    buffer += data  # Append received data to the buffer
                    if '\n' in buffer:  # Check if we received the full message
                        break

                try:
                    # Split the buffer and extract the first valid JSON message
                    # print('Buffer: ',buffer)
                    json_message = buffer.split('\n', 1)[0]
                    detObj = json.loads(json_message)  # Decode the JSON data
                    if detObj:  # Check if detObj is not empty
                        print(f"Received detObj data")
                        return detObj
                    else:
                        print("Received empty detObj. Retrying...")
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON: {e}. Retrying...")
                except Exception as e:
                    print(f"Unexpected error: {e}. Retrying...")

            #time.sleep(1)  # Retry after a short delay if no data is received


def read_received_distance():
    """
    Read the received distance from ESP32 (from received_distance.json).
    """
    if os.path.exists(received_distance_file_path):
        try:
            with open(received_distance_file_path, 'r') as file:
                data = json.load(file)
                received_distance = data.get('d', 0.0)
                print(f"Received distance from ESP32: {received_distance}")
                return received_distance
        except json.JSONDecodeError:
            print("Error decoding JSON from received_distance.json. File might be empty.")
            return 0.0
    else:
        print("No received distance data found.")
        return 0.0


def calculate_dynamic_frequency(last_timestamp):
    """
    Calculate dynamic frequency based on timestamp differences.
    """
    current_timestamp = time.time()
    time_diff = current_timestamp - last_timestamp
    if time_diff > 0:
        return 1 / time_diff, current_timestamp
    return 0, current_timestamp

def allocate_time_slots(radar_freq, esp32_freq):
    """
    Allocate time slots for sensing and communication based on their frequencies.
    """
    radar_slot = 1 / radar_freq if radar_freq > 0 else 0
    esp32_slot = 1 / esp32_freq if esp32_freq > 0 else 0

    # Add the time slots to the lists for visualization
    #current_time_slot = time.time()
    # time_slots.append(time.time())  # Record current time for X-axis
    # sensing_slots.append(radar_slot)  # Record sensing time slot for Y-axis
    # communication_slots.append(esp32_slot)  # Record communication time slot for Y-axis

    print(f"Allocated Sensing Slot: {radar_slot:.4f} seconds")
    print(f"Allocated Communication Slot: {esp32_slot:.4f} seconds")

    return radar_slot, esp32_slot

def save_sapr_to_file(sapr_data):
    """
    Save the SAPR data to a JSON file to be accessed by serial_data.py.
    """
    sapr_data["bs"] = sys.getsizeof(sapr_data)

    with open(sapr_file_path, 'w') as file:
        json.dump(sapr_data, file)
    print(f"SAPR data saved to {sapr_file_path}")


def time_division_isac_handler():
    """
    Orchestrate time division and dynamically allocate time slots for sensing and communication.
    """
    current_time_slot = time.time()
    last_radar_timestamp = time.time()
    last_esp32_timestamp = time.time()

    while True:
        # 1. Radar Sensing Phase
        print("\n--- Sensing Phase ---")
        radar_data = start_socket_server()  # Get radar data (detObj)
        if radar_data:
            radar_timestamp = last_radar_timestamp
            radar_freq, last_radar_timestamp = calculate_dynamic_frequency(last_radar_timestamp)  # Calculate radar frequency
            print(f"Radar Frequency: {radar_freq:.2f} Hz")


        # 2. ESP32 Communication Phase
        print("\n--- Communication Phase ---")
        esp32_distance = read_received_distance()  # Read distance from ESP32
        esp32_freq, last_esp32_timestamp = calculate_dynamic_frequency(last_esp32_timestamp)  # Calculate ESP32 frequency
        print(f"ESP32 Frequency: {esp32_freq:.2f} Hz")

        # 3. Allocate Time Slots
        radar_slot, esp32_slot = allocate_time_slots(radar_freq, esp32_freq)
        communication_timestamp = last_radar_timestamp + radar_slot

        # 4. Combine Radar and ESP32 Data (Generate SAPR)
        sapr_timestamp = time.time()
        sapr_data = {"t": sapr_timestamp,"radar": radar_data, "d": esp32_distance}

        # 5. Save SAPR to JSON file for serial_data.py to read
        save_sapr_to_file(sapr_data)

        # 6. Save time synchronization data to a text file
        save_time_sync_data(radar_timestamp, communication_timestamp, current_time_slot, radar_slot, esp32_slot, radar_freq, esp32_freq, sapr_data)

        # Add a small sleep to avoid overloading the CPU
        time.sleep(0.01)


if __name__ == "__main__":
    try:
        time_division_isac_handler()  # Start the time division handler
    except KeyboardInterrupt:
        print("Program terminated.")
