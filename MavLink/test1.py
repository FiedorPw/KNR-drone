from pymavlink import mavutil
import os
import json
import time
import math
from datetime import datetime
import shutil
from threading import Thread, Lock
from flask import Flask, jsonify

# Initialize Flask app
app = Flask(__name__)

# Establish connection to the flight controller
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Directory to save logs
log_dir = "/home/KNR/KNR-dron/MavLink/LOGS/"

# Function to create a new log filename
def create_log_filename():
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(log_dir, f"dane_telemetryczne_{current_time}.json")

# Initialize telemetry data and log rotation settings
telemetry_data = {
    "Roll": [],
    "Pitch": [],
    "Yaw": [],
    "Latitude": [],
    "Longitude": [],
    "Altitude": [],
    "HDOP": [],
    "VDOP": [],
    "Satellites": [],
    "Flight_Mode": [],
    "Voltage": [],
    "Current": [],
    "Armed": []
}

# Lock for thread-safe file operations
file_lock = Lock()

# Track number of saves and the current log filename
save_count = 0
log_filename = create_log_filename()

# Function to save telemetry data to JSON file
def save_to_json():
    global save_count, log_filename
    temp_filename = log_filename + '.temp'
    with file_lock:
        with open(temp_filename, 'w') as temp_file:
            json.dump(telemetry_data, temp_file, indent=4)
        shutil.move(temp_filename, log_filename)
    
    save_count += 1
    if save_count >= 5:
        # Reset save count and create a new log file
        save_count = 0
        log_filename = create_log_filename()
        # Reset telemetry_data for the new file
        reset_telemetry_data()

# Function to reset telemetry data
def reset_telemetry_data():
    global telemetry_data
    telemetry_data = {
        "Roll": [],
        "Pitch": [],
        "Yaw": [],
        "Latitude": [],
        "Longitude": [],
        "Altitude": [],
        "HDOP": [],
        "VDOP": [],
        "Satellites": [],
        "Flight_Mode": [],
        "Voltage": [],
        "Current": [],
        "Armed": []
    }

# Function to read telemetry data from JSON file
def read_from_json():
    with file_lock:
        with open(log_filename, 'r') as json_file:
            return json.load(json_file)

def get_gps_position():
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3
        hdop = msg.eph / 100.0
        vdop = msg.epv / 100.0
        satellites_visible = msg.satellites_visible
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters, HDOP: {hdop}, VDOP: {vdop}, Satellites: {satellites_visible}")
        return {
            "Latitude": lat,
            "Longitude": lon,
            "Altitude": alt,
            "HDOP": hdop,
            "VDOP": vdop,
            "Satellites": satellites_visible
        }
    else:
        print("No GPS position data received")
        return None

def get_attitude():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw = math.degrees(msg.yaw)
        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
        return {
            "Roll": roll,
            "Pitch": pitch,
            "Yaw": yaw
        }
    else:
        print("No attitude data received")
        return None

def get_battery_status():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
    if msg:
        voltage = msg.voltages[0] / 1000.0
        current = msg.current_battery / 100.0
        print(f"Voltage: {voltage}V, Current: {current}A")
        return {
            "Voltage": voltage,
            "Current": current
        }
    else:
        print("No battery status data received")
        return None

def get_flight_mode():
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        print(f"Current flight mode: {mode}")
        return mode
    return None

def get_arm_status():
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                              mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                              0, 0, 0)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print("Drone is armed")
            return True
        else:
            print("Drone is disarmed")
            return False
    else:
        print("No heartbeat message received")
        return False

def get_all_telemetry():
    attitude_data = get_attitude()
    if attitude_data:
        telemetry_data["Roll"].append(attitude_data["Roll"])
        telemetry_data["Pitch"].append(attitude_data["Pitch"])
        telemetry_data["Yaw"].append(attitude_data["Yaw"])

    gps_data = get_gps_position()
    if gps_data:
        telemetry_data["Latitude"].append(gps_data["Latitude"])
        telemetry_data["Longitude"].append(gps_data["Longitude"])
        telemetry_data["Altitude"].append(gps_data["Altitude"])
        telemetry_data["HDOP"].append(gps_data["HDOP"])
        telemetry_data["VDOP"].append(gps_data["VDOP"])
        telemetry_data["Satellites"].append(gps_data["Satellites"])

    flight_mode = get_flight_mode()
    if flight_mode:
        telemetry_data["Flight_Mode"].append(flight_mode)

    battery_data = get_battery_status()
    if battery_data:
        telemetry_data["Voltage"].append(battery_data["Voltage"])
        telemetry_data["Current"].append(battery_data["Current"])

    armed_status = get_arm_status()
    telemetry_data["Armed"].append(armed_status)

# Flask route to serve telemetry data
@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    try:
        data = read_from_json()
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

def telemetry_collection():
    print("Requesting data streams...")
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    print("Getting telemetry data...")

    while True:
        get_all_telemetry()
        save_to_json()
        print(f"Telemetry data saved to {log_filename}")
        time.sleep(0.5)  # Delay for 0.5 seconds

if __name__ == "__main__":
    # Start the telemetry collection in a separate thread
    telemetry_thread = Thread(target=telemetry_collection, daemon=True)
    telemetry_thread.start()

    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)
