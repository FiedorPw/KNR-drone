from pymavlink import mavutil
import os
import json
import time
import math
from datetime import datetime
import shutil
from threading import Thread, Lock
from flask import Flask, jsonify
from flask_cors import CORS

class FC_Controller:
    def __init__(self, connection_string='/dev/ttyACM0', baud_rate=57600, log_dir="/home/KNR/LOGS/"):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        self.log_dir = log_dir
        # Lock for thread-safe file operations
        self.file_lock = Lock()
        self.save_count = 0
        # Lock for attitude data
        self.attitude_lock = Lock()
        self.telemetry_data = self.reset_telemetry_data()
        self.log_filename = self.create_log_filename()

        # Start the connection by waiting for heartbeat
        self._wait_for_heartbeat()

        # Set arming check parameter to 0 - disable all arming flags
        self.set_param('ARMING_CHECK', 0)

        # Start Flask web app in a separate thread
        self.app = Flask(__name__)
        CORS(self.app)
        self.setup_flask_routes()
        self.flask_thread = Thread(target=self.start_flask, daemon=True)
        self.flask_thread.start()

        # Start telemetry collection in a separate thread
        self.telemetry_thread = Thread(target=self.telemetry_collection, daemon=True)
        self.telemetry_thread.start()

    # Waits for a heartbeat message from the flight controller
    def _wait_for_heartbeat(self):
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

    # Creates a new log filename with the current timestamp
    def create_log_filename(self):
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return os.path.join(self.log_dir, f"dane_telemetryczne_{current_time}.json")

    # Resets the telemetry data structure
    def reset_telemetry_data(self):
        return {
            "Timestamp": [],  # Added Timestamp field
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

    # Saves telemetry data to a JSON file
    def save_to_json(self):
        temp_filename = self.log_filename + '.temp'
        with self.file_lock:
            with open(temp_filename, 'w') as temp_file:
                json.dump(self.telemetry_data, temp_file, indent=4)
            shutil.move(temp_filename, self.log_filename)

        self.save_count += 1
        if self.save_count >= 5:
            self.save_count = 0
            self.log_filename = self.create_log_filename()
            self.telemetry_data = self.reset_telemetry_data()

    def set_param(self, param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        # Przesyłanie komendy do zmiany parametru
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )

    # Retrieves GPS position data
    def get_gps_position(self):
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            hdop = msg.eph / 100.0
            vdop = msg.epv / 100.0
            satellites_visible = msg.satellites_visible
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

    # Retrieves attitude data (roll, pitch, yaw)
    def get_attitude(self):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            return {
                "Roll": roll,
                "Pitch": pitch,
                "Yaw": yaw
            }
        else:
            print("No attitude data received")
            return None

    # Retrieves battery status data
    def get_battery_status(self):
        msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True)
        if msg:
            voltage = msg.voltages[0] / 1000.0
            current = msg.current_battery / 100.0
            return {
                "Voltage": voltage,
                "Current": current
            }
        else:
            print("No battery status data received")
            return None

    # Retrieves the current flight mode
    def get_flight_mode(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            return mode
        return None

    # Retrieves the arm status of the drone
    def get_arm_status(self):
        self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                       mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                       0, 0, 0)
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            return bool(armed)
        else:
            print("No heartbeat message received")
            return False

    # Collects all telemetry data and appends to the telemetry data structure
    def get_all_telemetry(self):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.telemetry_data["Timestamp"].append(timestamp)

        attitude_data = self.get_attitude()
        if attitude_data:
            self.telemetry_data["Roll"].append(attitude_data["Roll"])
            self.telemetry_data["Pitch"].append(attitude_data["Pitch"])
            self.telemetry_data["Yaw"].append(attitude_data["Yaw"])

        gps_data = self.get_gps_position()
        if gps_data:
            self.telemetry_data["Latitude"].append(gps_data["Latitude"])
            self.telemetry_data["Longitude"].append(gps_data["Longitude"])
            self.telemetry_data["Altitude"].append(gps_data["Altitude"])
            self.telemetry_data["HDOP"].append(gps_data["HDOP"])
            self.telemetry_data["VDOP"].append(gps_data["VDOP"])
            self.telemetry_data["Satellites"].append(gps_data["Satellites"])

        flight_mode = self.get_flight_mode()
        if flight_mode:
            self.telemetry_data["Flight_Mode"].append(flight_mode)

        battery_data = self.get_battery_status()
        if battery_data:
            self.telemetry_data["Voltage"].append(battery_data["Voltage"])
            self.telemetry_data["Current"].append(battery_data["Current"])

        armed_status = self.get_arm_status()
        self.telemetry_data["Armed"].append(armed_status)


    # Collects telemetry data and saves it periodically
    def telemetry_collection(self):
        print("Requesting data streams...")
        self.master.mav.request_data_stream_send(self.master.target_system,
                                                 self.master.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        print("Getting telemetry data...")

        while True:
            self.get_all_telemetry()
            self.save_to_json()
            print(f"Telemetry data saved to {self.log_filename}")
            time.sleep(0.5)

    # Arms or disarms the drone
    def arm_disarm(self, arm):
        if arm:
            self.master.arducopter_arm()
            print("Arming motors")
            self.master.motors_armed_wait()
            print("Motors armed")
        else:
            self.master.arducopter_disarm()
            print("Disarming motors")
            self.master.motors_disarmed_wait()
            print("Motors disarmed")

    # Sets the motor speed from 0 to 100%
    def set_motor_speed(self, speed_percent):
        """Set motor speed from 0 to 100%."""
        speed = speed_percent / 100.0
        self.master.mav.set_actuator_control_target_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            0,  # group_mlx (actuator group)
            [speed] * 8,  # controls (throttle values for motors 1-8)
            0  # target_system
        )

    # Sends position and velocity targets to the drone
    def send_position(self, target_x, target_y, target_z, velocity_x=0, velocity_y=0, velocity_z=0):
        current_position = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if current_position:
            start_x = current_position.x
            start_y = current_position.y
            start_z = current_position.z
            target_x += start_x
            target_y += start_y
            target_z += start_z
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (positions and velocities enabled)
                target_x, target_y, target_z,  # x, y, z positions in meters
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not used)
                0, 0  # yaw, yaw_rate (not used)
            )
            print(f"Moving to position (x: {target_x}, y: {target_y}, z: {target_z}) with velocity (vx: {velocity_x}, vy: {velocity_y}, vz: {velocity_z})")
        else:
            print("Failed to receive current position data.")

    # # Makes the drone fly in a square pattern
    # def fly_square(self):
    #     velocity = 0.25  # Speed of 0.25 m/s
    #     # Move forward by 0.5 meters
    #     self.send_position(0.5, 0, 0, velocity_x=int(velocity * 10))
    #     time.sleep(5)

    #     # Move left by 0.5 meters
    #     self.send_position(0, -0.5, 0, velocity_y=int(-velocity * 10))
    #     time.sleep(5)

    #     # Move backward by 0.5 meters
    #     self.send_position(-0.5, 0, 0, velocity_x=int(-velocity * 10))
    #     time.sleep(5)

    #     # Move right by 0.5 meters
    #     self.send_position(0, 0.5, 0, velocity_y=int(velocity * 10))
    #     time.sleep(5)

    def fly_square_small(self):
        # Ustal prędkość na 0.5 m/s (dostosuj do potrzeb)
        velocity = 1
        # altitude


        for i in range(10):
            # Przesuń do przodu o 0.5 metra
            if(i%2 == 0):
                value = 1
            else:
                value = -1

            self.send_position(0, 2*value-(i*0.2),  2-(i*0.2) , velocity_x=velocity)
            time.sleep(1.5)


        # # Przesuń do przodu o 0.5 metra
        # self.send_position(0.1, 0, 2, velocity_x=velocity)

        # time.sleep(3)

        # # Przesuń w lewo o 0.5 metra
        # self.send_position(0.2, -0.2, 2.5, velocity_y=-velocity)
        # time.sleep(3)

        # # Przesuń do tyłu o 0.5 metra
        # self.send_position(-0.3, 0.3, 2, velocity_x=-velocity)
        # time.sleep(3)

        # # Przesuń w prawo o 0.5 metra
        # self.send_position(-0.4, 0.4, 2.5, velocity_y=velocity)
        # time.sleep(3)




    def fly_square(self):
        # Ustal prędkość na 0.5 m/s (dostosuj do potrzeb)
        velocity = 1

        # Przesuń do przodu o 0.5 metra
        self.send_position(2, 0, 0, velocity_x=velocity)

        time.sleep(3)

        # Przesuń w lewo o 0.5 metra
        self.send_position(0, -2, 0, velocity_y=-velocity)
        time.sleep(3)

        # Przesuń do tyłu o 0.5 metra
        self.send_position(-2, 0, 0, velocity_x=-velocity)
        time.sleep(3)

        # Przesuń w prawo o 0.5 metra
        self.send_position(0, 2, 0, velocity_y=velocity)
        time.sleep(3)

    def fly_straight(self):
        # Ustal prędkość na 0.5 m/s (dostosuj do potrzeb)
        velocity = 1

        # Przesuń do przodu o 0.5 metra
        self.send_position(2, 0, 0, velocity_x=velocity)
        time.sleep(4)

        # Przesuń do tyłu o 0.5 metra
        self.send_position(-2, 0, 0, velocity_x=-velocity)
        time.sleep(4)

    # Executes a predefined mission

    def mission_one(self):

            self.set_flight_mode(0)
            time.sleep(1)

            self.arm_disarm(1)
            time.sleep(0.1)  # Wait for 2 seconds

            self.set_flight_mode(4)
            time.sleep(4)

            self.takeoff(2)
            time.sleep(4)

            # self.fly_square()
            # self.fly_straight()
            self.takeoff(2.5)
            print("lecimy test~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

            self.fly_square_small()
            time.sleep(2)

            self.set_flight_mode(8)
            time.sleep(4)

    # Sends a takeoff command to the drone
    def takeoff(self, altitude):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        print(f"Takeoff command sent for altitude {altitude} meters")

    # Sets the flight mode of the drone
    def set_flight_mode(self, mode_number):
        mode_mapping = {
            0: 'STABILIZE',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            5: 'LOITER',
            6: 'RTL',
            8: 'LAND',
            12: 'AUTOTUNE',
            13: 'POSHOLD',
            18: 'SMART_RTL'
        }

        if mode_number not in mode_mapping:
            print(f"Unknown mode number: {mode_number}")
            print("Valid mode numbers:", list(mode_mapping.keys()))
            return

        mode = mode_mapping[mode_number]
        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)
        print(f"Flight mode set to {mode} (mode number {mode_number})")

# Example usage
if __name__ == "__main__":
    drone = FC_Controller()
    drone.mission_one()
