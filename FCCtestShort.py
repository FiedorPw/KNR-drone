from pymavlink import mavutil
import os
import json
import time
import math
from datetime import datetime
import shutil

from threading import Thread, Lock, Event
import queue

from CV_Controller import BallDetector

class FC_Controller:
    def __init__(self, connection_string='/dev/ttyACM0', baud_rate=57600, log_dir="/home/KNR/KNR-dron/LOGS/"):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        self.log_dir = log_dir
        self.file_lock = Lock()
        self.save_count = 0
        self.port_mutex = Lock()
        self.telemetry_data = self.reset_telemetry_data()
        self.log_filename = self.create_log_filename()
        self.command_queue = queue.PriorityQueue()
        self.command_counter = 0
        
        # Start the connection by waiting for heartbeat
        self._wait_for_heartbeat()

        self.telemetry_lock = Lock()  # Mutex for telemetry
        self.telemetry_event = Event()  # Event to signal when telemetry collection is done

        # Start telemetry collection in a separate thread
        self.telemetry_thread = Thread(target=self.telemetry_collection, daemon=True)
        self.telemetry_thread.start()

        self.command_processor_thread = Thread(target=self.process_commands, daemon=True)
        self.command_processor_thread.start()

    # Waits for a heartbeat message from the flight controller
    def _wait_for_heartbeat(self):
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

############################################################################################################
############################################   LOG SAVING  #################################################

    # Creates a new log filename with the current timestamp
    def create_log_filename(self):
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return os.path.join(self.log_dir, f"dane_telemetryczne_{current_time}.json")

    # Resets the telemetry data structure
    def reset_telemetry_data(self):
        return {
            "Timestamp": [],  # Added Timestamp field
            "Altitude": [],
            "Satellites": [],
        }

    # Saves telemetry data to a JSON file
    def save_to_json(self):
        temp_filename = self.log_filename + '.temp'
        with self.file_lock:
            with open(temp_filename, 'w') as temp_file:
                json.dump(self.telemetry_data, temp_file, indent=4)
            shutil.move(temp_filename, self.log_filename)

        self.save_count += 1
        if self.save_count >= 100:
            self.save_count = 0
            self.log_filename = self.create_log_filename()
            self.telemetry_data = self.reset_telemetry_data()

    def set_param(self, param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        try:
            with self.port_mutex:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    param_id.encode('utf-8'),
                    param_value,
                    param_type
                )
        except Exception as e:
            print(f"Error setting parameter: {e}")

    def get_gps_position(self):
        try:
            with self.port_mutex:
                msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
                if msg:
                    alt = msg.alt / 1e3
                    satellites_visible = msg.satellites_visible
                    return {
                        "Altitude": alt,
                        "Satellites": satellites_visible
                    }
                else:
                    print("No GPS position data received")
                    return None
        except Exception as e:
            print(f"Error retrieving GPS position: {e}")
            return None

    # Collects all telemetry data and appends to the telemetry data structure
    def get_all_telemetry(self):
        with self.telemetry_lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.telemetry_data["Timestamp"].append(timestamp)

            gps_data = self.get_gps_position()
            if gps_data:    
                self.telemetry_data["Altitude"].append(gps_data["Altitude"])
                self.telemetry_data["Satellites"].append(gps_data["Satellites"])

            self.telemetry_event.set()


    # Collects telemetry data and saves it periodically
    def telemetry_collection(self):
        while True:
            try:
                with self.port_mutex:
                    print("Requesting data streams...")
                    self.master.mav.request_data_stream_send(self.master.target_system,
                                                    self.master.target_component,
                                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                    print("Getting telemetry data...")
                    self.get_all_telemetry()
                    self.save_to_json()
                    print(f"Telemetry data saved to {self.log_filename}")
                time.sleep(0.5)
            except Exception as e:
                print(f"Error in telemetry collection: {e}")
            time.sleep(0.5)

############################################################################################################
########################################  MAVLINK BASIC COMMANDS ###########################################

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

############################################################################################################
########################################  MAVLINK MISSION COMMANDS #########################################

    def send_command(self, command_func, priority):
        self.command_counter += 1
        self.command_queue.put((priority, self.command_counter, command_func))

    def process_commands(self):
        while True:
            try:
                if not self.command_queue.empty():
                    priority, counter, command_func = self.command_queue.get()
                    with self.port_mutex:
                        command_func()
                    self.command_queue.task_done()
            except Exception as e:
                print(f"Error in command processing: {e}")

############################################################################################################
##################################################  TESTYYYYYYY ############################################

# Send a test command (like velocity) every few seconds
    def send_periodic_commands(self):
        # while True:
        self.telemetry_event.wait()
        self.telemetry_event.clear()

        print("################### Sending command to the drone #############################")
        self.send_command(self.arm_disarm(1), priority=0)  # Or any other command like send_velocity
            # time.sleep(5)  # Wait for 5 seconds before sending the next command
            # self.send_command(self.arm_disarm(0), priority=0)
            # time.sleep(5)  # Or any other command like send_velocity

    # Function to test if telemetry is interrupted by commands
    def test_command_telemetry(self):
        command_thread = Thread(target=self.send_periodic_commands, daemon=True)
        command_thread.start()
        time.sleep(2)
        command_thread.join()


# Example usage
if __name__ == "__main__":
    fcc = FC_Controller()
    # detector = BallDetector()
    fcc.test_command_telemetry()

