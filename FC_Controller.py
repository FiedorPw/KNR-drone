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


#TODO:
# 1. (written in telemetry collection method)
# 2. __del__ (can be atted auto landing, fail safe etc, in case of program crash)
# 3. [DONE] Battery failsafe
# 4. Write down commands priority
# 5. [DONE] JSON cannot read properly flight mode parameter, rest works fine
# 6. [PARTIALLY DONE] When failsafe is triggered, abort mission and execute only RTL
# 7. [PARTIALLY DONE] When Land/RTL set, stop command thread 
# 8. [DONE] Before arming drone add method get position from launchpad
# 9. Commands for handling errors
# 10. [POORLY DONE - HARDCODE] Change logging to JSON so it ends after drone change state from armed to disarmed  
# 11. Change algorithm from last step of mission_phase_one
# 12. Add to JSON telemetry logs parameter of battery % next to voltage
# 13. Add navigate to barrel logic in navigate_to_target method

class FC_Controller:
    def __init__(self, connection_string='/dev/ttyACM0', baud_rate=57600, log_dir="/home/KNR/KNR-dron/LOGS/"):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        self.log_dir = log_dir
        # Lock for thread-safe file operations
        self.file_lock = Lock()
        self.save_count = 0
        self.port_mutex = Lock()
        self.telemetry_data = self.reset_telemetry_data()
        self.log_filename = self.create_log_filename()
        self.command_queue = queue.PriorityQueue()
        self.command_counter = 0
        self.lowBatteryCounter = 0
        self.failsafeVoltageThreshold = 13.5
        self.abortMission = False
        self.start_position = [0.0,0.0,0.0]
        self.is_armed = False
        self.isLanding = False
        
        # Start the connection by waiting for heartbeat
        self._wait_for_heartbeat()

        # Set arming check parameter to 0 - disable all arming flags
        self.set_param('ARMING_CHECK', 0)

        self.telemetry_lock = Lock()  # Mutex for telemetry
        self.telemetry_event = Event()  # Event to signal when telemetry collection is done

        # Start telemetry collection in a separate thread
        self.telemetry_thread = Thread(target=self.telemetry_collection)
        self.telemetry_thread.start()

        self.command_processor_thread = Thread(target=self.process_commands)
        self.command_processor_thread.start()

    def __del__(self):
        #TODO make safe deleting objects and make failsafe like RTL/Land
        self.command_processor_thread.join()
        self.telemetry_thread.join()

    # Waits for a heartbeat message from the flight controller
    def _wait_for_heartbeat(self):
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

############################################################################################################
############################################   LOG SAVING  #################################################
############################################################################################################

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
            with open(temp_filename, 'a') as temp_file:
                json.dump(self.telemetry_data, temp_file, indent=4)
            shutil.move(temp_filename, self.log_filename)
        self.save_count += 1
        if self.save_count >= 10000:
            self.save_count = 0
            self.log_filename = self.create_log_filename()
            self.telemetry_data = self.reset_telemetry_data()

############################################################################################################
######################################   RECIEVING TELEMETRY  ##############################################
############################################################################################################

    # Retrieves battery status data
    def get_battery_status(self):
        msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True)
        if msg:
            voltage = msg.voltages[0] / 1000.0
            current = msg.current_battery / 100.0
            if voltage < self.failsafeVoltageThreshold and voltage > 5:
                self.lowBatteryCounter+=1
            if self.lowBatteryCounter >= 5  and not self.abortMission: #When voltage log is below certain threshhold 5 times (10 sec), Land/RTL is triggered
                self.send_command(lambda: self.set_flight_mode(8), priority=2) # Always highest priority command
                self.abortMission = True # Need to be set after sending command
                print("@@@@@@@@@@@@@@@@@@@@@ ABORT MISSION - BATTERY FAILSAFE TRIGGERED @@@@@@@@@@@@@@@@@@@@@")
            return {
                "Voltage": voltage,
                "Current": current
            }
        else:
            print("No battery status data received")
            return None

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

    # Retrieves the current flight mode
    def get_flight_mode(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            if mode in ['LAND', 'RTL']:
                self.isLanding = True
                print("@@@@@@@@@@@@@@@@@@@@@ ABORT MISSION - LAND MODE TRIGGERED @@@@@@@@@@@@@@@@@@@@@")
                with self.port_mutex:
                    self.command_queue = queue.PriorityQueue() # When Land triggered, replace queue with new queue
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
        with self.telemetry_lock:
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

            #Drone start position
            if armed_status and not self.is_armed:
                self.start_position = [gps_data["Latitude"],gps_data["Longitude"],gps_data["Altitude"]]
                print(f'Drone Start Position {self.start_position}')
                self.is_armed = True

    # Collects telemetry data and saves it periodically
    def telemetry_collection(self):
        while True:
            try:
                with self.port_mutex:
                    # print("Requesting data streams...")
                    self.master.mav.request_data_stream_send(self.master.target_system,
                                                    self.master.target_component,
                                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
                    # print("Getting telemetry data...")
                    #TODO 2 functions below can be modified to be out of mutex zone
                    self.get_all_telemetry()
                    self.save_to_json()
                    print(f"Telemetry data saved to {self.log_filename}")
                    self.telemetry_event.set()
            except Exception as e:
                print(f"Error in telemetry collection: {e}")
            time.sleep(0.5)


# TODO -> SAVE TO JSON OUT OF MUTEX
    # def save_recieved_data(self):
    #     self.get_all_telemetry()
    #     self.save_to_json()
    #     print(f"Telemetry data saved to {self.log_filename}")   

############################################################################################################
########################################  MAVLINK BASIC COMMANDS ###########################################
############################################################################################################

    def set_param(self, param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        # Przesyłanie komendy do zmiany parametru
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )

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

    # Sends position x,y and velocities x,y to the drone
    def send_position(self, target_x, target_y, target_z, velocity_x, velocity_y, velocity_z):
        current_position = self.master.recv_match(type='MAV_FRAME_BODY_FRD', blocking=True)
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
                mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
                0b0000111111000111,  # type_mask (positions and velocities enabled)
                target_x, target_y, target_z,  # x, y, z positions in meters
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not used)
                0, 0  # yaw, yaw_rate (not used)
            )
            print(f"Moving to position (x: {target_x}, y: {target_y}, z: {target_z}) with velocity (vx: {velocity_x}, vy: {velocity_y}, vz: {velocity_z})")
        else:
            print("Failed to receive current position data.")

    def send_velocity(self, target_z, velocity_x, velocity_y):
        current_position = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if current_position:
            start_z = current_position.z
            target_z += start_z  # Dodanie pozycji startowej do target Z
            
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (not used)
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
                0b0000111111001011,  # type_mask (velocities X, Y and position Z enabled)
                0, 0, target_z,  # x, y positions ignored, z position used
                velocity_x, velocity_y, 0,  # x, y velocities used, z velocity ignored
                0, 0, 0,  # x, y, z acceleration (not used)
                0, 0  # yaw, yaw_rate (not used)
            )
            print(f"Moving with velocity (vx: {velocity_x}, vy: {velocity_y}) and target Z: {target_z}")
        else:
            print("Failed to receive current position data.")

    # Sends a takeoff command to the drone
    def takeoff(self, altitude):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        # print(msg)
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

############################################################################################################
########################################  MAVLINK MISSION COMMANDS #########################################
############################################################################################################

    def navigate_to_target(self, item):
        detector = BallDetector()
        mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'
        frame = detector.fetch_frame(mjpeg_url)
        balls = detector.process_frame_debug(frame)

        multV = 2 
        alt = 2

        if item == 'target':
            if detector.target_vector is None:
                vx=vy=0
                alt = -0.2
            else:
                vector = detector.target_vector 
                # camera_dims = detector.frame_dims
                vx = vector[0]/960*multV
                vy = vector[1]/540*multV
            self.send_command(lambda: self.send_velocity(alt,vx,vy), priority=1) 
            print(f"Target vector: {detector.target_vector[0]}, {detector.target_vector[1]}")
            print(f"Prędkość w osi X: {vx}, Prędkość w osi Y: {vy}")

        elif item == 'platform':
            if detector.platform_vector is None:
                vx=vy=0
                alt = -1
            else:
                vector = detector.platform_vector 
                # camera_dims = detector.frame_dims
                vx = vector[0]/960*multV
                vy = vector[1]/540*multV
            self.send_command(lambda: self.send_velocity(alt,vx,vy), priority=1) 
            print(f"Target vector: {detector.platform_vector[0]}, {detector.platform_vector[1]}")
            print(f"Prędkość w osi X: {vx}, Prędkość w osi Y: {vy}")
        else:
            #Add navigate to barrel logic
            print("Wrong item name")

###################################################################################################
######################################## MUTEX PROCESSING #########################################
###################################################################################################

    def send_command(self, command_func, priority):
        self.command_counter += 1
        self.command_queue.put((priority, self.command_counter, command_func))

    def process_commands(self):
        while not self.abortMission or not self.isLanding:
            priority, counter, command_func = self.command_queue.get(block=True, timeout=None)
            # print("Command in function ", command_func.__name__)
            self.telemetry_event.clear()
            with self.port_mutex:
                try:
                    command_func()
                except Exception as e:
                    print(f"Error in command processing: {e}")
            print("####################TASK DONE####################")
            self.command_queue.task_done()


############################################################################################################
##################################################  TESTYYYYYYY ############################################
############################################################################################################

    def do_testow(self):
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(4)

    def mission_start(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.mission_phase_one()
        time.sleep(1)
        self.mission_phase_two()
        time.sleep(1)

    def mission_phase_one(self):
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.takeoff(10), priority=1) 
        time.sleep(8)
        while not detector.large_contours: #while detected platforms == 0 (none detected)
            self.send_command(lambda: self.send_position(1,0,0,1,0,0), priority=1)
            time.sleep(2) 
        
        # probably cannot be written like this - when detect platform, fly to its center and then add altitude
        while not len(detector.large_contours) == 9: # while not all platforms are detected, add altitude
            self.send_command(lambda: self.send_position(0,0,1,0,0,1), priority=1)
        #Podaj pozycje 5 płachty, leć nad 5 płachte dopoki is_close = False
        detector.get_all_platform_positions


    def mission_phase_two():
        #navigateToTarget(get_position_platform_one)
        print("yo")
 
    # Function to start command thread 
    def start_command_thread(self):
        command_thread = Thread(target=self.do_testow, daemon=True)
        command_thread.start()
        command_thread.join()


# Example usage
if __name__ == "__main__":
    fcc = FC_Controller()
    detector = BallDetector()
    # self.send_command(lambda: fcc.arm_disarm(1), priority=1)
    # fcc.start_command_thread()

    del fcc



