from pymavlink import mavutil
import os
import json
import time
import math
from datetime import datetime
import shutil
import logging

from threading import Thread, Lock, Event
import queue

from CV_Controller import BallDetector


            # 0: 'STABILIZE',
            # 2: 'ALT_HOLD',
            # 3: 'AUTO',
            # 4: 'GUIDED',
            # 5: 'LOITER',
            # 6: 'RTL',
            # 8: 'LAND',
            # 12: 'AUTOTUNE',
            # 13: 'POSHOLD',
            # 18: 'SMART_RTL'

#COMMANDS PRIORITY => 
#                       priority= 1 - for mission commands
#                       priority= 2 - for failsafe purposes, i.e. battery failsafe

#TODO:
# 1. (written in telemetry collection method)
# 2. [DONE] __del__ (can be atted auto landing, fail safe etc, in case of program crash)
# 3. [DONE] Battery failsafe
# 4. [DONE] Write down commands priority
# 5. [DONE] JSON cannot read properly flight mode parameter, rest works fine
# 6. [DONE???] When failsafe is triggered, abort mission and execute only RTL
# 7. [DONE???] When Land/RTL set, stop command thread 
# 8. [DONE] Before arming drone add method get position from launchpad
# 9. Commands for handling errors
# 10. [POORLY DONE - HARDCODE] Change logging to JSON so it ends after drone change state from armed to disarmed  
# 11. [DONE in v2 algo] Change algorithm from last step of mission_phase_one
# 12. [DONE] Add to JSON telemetry logs parameter of battery % next to voltage
# 13. [DONE] Add navigate to barrel logic in navigate_to_target method
# 14. [DONE? CHECK IF CORRECTLY IMPLEMENTED] Implement current telemetry method to use it in algorithm
# 15. Add some interruptions - i.e. when drone is 1m above and going down, send commands to stay in place
# 16. Telemetry must be send way faster to command complition checks work

class FC_Controller:
    def __init__(self, connection_string='/dev/ttyACM0', baud_rate=115200, log_dir="/home/KNR/KNR-dron/LOGS/"):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        self.log_dir = log_dir

        # self.pending_commands = {}
        # self.ack_lock = Lock()

        self.file_lock = Lock()
        self.save_count = 0

        self.port_mutex = Lock()
        self.telemetry_data = self.reset_telemetry_data()
        self.log_filename = self.create_log_filename()
        self.command_queue = queue.PriorityQueue()
        self.command_counter = 0
        self.lowBatteryCounter = 0
        self.failsafeVoltageThreshold = 13
        self.abortMission = False
        self.start_position = [0.0,0.0,0.0] #lat, lon, alt
        self.reset_position = [0.0,0.0,0.0]
        self.is_armed = False
        self.isLanding = False
        self.is_on_ground = False
        self.iteration_count = 0
        self.last_flight_mode = None
        self.last_HDOP = None
        self.last_VDOP = None
        self.last_Satelites = None
        self.last_Voltage = None
        self.last_Current = None

        self.latest_telemetry = None
        
        # Start the connection by waiting for heartbeat
        self._wait_for_heartbeat()

        # Set arming check parameter to 0 - disable all arming flags
        self.set_param('ARMING_CHECK', 0)
        self.set_flight_mode(0)

        self.telemetry_lock = Lock()  # Mutex for telemetry
        self.telemetry_event = Event()  # Event to signal when telemetry collection is done

        # Start telemetry collection in a separate thread
        self.telemetry_thread = Thread(target=self.telemetry_collection)
        self.telemetry_thread.start()

        self.command_processor_thread = Thread(target=self.process_commands)
        self.command_processor_thread.start()

        # self.ack_event = Event() 

        self.current_altitude = None
        self.current_latitude = None
        self.current_longitude = None      

        # self.ack_listener_thread = Thread(target=self.ack_listener)
        # self.ack_listener_thread.start()

        self.mission_thread = None

        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(threadName)s] %(levelname)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            handlers=[
                logging.FileHandler("drone_log.log"),
                logging.StreamHandler()
            ]
        )

    def __del__(self):
        # Safe cleanup of threads in case of program termination
        if self.command_processor_thread.is_alive():
            self.command_processor_thread.join()
        if self.telemetry_thread.is_alive():
            self.telemetry_thread.join()
        # if self.ack_listener_thread.is_alive():
        #     self.ack_listener_thread.join()

        self.set_flight_mode(6)

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
            "Vx": [],
            "Vy": [],
            "Vz": [],
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
        if self.save_count >= 100000:
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
            if isinstance(msg.voltages, list) and len(msg.voltages) > 0:
                voltage = msg.voltages[0] / 1000.0  # Napięcie w V
            else:
                print("Invalid voltage data format")
                return None
            current = msg.current_battery / 100.0
            min_voltage = 12.8
            max_voltage = 16.8

            # Sprawdzamy, czy napięcie nie wychodzi poza zakres
            if voltage < min_voltage:
                battery_percentage = 0  # Zwracamy 0% jako integer
            elif voltage > max_voltage:
                battery_percentage = 100  # Zwracamy 100% jako integer
            else:
                # Obliczamy procent naładowania baterii jako wartość numeryczną (bez znaku "%")
                battery_percentage = round(((voltage - min_voltage) / (max_voltage - min_voltage)) * 100)


            if voltage < self.failsafeVoltageThreshold and voltage > 5:
                self.lowBatteryCounter+=1
            if self.lowBatteryCounter >= 5  and not self.abortMission: #When voltage log is below certain threshhold 5 times (10 sec), Land/RTL is triggered
                self.send_command(lambda: self.set_flight_mode(8), priority=2) # Always highest priority command
                self.abortMission = True # Need to be set after sending command
                print("@@@@@@@@@@@@@@@@@@@@@ ABORT MISSION - BATTERY FAILSAFE TRIGGERED @@@@@@@@@@@@@@@@@@@@@")

            return {
                "Voltage": [round(voltage, 2), battery_percentage],  # Zwracamy napięcie jako float oraz procent jako integer.
                "Current": round(current, 2)  # Zwracamy prąd jako float z dwoma miejscami po przecinku.
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
        
     # Retrieves GPS position data
    def get_coordinates(self):
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            return [lat, lon, alt]
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
                # self.command_queue = queue.PriorityQueue() # When Land triggered, replace queue with new empty queue
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

    # Retrieves speed data (Vx, Vy, Vz) from GLOBAL_POSITION_INT
    def get_global_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            vx = msg.vx / 100.0  # Prędkość w osi X w m/s
            vy = msg.vy / 100.0  # Prędkość w osi Y w m/s
            vz = msg.vz / 100.0  # Prędkość w osi Z w m/s
            return {
                "Vx": vx,
                "Vy": vy,
                "Vz": vz
            }
        else:
            print("No global position data received")
            return None

   # Collects all telemetry data and appends to the telemetry data structure
    def get_all_telemetry(self):
        with self.telemetry_lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.telemetry_data["Timestamp"].append(timestamp)

            # Pobieranie danych attitude
            attitude_data = self.get_attitude()
            if attitude_data:
                self.telemetry_data["Roll"].append(attitude_data["Roll"])
                self.telemetry_data["Pitch"].append(attitude_data["Pitch"])
                self.telemetry_data["Yaw"].append(attitude_data["Yaw"])

            # Pobieranie danych GPS
            gps_data = self.get_gps_position()
            if gps_data:
                self.telemetry_data["Latitude"].append(gps_data["Latitude"])
                self.telemetry_data["Longitude"].append(gps_data["Longitude"])
                self.telemetry_data["Altitude"].append(gps_data["Altitude"])
                if self.iteration_count % 10 == 0:
                    if gps_data:
                        self.telemetry_data["HDOP"].append(gps_data["HDOP"])
                        self.telemetry_data["VDOP"].append(gps_data["VDOP"])
                        self.telemetry_data["Satellites"].append(gps_data["Satellites"])
                        self.last_HDOP = gps_data["HDOP"]
                        self.last_VDOP = gps_data["VDOP"]
                        self.last_Satelites = gps_data["Satellites"]
                else:
                    if self.last_HDOP and self.last_VDOP and self.last_Satelites is not None:
                        self.telemetry_data["HDOP"].append(self.last_HDOP)
                        self.telemetry_data["VDOP"].append(self.last_VDOP)
                        self.telemetry_data["Satellites"].append(self.last_Satelites)
                    else:
                        self.telemetry_data["HDOP"].append('UNKNOWN HDOP')
                        self.telemetry_data["VDOP"].append('UNKNOWN VDOP')
                        self.telemetry_data["Satellites"].append('UNKNOWN SATELlITES')
            

            if self.iteration_count % 5 == 0:
                flight_mode = self.get_flight_mode()
                if flight_mode:
                    self.telemetry_data["Flight_Mode"].append(flight_mode)
                    self.last_flight_mode = flight_mode
            else:
                if self.last_flight_mode is not None:
                    self.telemetry_data["Flight_Mode"].append(self.last_flight_mode)
                else:
                    self.telemetry_data["Flight_Mode"].append('UNKNOWN FLIGHT MODE')

            # Pobieranie prędkości z GLOBAL_POSITION_INT
            global_position_data = self.get_global_position()
            if global_position_data:
                self.telemetry_data["Vx"].append(global_position_data["Vx"])
                self.telemetry_data["Vy"].append(global_position_data["Vy"])
                self.telemetry_data["Vz"].append(global_position_data["Vz"])
            else:
                self.telemetry_data["Vx"].append('UNKNOWN Vx')
                self.telemetry_data["Vy"].append('UNKNOWN Vy')
                self.telemetry_data["Vz"].append('UNKNOWN Vz')

            # # Pobieranie prędkości
            # speed_data = self.get_speed()
            # if speed_data:
            #     self.telemetry_data["Airspeed"].append(speed_data["Airspeed"])
            #     self.telemetry_data["Groundspeed"].append(speed_data["Groundspeed"])
            # else:
            #     self.telemetry_data["Airspeed"].append('UNKNOWN AIRSPEED')
            #     self.telemetry_data["Groundspeed"].append('UNKNOWN GROUND SPEED')

            battery_data = self.get_battery_status()
            self.telemetry_data["Voltage"].append(battery_data["Voltage"])
            if self.iteration_count % 10 == 0:
                if battery_data:
                    self.telemetry_data["Current"].append(battery_data["Current"])
                    self.last_Current = battery_data["Current"]
            else:
                if self.last_Current is not None:
                    self.telemetry_data["Current"].append(self.last_Current)
                else:
                    self.telemetry_data["Current"].append('UNKNOWN CURRENT')

            # Pobieranie statusu uzbrojenia co kilka iteracji (np. co 5 iteracji)
            if self.iteration_count % 10 == 0:
                armed_status = self.get_arm_status()
                self.telemetry_data["Armed"].append(armed_status)

            # Inkrementacja licznika iteracji
            self.iteration_count += 1


    # Collects telemetry data and saves it periodically
    def telemetry_collection(self):
        print_counter = 0
        while True:
            try:
                with self.port_mutex:
                # print("Requesting data streams...")
                    self.master.mav.request_data_stream_send(self.master.target_system,
                                                    self.master.target_component,
                                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 100, 1)
                    # print("Getting telemetry data...")
                    #TODO 2 functions below can be modified to be out of mutex zone
                    self.get_all_telemetry()
                    # ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                    # if ack_msg:
                    #     self.handle_command_ack(ack_msg)
                    # if print_counter%50 == 0:
                    #     print(f"Telemetry data saved to {self.log_filename}")
                    # logging.info(f"Telemetry data saved to {self.log_filename}")
                    self.telemetry_event.set()
                self.save_to_json()
                print_counter += 1

            except Exception as e:
                print(f"Error in telemetry collection: {e}")
            time.sleep(0.2)

# TODO -> SAVE TO JSON OUT OF MUTEX
    # def save_recieved_data(self):
    #     self.get_all_telemetry()
    #     self.save_to_json()
    #     print(f"Telemetry data saved to {self.log_filename}")   

############################################################################################################
########################################  MAVLINK BASIC COMMANDS ###########################################
############################################################################################################

    def set_param(self, param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        command_id = mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER  # Using the MAV_CMD_DO_SET_PARAMETER command ID

        # # Register the command before sending
        # with self.ack_lock:
        #     self.pending_commands[command_id] = f'Set param {param_id}'

        # Send the parameter change command
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )
        print(f"Setting parameter {param_id} to {param_value}")


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

        command_id = mavutil.mavlink.MAV_CMD_DO_SET_MODE

        # # Register the command before sending
        # with self.ack_lock:
        #     self.pending_commands[command_id] = f'Set flight mode to {mode}'

        self.master.set_mode(mode_id)
        print(f"Flight mode set to {mode} (mode number {mode_number})")


    def arm_disarm(self, arm):
        command_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

        if arm:
            # # Register the command before sending
            # with self.ack_lock:
            #     self.pending_commands[command_id] = 'Arm motors'

            self.master.arducopter_arm()
            print("Arming motors")
            self.master.motors_armed_wait()
            print("Motors armed")

        else:
            # # Register the command before sending
            # with self.ack_lock:
            #     self.pending_commands[command_id] = 'Disarm motors'

            self.master.arducopter_disarm()
            print("Disarming motors")
            self.master.motors_disarmed_wait()
            print("Motors disarmed")


    def send_position(self, target_x, target_y, target_z, velocity_x, velocity_y, velocity_z):
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
                mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
                 0b110111000000,
                # 0b0000111111000011,  # type_mask (positions and velocities enabled)
                target_x, target_y, target_z,  # x, y, z positions in meters
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not used)
                0, 0  # yaw, yaw_rate (not used)
            )
            print(f"Moving to position (x: {target_x}, y: {target_y}, z: {target_z}) with velocity (vx: {velocity_x}, vy: {velocity_y}, vz: {velocity_z})")
        else:
            print("Failed to receive current position data.")

    def send_global_position(self, target_lat, target_lon, target_alt, velocity_x, velocity_y, velocity_z):
        # Convert latitude and longitude to integers required by MAVLink (scaled by 1e7)
        lat_int = int(target_lat * 1e7)
        lon_int = int(target_lon * 1e7)
        alt = target_alt  # Assuming target_alt is in meters relative to home position

        # Set the target position and velocities
        self.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame
            0b110111000000,  # type_mask (positions and velocities enabled)
            lat_int,
            lon_int,
            alt,
            velocity_x,
            velocity_y,
            velocity_z,
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        print(f"Moving to global position (lat: {target_lat}, lon: {target_lon}, alt: {target_alt}) "
            f"with velocity (vx: {velocity_x}, vy: {velocity_y}, vz: {velocity_z})")
        
    def reposition_copter(self, target_lat, target_lon, target_alt, ground_speed, yaw_angle):

        lat_int = int(target_lat * 1e7)
        lon_int = int(target_lon * 1e7)
        alt = target_alt  # Assuming target_alt is in meters relative to home position

        # command_id = mavutil.mavlink.MAV_CMD_DO_REPOSITION
        # with self.ack_lock:
        #     self.pending_commands[command_id] = 'Reposition'

        self.master.mav.command_long_send(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # Command ID for reposition
            0,  # Confirmation (0 = First transmission)
            lat_int,  # param1: Latitude in WGS84 format
            lon_int,  # param2: Longitude in WGS84 format
            alt,  # param3: Altitude (in meters above home position)
            ground_speed,  # param4: Desired ground speed (in m/s)
            yaw_angle,  # param5: Yaw angle in degrees
            0, 0  # Unused parameters
        )
        print(f"Repositioning vehicle to (lat: {target_lat}, lon: {target_lon}, alt: {target_alt}) "
            f"with ground speed: {ground_speed} m/s and yaw: {yaw_angle} degrees")


    # If sending velocity or acceleration commands, they should be re-sent every second 
    # (the vehicle will stop after 3 seconds if no command is received)
    def send_velocity(self, velocity_x, velocity_y, velocity_z):
        # Set the target velocities in the body frame (FRD)
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use the body frame (Forward-Right-Down)
            0b110111000111,  # type_mask (ignore position, acceleration, yaw; enable velocity only)
            0, 0, 0,  # x, y, z positions (ignored)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocities in body frame
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        print(f"Commanding drone to move with velocity vx: {velocity_x} m/s, vy: {velocity_y} m/s, vz: {velocity_z} m/s")


    def takeoff(self, altitude):
        command_id = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        # with self.ack_lock:
        #     self.pending_commands[command_id] = 'Takeoff'

        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                        command_id, 0, 0, 0, 0, 0, 0, 0, altitude)
        print(f"Takeoff command sent for altitude {altitude} meters")


############################################################################################################
####################################  MAVLINK COMMANDS COMPLITION CHECK ####################################
############################################################################################################
    
    def wait_until_altitude(self, target_altitude, tolerance=0.2, timeout=30):
        start_time = time.time()
        logging.info(f"Waiting to reach altitude: {target_altitude}m with tolerance {tolerance}m")
        log_counter=0
        while not self.abortMission and not self.isLanding:
            with self.telemetry_lock:
                current_altitude = self.current_altitude

            if current_altitude is not None and abs(current_altitude - target_altitude) <= tolerance:
                logging.info(f"Target altitude reached: {current_altitude}m")
                break
            if time.time() - start_time > timeout:
                logging.warning(f"Timeout waiting for target altitude: Current = {current_altitude}m, Target = {target_altitude}m")
                break
            time.sleep(0.1)
            if log_counter%10 == 0:
                logging.info(f"Drone changing altitude... Current altitude: {current_altitude}m")
            log_counter += 1




    def gps_to_frd(self, current_position, start_position):
        lat_diff = (current_position[0] - start_position[0]) * 111320  # meters per degree of latitude
        lon_diff = (current_position[1] - start_position[1]) * 111320 * math.cos(math.radians(start_position[0]))  # meters per degree of longitude
        alt_diff = (start_position[2] - current_position[2])  # Altitude in meters (downward, so start_alt - current_alt)
        
        return lat_diff, lon_diff, alt_diff  # x = lat_diff, y = lon_diff, z = alt_diff (FRD)
      

    def wait_until_position(self, target_x, target_y, target_z, tolerance=0.5, timeout=30):
        start_time = time.time()
        while not self.abortMission and not self.isLanding:
            with self.telemetry_lock:
                current_gps = [self.current_altitude,self.current_latitude,self.current_longitude]

            if current_gps is None:
                print("No current GPS data, retrying...")
                time.sleep(0.1)
                continue
            current_frd = self.gps_to_frd(current_gps, self.start_position)
            dx = abs(current_frd[0] - target_x)
            dy = abs(current_frd[1] - target_y)
            dz = abs(current_frd[2] - target_z)

            print(f"Current position (FRD): x={current_frd[0]}, y={current_frd[1]}, z={current_frd[2]}")
            print(f"Target position: x={target_x}, y={target_y}, z={target_z}")
            print("Drone changing position...")

            if dx <= tolerance and dy <= tolerance and dz <= tolerance:
                logging.info("# Target position reached #")
                break

            if time.time() - start_time > timeout:
                logging.warning("# Timeout waiting for target position #")
                break
            time.sleep(0.1)
        print("# Target position reached #")
    
    def wait_until_GPSposition(self, target_lat, target_lon, target_alt, tolerance=0.5, timeout=30):
        start_time = time.time()
        logging.info(f"Waiting to reach GPS position: ({target_lat}, {target_lon}, {target_alt}) with tolerance {tolerance}")
        while not self.abortMission and not self.isLanding:
            with self.telemetry_lock:
                current_gps = [self.current_altitude,self.current_latitude,self.current_longitude]
            dx = abs(current_gps[0] - target_lat)
            dy = abs(current_gps[1] - target_lon)
            dz = abs(current_gps[2] - target_alt)
            logging.info(f"Current GPS position: ({current_gps[0]}, {current_gps[1]}, {current_gps[2]})")

            if dx <= tolerance and dy <= tolerance and dz <= tolerance:
                logging.info(f"Reached target GPS position: ({target_lat}, {target_lon}, {target_alt})")
                break
            if time.time() - start_time > timeout:
                logging.warning("# Timeout waiting for target position #")
                break
            time.sleep(0.1)
            logging.info("Drone moving towards target GPS position...")


    # def handle_command_ack(self, ack_msg):
    #     command = ack_msg.command
    #     result = ack_msg.result

    #     if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #         logging.info(f"Command {command} executed successfully.")
    #         self.ack_event.set()
    #     elif result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
    #         logging.warning(f"Command {command} temporarily rejected.")
    #         self.ack_event.set()
    #     elif result == mavutil.mavlink.MAV_RESULT_DENIED:
    #         logging.error(f"Command {command} denied.")
    #         self.ack_event.set()
    #     elif result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
    #         logging.error(f"Command {command} unsupported.")
    #         self.ack_event.set()
    #     elif result == mavutil.mavlink.MAV_RESULT_FAILED:
    #         logging.error(f"Command {command} failed.")
    #         self.ack_event.set()




############################################################################################################
########################################  MAVLINK MISSION COMMANDS #########################################
############################################################################################################

    def begin_flight(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1, command_name='set_flight_mode_STABILIZE')
        self.send_command(lambda: self.arm_disarm(1), priority=1, command_name='arm_disarm')
        self.send_command(lambda: self.set_flight_mode(4), priority=1, command_name='set_flight_mode_GUIDED')

    def navigate_to_target(self, item, platform_numer=None):
        detector = BallDetector()
        mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'
        frame = detector.fetch_frame(mjpeg_url)
        balls = detector.process_frame_debug(frame)
        multV = 2  # Mnożnik prędkości

        # Ustalanie wektora prędkości na podstawie przedmiotu
        if item == 'target':
            vector = detector.target_vector
        elif item == 'platform':
            vector = detector.get_platform_vector(platform_numer)
        elif item == 'barrel':
            vector = detector.barrel_vector
        else:
            print("Wrong item name")
            return  # Zakończ funkcję, jeśli nie rozpoznano przedmiotu

        # Jeśli wektor nie istnieje, ustaw prędkości na 0
        if vector is None:
            vx = vy = 0
            logging.info(f"No vector detected for {item}. Holding position.")
        else:
            # Przeliczanie prędkości w osiach X i Y
            vx = vector[0] / 960 * multV
            vy = vector[1] / 540 * multV
            logging.info(f"{item.capitalize()} vector: {vector[0]}, {vector[1]}")
            logging.info(f"Calculated speed in X: {vx}, Y: {vy}")

        # Wysyłanie komendy sterowania prędkością
        self.send_command(lambda: self.send_velocity(vx, vy, 0), priority=1)

    # Vz - absolute value, direction is handled here
    def change_altitude(self, target_altitude, dir, vz):
        if dir == 'up':
            while self.current_altitude < (self.start_position[2]+target_altitude):
                self.send_command(lambda: self.send_velocity(0,0,-vz), priority=1)
                time.sleep(0.1)
 
        else:
            while self.current_altitude > (self.start_position[2]+target_altitude):
                self.send_command(lambda: self.send_velocity(0,0,vz), priority=1)
                time.sleep(0.1)


    def smooth_landing_with_detection(self, target, detected_balls, platform_positions, final_altitude=0.1, threshold=8.0):
        detector = BallDetector()  # Inicjalizacja detektora
        detected_balls = {} 
        platform_positions = {}
        multV = 2  # Współczynnik prędkości do `navigate_to_target`

        while True:
            with self.telemetry_lock:
                current_altitude = self.current_altitude
                velocity_z = self.current_velocity_z

            if current_altitude is None:
                logging.warning("No altitude data available.")
                break

            # Gdy dron jest powyżej progu (np. 8 metrów)
            if current_altitude > threshold:
                new_altitude = max(final_altitude, current_altitude - 0.5)  # Obniżanie o 0.5m
                logging.info(f"Reducing altitude from {current_altitude}m to {new_altitude}m")
                self.send_command(lambda: self.change_altitude(new_altitude, 'down', 1), priority=1, command_name='change_altitude_large')

            # Gdy dron jest poniżej progu (np. 8 metrów) - zaczynamy sprawdzać platformę lub beczkę
            elif current_altitude > 1.0:
                new_altitude = max(final_altitude, current_altitude - 0.2)  # Mniejsze schodzenie o 0.2m
                logging.info(f"Reducing altitude from {current_altitude}m to {new_altitude}m (small decrement)")
                self.send_command(lambda: self.change_altitude(new_altitude, 'down', 0.8), priority=1, command_name='change_altitude_small')

                # nie mamy tu wektora - trzeba zmienic
                if len(detector.large_contours) != 0:
                    logging.info(f"{target.capitalize()} detected, navigating to {target}.")
                    self.send_command(lambda: self.navigate_to_target(target), priority=1, command_name=f'navigate_to_{target}')
                else:
                    logging.info(f"{target.capitalize()} not detected yet.")

            # Gdy dron jest poniżej 1 metra, kontynuujemy schodzenie z mniejszą prędkością
            elif current_altitude <= 1.0:
                new_altitude = max(final_altitude, current_altitude - 0.1)  # Bardzo małe schodzenie
                logging.info(f"Reducing altitude from {current_altitude}m to {new_altitude}m (very small decrement)")
                self.send_command(lambda: self.change_altitude(new_altitude, 'down', 0.2), priority=1, command_name='change_altitude_very_small')
    
                ball_color = detector.detect_ball_color()
                if ball_color:
                    logging.info(f"Ball detected on {target}: {ball_color}. Navigating to target.")
                    self.send_command(lambda: self.navigate_to_target(target), priority=1, command_name=f'navigate_to_{target}')

                    # Zapis wykrytego koloru piłki i pozycji
                    detected_balls[target] = ball_color
                    platform_positions[target] = [
                        self.latest_telemetry["Latitude"],
                        self.latest_telemetry["Longitude"],
                        self.latest_telemetry["Altitude"]
                    ]
                    logging.info(f"Ball color {ball_color} detected and saved for {target}.")
                    break

                # # Sprawdzenie, czy dron już wylądował
                # if abs(current_altitude - final_altitude) <= 0.05 and abs(velocity_z) < 0.1:
                #     logging.info("Drone has landed.")

                #     # Sprawdzenie, czy piłka jest nadal w polu widzenia
                #     ball_color = detector.detect_ball_color()
                #     if ball_color:
                #         logging.info(f"Ball detected on the ground: {ball_color}. Closing gripper.")
                        
                #         # Zamknięcie chwytaka
                #         self.close_gripper()
                        
                #         # Utrzymywanie silników na niskich obrotach
                #         while self.is_gripper_closing():
                #             logging.info("Keeping motors active while closing the gripper.")
                #             self.send_command(lambda: self.set_flight_mode(4), priority=1, command_name='set_flight_mode_GUIDED')  # GUIDED mode
                #             time.sleep(0.1)  # Krótkie opóźnienie, aby nie przeciążać komend

                #     break  # Zakończenie pętli po zakończeniu operacji chwytaka

            time.sleep(0.4)  # Krótkie opóźnienie przed następnym krokiem schodzenia
###################################################################################################
######################################## MUTEX PROCESSING #########################################
###################################################################################################

    def send_command(self, command_func, priority, command_name=None):
        if command_name is None:
            command_name = command_func.__name__ if hasattr(command_func, '__name__') else 'lambda'
        logging.info(f"Sending command: {command_name} with priority {priority}")
        
        if self.command_processor_thread.is_alive():
            self.command_counter += 1
            self.command_queue.put((priority, self.command_counter, command_func))
            logging.info(f"Command {command_name} queued with counter {self.command_counter}")
            return True
        else:
            logging.error(f"Failed to send command: {command_name} - command processor thread not alive")
            return False
        
    def send_command_and_wait(self, command_func, wait_func, wait_params, priority=1, command_name=None):
        # Wysyła komendę i automatycznie czeka na jej realizację
        # - command_func: funkcja wysyłająca komendę (np. takeoff, send_position)
        # - wait_func: funkcja czekająca na realizację (np. wait_until_altitude, wait_until_position)
        # - wait_params: parametry do funkcji oczekiwania
        self.send_command(command_func, priority, command_name)
        
        # Automatyczne czekanie na realizację komendy
        wait_func(*wait_params)
    


    def process_commands(self):
        while not self.abortMission and not self.isLanding:
            priority, counter, command_func = self.command_queue.get(block=True, timeout=None)

            logging.info(f"Processing command with counter {counter} and priority {priority}")
            self.telemetry_event.clear()
            # self.ack_event.clear()

            with self.port_mutex:
                try:
                    command_func()
                except Exception as e:
                    logging.error(f"Error processing command: {e}")
            
            if not self.telemetry_event.wait(timeout=5):
                logging.warning("Timeout waiting for telemetry update.")
            
            # if not self.ack_event.wait(timeout=10):
            #     logging.warning("Timeout waiting for command ACK.")
            
            logging.info(f"Command {counter} processed successfully.")
            self.command_queue.task_done()

        logging.info("Command processing thread stopped.")

############################################################################################################
###########################################  TESTYYYYYYY ###################################################
############################################################################################################

    
    def do_testow(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1, command_name='set_flight_mode_STABILIZE') 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1, command_name='ARM')  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1, command_name='set_flight_mode_GUIDED')  
        time.sleep(2)
        self.send_command(lambda: self.takeoff(3), priority=1, command_name='takeoff')  
        self.wait_until_altitude(4)
        self.send_command(lambda: self.set_flight_mode(8), priority=1, command_name='set_flight_mode_LAND')
        time.sleep(1)

    def test_podstawowy(self):
        self.begin_flight()
        self.send_command(lambda: self.takeoff(4), priority=1, command_name='takeoff')
        self.wait_until_altitude(4)

        # self.send_command(lambda: self.send_position(0, 0, -1, 0, 0, 0.2), priority=1, command_name='fly_to_point_1')
        # self.wait_until_position(0, 0, -1)
        # self.send_command(lambda: self.send_position(0, 0, 1, 0, 0, 0.2), priority=1, command_name='fly_to_point_2')
        # self.wait_until_position(0, 0, 1)

        # self.send_command(lambda: self.send_position(2, 0, -1, 0, 0, 0.2), priority=1, command_name='fly_to_point_1')
        # self.wait_until_position(2, 0, -1)
        # self.send_command(lambda: self.send_position(-2, 0, 1, 0, 0, 0.2), priority=1, command_name='fly_to_point_1')
        # self.wait_until_position(-2, 0, 1)
 
        self.send_command(lambda: self.set_flight_mode(8), priority=1, command_name='set_flight_mode_LAND')
        self.wait_until_altitude(0.1)
        time.sleep(2)
        self.send_command(lambda: self.set_flight_mode(0), priority=1, command_name='set_flight_mode_STABILIZE')
        time.sleep(1)

    def test_podstawowy2(self):
        self.begin_flight()
        self.send_command(lambda: self.takeoff(4), priority=1, command_name='takeoff')
        self.wait_until_altitude(4)

        self.send_command(lambda: self.send_position(0, 0, -1, 0, 0, 0.2), priority=1, command_name='fly_to_point_1')
        self.wait_until_position(0, 0, -1)
        self.send_command(lambda: self.send_position(0, 0, 1, 0, 0, 0.2), priority=1, command_name='fly_to_point_1')
        self.wait_until_position(0, 0, 1)
 

        self.send_command(lambda: self.set_flight_mode(8), priority=1, command_name='set_flight_mode_LAND')
        self.wait_until_altitude(0.1)
        self.send_command(lambda: self.set_flight_mode(0), priority=1, command_name='set_flight_mode_STABILIZE')
        time.sleep(1)

    def test_lotu_xy(self):
        self.begin_flight()
        self.send_command(lambda: self.takeoff(3), priority=1, command_name='takeoff')
        self.wait_until_altitude(3)

        waypoints = [
            (3, 0, 0),
            (3, 3, 0),
            (0, 3, 0),
            (0, 0, 0)
        ]
    
        for x,y,z in waypoints:
            self.send_command(lambda: self.send_position(x,y,z,0.5,0.5,0), priority=1, command_name='fly_to_point_1')
            self.wait_until_position(x, y, z)

        # # FLY SQUARE
        # self.send_command(lambda: self.send_position(3, 0, 0, 0.5, 0, 0), priority=1, command_name='fly_to_point_1')
        # self.wait_until_position(3, 0, 3)
        # self.send_command(lambda: self.send_position(0, 3, 0, 0, 0.5, 0), priority=1, command_name='fly_to_point_2')
        # self.wait_until_position(0, 3, 3)
        # self.send_command(lambda: self.send_position(-3, 0, 0, 0.5, 0, 0), priority=1, command_name='fly_to_point_3')
        # self.wait_until_position(-3, 0, 3)
        # self.send_command(lambda: self.send_position(0, -3, 0, 0, 0.5,0), priority=1, command_name='fly_to_point_4')
        # self.wait_until_position(0, -3, 3)

        self.send_command(lambda: self.set_flight_mode(8), priority=1, command_name='set_flight_mode_LAND')
        self.wait_until_altitude(0.1)
        self.send_command(lambda: self.set_flight_mode(0), priority=1, command_name='set_flight_mode_STABILIZE')
        time.sleep(1)

    def test_lotu_xz(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.takeoff(4), priority=1)  
        time.sleep(3)

        # FLY SQUARE
        self.send_command(lambda: self.send_position(-3, 0, 0, 0.5, 0, 0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(0, 0, -3, 0, 0, 0.5), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(3, 0, 0, 0.5, 0, 0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(0, 0, 2, 0, 0, 0.5), priority=1) 
        time.sleep(4)

        self.send_command(lambda: self.set_flight_mode(8), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)

    def test_lotu_VxVy(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.takeoff(3), priority=1)  
        time.sleep(3)

        # POSTAWIĆ DRONA DALEKO OD LUDZI I BYĆ W GOTOWOŚCI Z PILOTEM, NIE WIEM JAK TO DZIAŁA W REALU
        self.send_command(lambda: self.send_velocity(0.5,0,0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(0,0,-0.5), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_velocity(-0.5,0,0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_velocity(0,0,0.5), priority=1) 
        time.sleep(4)

        self.send_command(lambda: self.set_flight_mode(8), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)

    #JEŚLI DZIAŁAJĄ KOMENDY DO LOTU XYZ TO MOŻNA TESTOWAĆ TE 
    def test_lotu_gps(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.takeoff(4), priority=1)  
        time.sleep(3)
        [curr_lat, curr_lon, curr_alt] = self.send_command(lambda: self.get_coordinates())

        # NAJPIERW POLEĆ LEĆ GDZIEŚ W PIZDU 
        self.send_command(lambda: self.send_position(6, 0, 0, 0.5, 0, 0), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(0, 0, -3, 0, 0, 0.5), priority=1) 
        time.sleep(4)
        self.send_command(lambda: self.send_position(0, 4, 0, 0.5, 0, 0), priority=1) 
        time.sleep(4)

        # WRÓC NA KORDY KTÓRE ZAPISAŁEŚ PO TAKEOFF (BADANIE DOKLADNOSCI POWROTU NA KORDY GPS)
        self.send_command(lambda: self.send_global_position(curr_lat,curr_lon, curr_alt, 0.5,0.5,0.5), priority=1) 
        time.sleep(4)

        self.send_command(lambda: self.set_flight_mode(8), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)

    #DRONA POSTAWIĆ TROCHE PRZED (2-4 M) BIALA PLANSZA Z PILKA I ODPALIC TEST - DRON MA BYĆ SKIEROWANY PRZODEM DO PLANSZY
    def test_navigate_to_target(self):
        test_done = False

        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(6)
        self.send_command(lambda: self.takeoff(4), priority=1)  
        time.sleep(3)
        self.send_command(lambda: self.send_position(4, 0, 0, 0.5, 0, 0), priority=1) 

        while not test_done:
            if len(detector.large_contours != 0):
                #  NAKIEROWYWUJ DRONA NA PLATFORME DOPÓKI PIŁKA NIE JEST W ŚRODKU KADRU 
                while not len(detector.balls) == 0:
                    self.send_command(lambda: fcc.navigate_to_target('platform'), priority=1)
                    time.sleep(0.2)
                    self.send_command(lambda: self.send_velocity(0,0,0.2), priority=1) 
                    time.sleep(0.2)
                
                #  NAKIEROWYWUJ DRONA NA PIŁKĘ DOPÓKI PIŁKA NIE JEST W ŚRODKU KADRU LUB GDY DRON JEST POWYŻEJ 1 METRA
                while not detector.is_target_close or (self.latest_telemetry["Altitude"] > self.start_position[2] + 1):
                    self.send_command(lambda: fcc.navigate_to_target('target'), priority=1)
                    time.sleep(0.2)
                    self.send_command(lambda: self.send_velocity(0,0,0.2), priority=1) 
                    time.sleep(0.2)
                
                test_done = True
        
        self.send_command(lambda: self.set_flight_mode(8), priority=1)  
        time.sleep(4)
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)

    # Function to start command thread 
    def start_command_thread(self):
        #command_thread = Thread(target=self.mission_start_v2, daemon=True)
        self.command_thread = Thread(target=self.test_podstawowy, daemon=True)
        self.command_thread.start()
        self.command_thread.join()

# Example usage
if __name__ == "__main__":

    fcc = FC_Controller()
    detector = BallDetector()
    fcc.start_command_thread()
    

    del fcc



