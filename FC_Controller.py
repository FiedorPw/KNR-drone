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

#TODO:
# 1. (written in telemetry collection method)
# 2. __del__ (can be atted auto landing, fail safe etc, in case of program crash)
# 3. [DONE] Battery failsafe
# 4. Write down commands priority
# 5. [DONE] JSON cannot read properly flight mode parameter, rest works fine
# 6. [DONE???] When failsafe is triggered, abort mission and execute only RTL
# 7. [DONE???] When Land/RTL set, stop command thread 
# 8. [DONE] Before arming drone add method get position from launchpad
# 9. Commands for handling errors
# 10. [POORLY DONE - HARDCODE] Change logging to JSON so it ends after drone change state from armed to disarmed  
# 11. [DONE in v2 algo] Change algorithm from last step of mission_phase_one
# 12. [DONE] Add to JSON telemetry logs parameter of battery % next to voltage
# 13. Add navigate to barrel logic in navigate_to_target method
# 14. [DONE? CHECK IF CORRECTLY IMPLEMENTED] Implement current telemetry method to use it in algorithm

class FC_Controller:
    def __init__(self, connection_string='/dev/ttyACM0', baud_rate=115200, log_dir="/home/KNR/KNR-dron/LOGS/"):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        self.log_dir = log_dir

        self.pending_commands = {}
        self.ack_lock = Lock()

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

        self.ack_listener_thread = Thread(target=self.ack_listener)
        self.ack_listener_thread.start()

        self.mission_thread = None

    def __del__(self):
        # Safe cleanup of threads in case of program termination
        if self.command_processor_thread.is_alive():
            self.command_processor_thread.join()
        if self.telemetry_thread.is_alive():
            self.telemetry_thread.join()
        if self.ack_listener_thread.is_alive():
            self.ack_listener_thread.join()

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
            "Percentage": [],
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
            min_voltage = 13.2
            max_voltage = 16.8
            battery_percentage = 0.0
            # Sprawdzamy czy napięcie nie wychodzi poza zakres
            if voltage < min_voltage:
                return 0
            elif voltage > max_voltage:
                return 100
            # Obliczamy procent naładowania baterii
            battery_percentage = round(((voltage - min_voltage) / (max_voltage - min_voltage)) * 100, 3)
            #print(f'{self.battery_percentage}%')

            if voltage < self.failsafeVoltageThreshold and voltage > 5:
                self.lowBatteryCounter+=1
            if self.lowBatteryCounter >= 5  and not self.abortMission: #When voltage log is below certain threshhold 5 times (10 sec), Land/RTL is triggered
                self.send_command(lambda: self.set_flight_mode(8), priority=2) # Always highest priority command
                self.abortMission = True # Need to be set after sending command
                print("@@@@@@@@@@@@@@@@@@@@@ ABORT MISSION - BATTERY FAILSAFE TRIGGERED @@@@@@@@@@@@@@@@@@@@@")
            return {
                "Voltage": voltage,
                "Current": current,
                "Percentage": battery_percentage
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
                self.telemetry_data["Percentage"].append(battery_data["Percentage"])

            armed_status = self.get_arm_status()
            self.telemetry_data["Armed"].append(armed_status)

            #Drone start position
            if armed_status and not self.is_armed:
                self.start_position = [gps_data["Latitude"],gps_data["Longitude"],gps_data["Altitude"]]
                print(f'Drone Start Position {self.start_position}')
                self.is_armed = True

            self.latest_telemetry = self.telemetry_data.copy()

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
        command_id = mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER  # Using the MAV_CMD_DO_SET_PARAMETER command ID

        # Register the command before sending
        with self.ack_lock:
            self.pending_commands[command_id] = f'Set param {param_id}'

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

        # Register the command before sending
        with self.ack_lock:
            self.pending_commands[command_id] = f'Set flight mode to {mode}'

        self.master.set_mode(mode_id)
        print(f"Flight mode set to {mode} (mode number {mode_number})")


    def arm_disarm(self, arm):
        command_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

        if arm:
            # Register the command before sending
            with self.ack_lock:
                self.pending_commands[command_id] = 'Arm motors'

            self.master.arducopter_arm()
            print("Arming motors")
            self.master.motors_armed_wait()
            print("Motors armed")

        else:
            # Register the command before sending
            with self.ack_lock:
                self.pending_commands[command_id] = 'Disarm motors'

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
                0b0000111111000011,
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
            0b0000111111000111,  # type_mask (positions and velocities enabled)
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

        command_id = mavutil.mavlink.MAV_CMD_DO_REPOSITION
        with self.ack_lock:
            self.pending_commands[command_id] = 'Reposition'

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

    # Sends velocity targets to the drone in the body frame
    def send_velocity(self, velocity_x, velocity_y, velocity_z):
        # Set the target velocities in the body frame (FRD)
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # Use the body frame (Forward-Right-Down)
            0b0000111111000110,  # type_mask (ignore position, acceleration, yaw; enable velocity only)
            0, 0, 0,  # x, y, z positions (ignored)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocities in body frame
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        print(f"Commanding drone to move with velocity vx: {velocity_x} m/s, vy: {velocity_y} m/s, vz: {velocity_z} m/s")


    def takeoff(self, altitude):
        command_id = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        with self.ack_lock:
            self.pending_commands[command_id] = 'Takeoff'

        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                        command_id, 0, 0, 0, 0, 0, 0, 0, altitude)
        print(f"Takeoff command sent for altitude {altitude} meters")

    # def takeoff(self, altitude):
    #     # Send takeoff command
    #     self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
    #                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    #     print(f"Takeoff command sent for altitude {altitude} meters")
        
    #     while True:
    #         # Receive the COMMAND_ACK message
    #         msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
    #         if msg:
    #             command = msg.command
    #             result = msg.result
    #             progress = getattr(msg, 'progress', None)  # Progress can be 0-100, 255 if not available

    #             # Check if the ACK is for the takeoff command
    #             if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
    #                 if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
    #                     # Report progress if available
    #                     if progress is not None and progress != 255:
    #                         print(f"Takeoff in progress: {progress}%")
    #                     else:
    #                         print("Takeoff in progress...")
    #                 elif result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #                     print("Takeoff complete. Proceeding to next commands...")
    #                     break  # Exit the loop and proceed with the mission
    #                 elif result == mavutil.mavlink.MAV_RESULT_FAILED:
    #                     print("Takeoff failed.")
    #                     return
    #                 else:
    #                     print(f"Takeoff command resulted in {result}")

############################################################################################################
########################################  MAVLINK MISSION COMMANDS #########################################
############################################################################################################

    def navigate_to_target(self, item, platform_numer):
        detector = BallDetector()
        mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'
        frame = detector.fetch_frame(mjpeg_url)
        balls = detector.process_frame_debug(frame)
        multV = 2 

        if item == 'target':
            if detector.target_vector is None:
                vx=vy=0
            else:
                vector = detector.target_vector 
                # camera_dims = detector.frame_dims
                vx = vector[0]/960*multV
                vy = vector[1]/540*multV
            self.send_command(lambda: self.send_velocity(vx,vy,0), priority=1) 
            print(f"Target vector: {detector.target_vector[0]}, {detector.target_vector[1]}")
            print(f"Prędkość w osi X: {vx}, Prędkość w osi Y: {vy}")

        elif item == 'platform':
            if detector.platform_vector is None:
                vx=vy=0
            else:
                vector = detector.get_platform_vector(platform_numer) 
                # camera_dims = detector.frame_dims
                vx = vector[0]/960*multV
                vy = vector[1]/540*multV
            self.send_command(lambda: self.send_velocity(vx,vy,0), priority=1) 
            print(f"Target vector: {detector.platform_vector[0]}, {detector.platform_vector[1]}")
            print(f"Prędkość w osi X: {vx}, Prędkość w osi Y: {vy}")
        else:
            #Add navigate to barrel logic
            print("Wrong item name")

    # Vz - absolute value, direction is handled here
    def change_altitude(self, target_altitude, dir, vz):
        if dir == 'up':
            while self.get_coordinates()[2] < (self.start_position[2]+target_altitude):
                self.send_command(lambda: self.send_velocity(0,0,-vz), priority=1)
                time.sleep(0.1)
 
        else:
            while self.get_coordinates()[2] > (self.start_position[2]+target_altitude):
                self.send_command(lambda: self.send_velocity(0,0,vz), priority=1)
                time.sleep(0.1)
###################################################################################################
######################################## MUTEX PROCESSING #########################################
###################################################################################################

    def send_command(self, command_func, priority):
        if self.command_processor_thread.is_alive():
            self.command_counter += 1
            self.command_queue.put((priority, self.command_counter, command_func))
            return True
        else:
            return False

    def process_commands(self):
        while not self.abortMission and not self.isLanding:
            priority, counter, command_func = self.command_queue.get(block=True, timeout=None)
            # print("Command in function ", command_func.__name__)
            self.telemetry_event.clear()
            with self.port_mutex:
                try:
                    # print(command_func.__name__)
                    command_func()
                except Exception as e:
                    print(f"Error in command processing: {e}")
            print("####################TASK DONE####################")
            self.command_queue.task_done()

        print("PRCESS COMMAND THREAD STOPPED")

    def ack_listener(self):
        while True:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            if msg:
                command = msg.command
                result = msg.result
                with self.ack_lock:
                    if command in self.pending_commands:
                        command_name = self.pending_commands.pop(command)
                        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"{command_name} command acknowleged and accepted.")
                        else:
                            print(f"{command_name} command not accepted. Result: {result}")
            time.sleep(0.1)

############################################################################################################
###########################################  TESTYYYYYYY ###################################################
############################################################################################################

    def do_testow(self):
        self.telemetry_event.wait()
        self.telemetry_event.clear()
        self.send_command(lambda: self.set_flight_mode(0), priority=1) 
        time.sleep(1)
        self.send_command(lambda: self.arm_disarm(1), priority=1)  
        time.sleep(1)
        self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        time.sleep(2)
        self.send_command(lambda: self.takeoff(3), priority=1)  
        time.sleep(6)
        # self.send_command(lambda: self.set_flight_mode(4), priority=1)  
        # time.sleep(1)
        for i in range(30):
            self.send_command(lambda: self.send_global_position(52.2159343,21.0035504, 4,2,2,1), priority=1)  
            time.sleep(0.1)
        # for i in range(30):
        #     self.send_command(lambda: self.send_position(2,0,0,1,0,0), priority=1)
        #     # time.sleep(0.1)
        # for i in range(30):
        #     self.send_command(lambda: self.send_position(0,2,0,0,1,0), priority=1)
        #     # time.sleep(0.1)
        # for i in range(30):
        #     self.send_command(lambda: self.send_position(2,0,0,1,0,0), priority=1)
        #     # time.sleep(0.1)
        # for i in range(30):
        #     self.send_command(lambda: self.send_position(0,2,0,0,1,0), priority=1)
            # time.sleep(0.1)
        # for i in range(100):
        #     self.send_command(lambda: self.send_velocity(1,0,0), priority=1)
        #     time.sleep(0.1)
        # for i in range(100):
        #     self.send_command(lambda: self.send_velocity(0,1,0), priority=1)
        #     time.sleep(0.1)     
        # for i in range(100):
        #     self.send_command(lambda: self.send_velocity(-1,0,0), priority=1)
        #     time.sleep(0.1)
        # for i in range(100):
        #     self.send_command(lambda: self.send_velocity(0,1,0), priority=1)
        #     time.sleep(0.1)  
        # while not detector.large_contours: #while detected platforms == 0 (none detected)
        #     self.send_command(lambda: self.send_position(0.5,0,0,1,0,0), priority=1)
        #     time.sleep(0.1)
        self.send_command(lambda: self.set_flight_mode(6), priority=1)  
        time.sleep(1)

    # Function to start command thread 
    def start_command_thread(self):
        #command_thread = Thread(target=self.mission_start_v2, daemon=True)
        self.command_thread = Thread(target=self.do_testow, daemon=True)
        self.command_thread.start()
        self.command_thread.join()

# Example usage
if __name__ == "__main__":

    fcc = FC_Controller()
    detector = BallDetector()
    fcc.start_command_thread()

    del fcc



