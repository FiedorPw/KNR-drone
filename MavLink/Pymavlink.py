from pymavlink import mavutil
import time
import math

# Establish connection to the flight controller
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
# SITL
# master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

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

def set_flight_mode(mode_number):
    if mode_number not in mode_mapping:
        print(f"Unknown mode number: {mode_number}")
        print("Valid mode numbers:", list(mode_mapping.keys()))
        return

    mode = mode_mapping[mode_number]
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print(f"Flight mode set to {mode} (mode number {mode_number})")

def arm_disarm(arm):
    if arm:
        master.arducopter_arm()
        print("Arming motors")
        master.motors_armed_wait()
        print("Motors armed")
    else:
        master.arducopter_disarm()
        print("Disarming motors")
        master.motors_disarmed_wait()
        print("Motors disarmed")

def takeoff(altitude):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    msg = master.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)
    
    print(f"Takeoff command sent for altitude {altitude} meters")

def get_gps_position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
    else:
        print("No GPS position data received")

def get_attitude():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw = math.degrees(msg.yaw)
        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
    else:
        print("No attitude data received")

def get_battery_status():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
    if msg:
        voltage = msg.voltages[0] / 1000.0
        current = msg.current_battery / 100.0
        print(f"Voltage: {voltage}V, Current: {current}A")
    else:
        print("No battery status data received")

def get_flight_mode():
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        print(f"Current flight mode: {mode}")
        return mode
    return None

def get_all_telemetry():
    get_gps_position()
    get_attitude()
    get_battery_status() 

def set_motor_speed(speed_percent):
    """Set motor speed from 0 to 100%."""
    speed = speed_percent / 100.0
    master.mav.set_actuator_control_target_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        0,  # group_mlx (actuator group)
        [speed] * 8,  # controls (throttle values for motors 1-8)
        0  # target_system
    )

def gradual_motor_speed_change():
    """Gradually change motor speed from 0% to 20% to 40% to 20% to 0% with 2-second intervals."""
    speeds = [0, 20, 40, 20, 0]
    for speed in speeds:
        set_motor_speed(speed)
        print(f"Setting motor speed to {speed}%")
        time.sleep(2)

def send_position_body_frame(target_x_body, target_y_body, target_z_body, velocity_x=0, velocity_y=0, velocity_z=0):
    # Get the current attitude of the drone (yaw, pitch, roll)
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        yaw = msg.yaw  # Yaw in radians
    else:
        print("Failed to receive attitude data.")
        return
    
    # Create a rotation matrix for yaw transformation
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Apply the rotation matrix to convert body frame to NED frame
    # target_x_ned = cos_yaw * target_x_body - sin_yaw * target_y_body
    # target_y_ned = sin_yaw * target_x_body + cos_yaw * target_y_body
    # target_z_ned = target_z_body  # Z remains the same (vertical axis)

    target_x_ned = target_x_body
    target_y_ned = target_y_body
    target_z_ned = target_z_body


        

    # Get the current local position in NED frame
    current_position = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    if current_position:
        start_x = current_position.x
        start_y = current_position.y
        start_z = current_position.z

        target_x_ned += start_x
        target_y_ned += start_y
        target_z_ned += start_z

        # Send position and velocity target in NED frame
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (positions and velocities enabled)
            target_x_ned, target_y_ned, target_z_ned,  # x, y, z positions in meters (NED frame)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            yaw, 0  # Yaw in radians, with no yaw rate
        )
        print(f"Moving to position (x: {target_x_ned}, y: {target_y_ned}, z: {target_z_ned}) in NED frame, converted from body frame (x: {target_x_body}, y: {target_y_body}, z: {target_z_body})")

    else:
        print("Failed to receive current position data.")




def fly_square():
    # Ustal prędkość na 0.5 m/s (dostosuj do potrzeb)
    velocity = 1
    altitude = 2

    # Przesuń do przodu o 0.5 metra
    send_position_body_frame(2, 0, altitude, velocity_x=velocity)
    time.sleep(3)

    # Przesuń w lewo o 0.5 metra
    send_position_body_frame(0, -2, altitude, velocity_y=-velocity)
    time.sleep(3)

    # Przesuń do tyłu o 0.5 metra
    send_position_body_frame(-2, 0, altitude, velocity_x=-velocity)
    time.sleep(3)

    # Przesuń w prawo o 0.5 metra
    send_position_body_frame(0, 2, altitude, velocity_y=velocity)
    time.sleep(3)

def fly_straight():
    # Ustal prędkość na 0.5 m/s (dostosuj do potrzeb)
    velocity = 1
    altitude = 2

    # Przesuń do przodu o 0.5 metra
    send_position_body_frame(3, 0, altitude, velocity_x=velocity)
    time.sleep(4)

    # Przesuń do tyłu o 0.5 metra
    send_position_body_frame(-3, 0, altitude, velocity_x=-velocity)
    time.sleep(4)

def mission_one():

        set_flight_mode(0)
        time.sleep(1)

        arm_disarm(1)
        time.sleep(0.1)  # Wait for 2 seconds

        set_flight_mode(4)
        time.sleep(3)
        
        takeoff(2.5)

        time.sleep(3)


        # fly_square()

        # takeoff(2.5)

        fly_straight()

        time.sleep(2)

        set_flight_mode(8)
        time.sleep(4)  


if __name__ == "__main__":
    try:
        set_flight_mode(0)
        time.sleep(2)
        
        mission_one()
        

        # # Continuous loop for telemetry data and flight mode monitoring
        # while True:
        #     # Get telemetry data
        #     print("Getting telemetry data...")
        #     get_gps_position()
        #     get_attitude()
        #     get_battery_status()

        #     # Monitor flight mode changes
        #     print("Monitoring flight mode changes...")
        #     get_flight_mode()

        arm_disarm(0)  

    except Exception as e:
        print(f"An error occurred: {e}")    
    finally:
        # set_flight_mode(0)
        # time.sleep(2)
        # Disarm the drone
        set_flight_mode(8)