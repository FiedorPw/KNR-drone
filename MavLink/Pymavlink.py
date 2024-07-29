from pymavlink import mavutil
import time

# Establish connection to the flight controller
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Function to arm the drone
def arm_drone():
    master.arducopter_arm()
    print("Arming motors")
    master.motors_armed_wait()
    print("Motors armed")

# Function to disarm the drone
def disarm_drone():
    master.arducopter_disarm()
    print("Disarming motors")
    master.motors_disarmed_wait()
    print("Motors disarmed")

# Function to send a takeoff command
def takeoff(altitude):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    print(f"Takeoff command sent for altitude {altitude} meters")

# Function to get GPS position
def get_gps_position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
    else:
        print("No GPS position data received")

# Function to get attitude
def get_attitude():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
    else:
        print("No attitude data received")

# Function to get battery status
def get_battery_status():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
    if msg:
        voltage = msg.voltages[0] / 1000.0
        current = msg.current_battery / 100.0
        print(f"Voltage: {voltage}V, Current: {current}A")
    else:
        print("No battery status data received")

# Example usage
if __name__ == "__main__":
    arm_drone()
    time.sleep(2)  # Wait for 2 seconds
    print("Takeoff start")
    takeoff(2)    # Take off to 2 meters altitude
    time.sleep(2) # Hover for 2 seconds
    
    print("Getting telemetry data...")
    get_gps_position()
    get_attitude()
    get_battery_status()
    
    disarm_drone()
