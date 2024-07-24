import time
from pymavlink import mavutil

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

# Example usage
if __name__ == "__main__":
    arm_drone()
    time.sleep(5)  # Wait for 5 seconds
    disarm_drone()
    # takeoff(10)    # Take off to 10 meters altitude
    # time.sleep(10) # Hover for 10 seconds
