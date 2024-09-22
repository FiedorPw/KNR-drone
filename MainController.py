from GripperController import GripperController
from CV_Controller import BallDetector
from FC_Controller import FC_Controller
import time
from threading import Thread


#TODO:
# 1. [DONE] Handle keyboard interruption - stop threads and del objects when ctrl+c pushed
# 2. [DONE] Convert mission_phase methods from FCC to be executed here and be started in start_command_thread 
# 3. [DONE] Add attributes: isPhaseXDone; platform_X_position; platform_X_color
# 4. [DONE] Phase 2 should be done until platform_X_color list has [red, blue, purple] balls
# 5. Init somewhere reset_position and start_position so its here as global var, not locally in function

###### MISSION INIT VARIABLES ######
mission_thread = None

is_phase_one_done = False
is_phase_two_done = False
is_phase_three_done = False
is_phase_four_done = False
is_phase_five_done = False

platform_5_position = [52.12312312,13.32523423] #Mock data, also it is reset position but without alt parameter
barrel_position = [52.12314535,13.32522395] #Mock data
start_altitude = 6 # Starting takeoff altitude -6m

detected_balls = {} 
platform_positions = {}
required_balls = {'red','blue','purple'}

release_position = []

retry_attempts = 3 # Retries of cathcing ball in phase 4
####################################

############################################################################################################
######################### ALGORYTM MISJI - WERSJA 2 -> GPS: PLATFORM 5 AND BARREL ###########################
############################################################################################################

def mission_start_v2():
    fcc.telemetry_event.wait()
    fcc.telemetry_event.clear()

    fcc.send_command(lambda: fcc.set_flight_mode(0), priority=1) 
    time.sleep(1)
    fcc.send_command(lambda: fcc.arm_disarm(1), priority=1)  
    time.sleep(1)
    fcc.send_command(lambda: fcc.set_flight_mode(4), priority=1)  
    time.sleep(1)
    fcc.send_command(lambda: fcc.takeoff(start_altitude), priority=1) 
    time.sleep(5)

    fcc.mission_phase_one()
    time.sleep(1)
    fcc.mission_phase_two()
    time.sleep(1)
    fcc.mission_phase_three()
    time.sleep(1)
    fcc.mission_phase_four()
    time.sleep(1)
    fcc.mission_phase_five()

# FLY TO CENTER OF MISSION AREA AND DETECT 9 PLATFORMS
def mission_phase_one():
    while not is_phase_one_done:
        fcc.send_command(lambda: fcc.reposition_copter(platform_5_position[0],platform_5_position[1],start_altitude,2,0), priority=1)
        
        while not detector.large_contours: #while detected platforms == 0 (none detected)
            fcc.send_command(lambda: fcc.change_altitude(fcc.latest_telemetry["Altitude"]+1,'up',1), priority=1)
            time.sleep(0.1)

        while not len(detector.large_contours) == 9: # while not all platforms are detected, add altitude
            fcc.send_command(lambda: fcc.reposition_copter(platform_5_position[0],platform_5_position[1],0,1,0), priority=1)
            fcc.send_command(lambda: fcc.change_altitude(fcc.latest_telemetry["Altitude"]+1,'up',1), priority=1)
            time.sleep(0.1)

        fcc.reset_position = [fcc.latest_telemetry["Latitude"],fcc.latest_telemetry["Longitude"],fcc.latest_telemetry["Altitude"]] # Can be start safe starting position for another phases
        is_phase_one_done = True 


# GET POSITION OF BALLS OF INTEREST
def mission_phase_two():
    reset_lat, reset_lon, reset_alt = fcc.reset_position

    while not is_phase_two_done:
        # TODO IF NOT ALL BALLS ARE FOUND -> CHECK ONLY PLATFORMS WHERE BALLS WERENT DETECTED
        for platform_num in range(1, 10):
            while len(detector.large_contours) != 9: 
                # fcc.send_command(lambda: fcc.send_global_position(reset_lat, reset_lon, reset_alt, 2, 2, 2), priority=1)
                fcc.send_command(lambda: fcc.reposition_copter(reset_lat, reset_lon, reset_alt,2,0), priority=1)
                time.sleep(0.1)

            #add tracking in CV (in TODO)
            while not detector.is_platform_close:
                fcc.send_command(lambda: fcc.navigate_to_target('platform', platform_num), priority=1)
                time.sleep(0.1)

            while fcc.latest_telemetry["Altitude"] > fcc.start_position[2] + 2:
                fcc.send_command(lambda: fcc.change_altitude(fcc.latest_telemetry["Altitude"]-1,'down',1), priority=1)
                time.sleep(0.1)
                fcc.send_command(lambda: fcc.navigate_to_target('platform'), priority=1)
                time.sleep(0.1)
            
            # IF BALL NOT DETECTED -> NAVIGATE_TO_TARGET(TARGET)
            ball_color = detector.detect_ball_color()
            if ball_color:
                detected_balls[platform_num] = ball_color
                platform_positions[platform_num] = [fcc.latest_telemetry["Latitude"],fcc.latest_telemetry["Longitude"],fcc.latest_telemetry["Altitude"]]

            print(f"Detected ball on platform {platform_num}: {ball_color}")
            detected_colors = set(detected_balls.values())      
            if 'red' in detected_colors and  'blue' in detected_colors and 'purple' in detected_colors:   
                print("All required colors detected: red, blue, purple")
                is_phase_two_done = True  
                break 

        if is_phase_two_done:
            # fcc.send_command(lambda: fcc.send_global_position(reset_lat, reset_lon, reset_alt, 2, 2, 2), priority=1)
            fcc.send_command(lambda: fcc.reposition_copter(reset_lat, reset_lon, reset_alt,2,0), priority=1)
            break 
    print("Detected balls on platforms:", detected_balls)

# GET POSITION OF BARREL AND ALTITUDE OF PAYLOAD DROP
def mission_phase_three():
    reset_lat, reset_lon, reset_alt = fcc.reset_position

    while not is_phase_three_done:
        while not detector.is_barrel_close: 
            fcc.send_command(lambda: fcc.reposition_copter(barrel_position[0], barrel_position[1], reset_alt,2,0), priority=1)
            time.sleep(0.1)

        while not detector.is_barrel_size_enough: 
            fcc.send_command(lambda: fcc.change_altitude(fcc.latest_telemetry["Altitude"]-0.5,'down',1), priority=1)
            time.sleep(0.1)
            fcc.send_command(lambda: fcc.navigate_to_target('barrel'), priority=1) #TODO - CV barrel 
            time.sleep(0.1)
        
        if detector.is_barrel_size_enough and detector.is_barrel_close:
            release_position = [fcc.latest_telemetry["Latitude"],fcc.latest_telemetry["Longitude"],fcc.latest_telemetry["Altitude"]] #TODO - release pos as global var
            fcc.send_command(lambda: fcc.reposition_copter(reset_lat, reset_lon, reset_alt,2,0), priority=1)
            is_phase_three_done = True

# MAIN TODO -> HOW TO KNOW DRONE IS STANDING IN GROUND? NOT ACC_Z, RATHER NOT ALTITUDE, PROLLY BALL SIZE???
def mission_phase_four():
    reset_lat, reset_lon, reset_alt = fcc.reset_position

    while not is_phase_four_done:
        for index, color in enumerate(detected_balls):
            if color in required_balls:
                lat,lon,alt = platform_positions[index]
                # Reposition the copter to the target platform position
                print(f"Repositioning to {color} ball at position lat: {lat}, lon: {lon}, alt: {alt}")
            
                # Retry loop
                for attempt in range(retry_attempts):
                    while not detector.is_target_close:
                        fcc.send_command(lambda: fcc.reposition_copter(lat, lon, alt, 2, 0), priority=1)
                        time.sleep(0.1)
                    while not detector.is_on_ground:
                        fcc.send_command(lambda: fcc.change_altitude(fcc.latest_telemetry["Altitude"]-0.2,'down',1), priority=1)
                        time.sleep(0.1)
                        fcc.send_command(lambda: fcc.navigate_to_target('target'), priority=1)
                        time.sleep(0.1)

                    #TODO - ADD GRIPPER CLOSE METHOD
                    if detector.is_on_ground and detector.is_target_close:
                        gripper.close_gripper()

                    # TODO - DISTANCE SENSOR (is_ball_caught)
                    if gripper.is_ball_caught:
                        print(f"{color.capitalize()} ball successfully caught!")
                        break

                    if attempt == retry_attempts - 1:
                        print(f"Failed to catch the {color} ball after {retry_attempts} attempts. Moving to the next target.")
                        
                # NEED TO BE CHANGED - HAVENT GOT VISION THERE
                while not detector.is_barrel_close:
                    fcc.send_command(lambda: fcc.reposition_copter(release_position[0], release_position[1], release_position[2],2,0), priority=1)
                    time.sleep(0.1)
                    fcc.send_command(lambda: fcc.navigate_to_target('barrel'), priority=1)
                    time.sleep(0.1)
                
                gripper.open_gripper()

        fcc.send_command(lambda: fcc.reposition_copter(reset_lat, reset_lon, reset_alt,2,0), priority=1)
        is_phase_four_done = True

#RETURN TO HOME AND DISARM
def mission_phase_five():
    while not is_phase_five_done:
        while not detector.is_platform_close:
            fcc.send_command(lambda: fcc.reposition_copter(fcc.start_position[0], fcc.start_position[1], fcc.start_position[2]+2,2,0), priority=1)
            time.sleep(0.1)
        fcc.send_command(lambda: fcc.set_flight_mode(6), priority=1)
        time.sleep(4)
        is_phase_five_done = True

# def print_balls(balls):
#     for ball in balls:
#         print("red: center: ", balls['red'].center,"size: " ,balls['red'].size)  # red center:  (99, 306)
#         print("blue: center: ", balls['blue'].center,"size: " ,balls['blue'].size)  # red center:  (99, 306)
#         print("purple: center: ", balls['purple'].center,"size: ", balls['purple'].size)  # red center:  (99, 306)

# def detect_balls():
#     # Initialize the BallDetector
#     detector = BallDetector()
#     # URL of your MJPEG stream
#     mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'  # Replace with the actual URL
#     # Fetch and process a single JPEG frame
#     frame = detector.fetch_frame(mjpeg_url)
#     balls = detector.process_frame(frame)
#     print("Detected balls:")
#     print_balls(balls)

# def test_gripper_controller():
#     gripper_controller_obj = GripperController()

#     #działa
#     gripper_controller_obj.open_gripper(0.4)
#     time.sleep(2)
#     gripper_controller_obj.close_gripper(0.4)
#     time.sleep(2)

#     print("czytanie z czujnika odbiciowego")
#     distance_from_sensor = gripper_controller_obj.get_distance_claw_sensor()
#     print(distance_from_sensor)

def start_mission_thread():
    mission_thread = Thread(target=mission_start_v2, daemon=True)
    mission_thread.start()
    mission_thread.join()

if __name__ == "__main__":
    try:
        fcc = FC_Controller()
        detector = BallDetector()
        gripper = GripperController()
        start_mission_thread()

    except KeyboardInterrupt:
        print("\nManual interruption detected. Exiting the program safely.")
    finally:

        print("Program has been safely terminated.")
