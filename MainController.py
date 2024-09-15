# from GripperController import GripperController
# from CameraController import CameraController
# from FC_Controller import FC_Controller
from CV_Controller import BallDetector
from FC_Controller import FC_Controller

import subprocess
import time
import numpy as np
import requests

def run_mission():
    # camera_controller_obj = CameraController()
    # gripper_controller_obj = GripperController()


    # algorytm 1 - znajdywanie piłek
    #
    # przeleć po wszyskich 9 na wysokości 1,5 - 2,5m(do sprawdzenia) stań nad każdą
    #   jeżeli nie znaleziono już trzech: leć nad n-tą piłke(zhardkodowane lokalizacje)
    #   jeżeli znaleziona piłka i nie znaleziono wcześniej dodaj jej lokalizacje do dict np 'red': (52,1234,21,34234)
    #   jeżeli len(balls) == 3 (znaleziono wszyskie)
    #       leć nad niebieską wykonaj alg2
    #       leć nad czerwoną wykonaj alg2
    #       leć nad fioletową wykonaj alg2

    # algorytm 2 - zejścia
    #   1. pobierz dane z telemetri
    #   2. sprawdz wizje, przekaż jej wysokość, znajdz piłki, znajdz kierunek
    #   3. skoryguj pozycje
    #   4. jak wylądował zamknij łape
    #   5. leć nad beczkę(cv skoryguj ją albo nie)
    #   6. zrzuć

    pass

def print_balls(balls):
    for ball in balls:
        print("red: center: ", balls['red'].center,"size: " ,balls['red'].size)  # red center:  (99, 306)
        print("blue: center: ", balls['blue'].center,"size: " ,balls['blue'].size)  # red center:  (99, 306)
        print("purple: center: ", balls['purple'].center,"size: ", balls['purple'].size)  # red center:  (99, 306)

def detect_balls():
    # Initialize the BallDetector
    detector = BallDetector()

    # URL of your MJPEG stream
    mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'  # Replace with the actual URL

    # Fetch and process a single JPEG frame
    frame = detector.fetch_frame(mjpeg_url)
    balls = detector.process_frame(frame)

    print("Detected balls:")
    print_balls(balls)

# def test_camera_controller():
#     # inicjalizacja obiektu
#     camera_controller_obj = CameraController()

#     # patrzy w dółsleep(1)
#     print("kamera patrzy w dół")
#     camera_controller_obj.set_angle(33)
#     # sleep żeby miało czas się ruszyć
#     sleep(0.5)
#     # strzel fote
#     print("zrobienie zdjęcia")
#     camera_controller_obj.take_picture()

#     #patrzy do przodu pod kątem lekko w dół
#     print("kamera patrzy na skos")
#     camera_controller_obj.set_angle(-25)
#     sleep(0.5)
#     print("zrobienie zdjęcia")
#     camera_controller_obj.take_picture()

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


#    def navigate_to_target(self, pipe_path, target_z=2, max_velocity=2.0):
#         """
#         Navigate drone based on velocity direction from named pipe.
#         pipe_path: Path to the named pipe.
#         """
#         while True:
#             # Read velocity vector from the pipe
#             vector = self.read_vector_from_pipe(pipe_path)
#             if vector:
#                 velocity_x, velocity_y, velocity_z = vector

#                 # Normalize and limit velocity
#                 velocity_magnitude = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
#                 if velocity_magnitude > max_velocity:
#                     velocity_x = (velocity_x / velocity_magnitude) * max_velocity
#                     velocity_y = (velocity_y / velocity_magnitude) * max_velocity
#                     velocity_z = (velocity_z / velocity_magnitude) * max_velocity

#                 # Send position and velocity commands to the drone
#                 self.send_position(0, 0, target_z, velocity_x, velocity_y, velocity_z)

#                 print(f"Navigating with velocity vector ({velocity_x}, {velocity_y}, {velocity_z})")

#             else:
#                 print("Failed to read vector from pipe.")

#             # Add a small delay to avoid overwhelming the system
#             time.sleep(0.5)



if __name__ == "__main__":

    fcc = FC_Controller()
    detector = BallDetector()
    
    fcc.start_command_thread()

    # test_camera_controller()
    # detect_balls()
    #take_pictures_continously()


    # print(status)
