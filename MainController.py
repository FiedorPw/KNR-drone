#
# from GripperController import GripperController
# from CameraController import CameraController
from FC_controller import FC_Controller
from CV_Controller import BallDetector


import subprocess
import time
from time import sleep
import requests
import json

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

def test_FCC():

    FCC_obj = FC_Controller()

    GPSData = FCC_obj.get_gps_position()
    print(GPSData)
    pass

def detect_balls():
    # Initialize the BallDetector
    detector = BallDetector()

    # URL of your MJPEG stream
    mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'  # Replace with the actual URL

    # Fetch and process a single JPEG frame
    frame = detector.fetch_frame(mjpeg_url)
    balls = detector.process_frame(frame)

    print("Detected balls:")
    for color, ball in balls.items():
        print(f"{color} ball - Center: {ball.center}, Size: {ball.size}")

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




if __name__ == "__main__":

    # test_FCC()
    # test_camera_controller()
    detect_balls()
    #take_pictures_continously()

    # print(status)
