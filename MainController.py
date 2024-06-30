from GripperController import GripperController
from CameraController import CameraController
from CV import BallFinder
# import LandingController
#import FlightControllerInterface
import subprocess
import time
from time import sleep
import requests
import json

def run_mission():
    camera_controller_obj = CameraController()
    gripper_controller_obj = GripperController()
    pass

def find_balls_img(img):
    pass

def send_data_to_flask(data):
    # metoda wysyłanie danych do serwera
    url = 'http://127.0.0.1:5000/receive_data'
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, data=json.dumps(data), headers=headers)
    return response.json()

def test_camera_controller():
    # inicjalizacja obiektu
    camera_controller_obj = CameraController()
    
    # patrzy w dółsleep(1)
    print("kamera patrzy w dół")
    camera_controller_obj.set_angle(33)
    # sleep żeby miało czas się ruszyć
    sleep(0.5)
    # strzel fote
    print("zrobienie zdjęcia")
    camera_controller_obj.take_picture()

    #patrzy do przodu pod kątem lekko w dół
    print("kamera patrzy na skos")
    camera_controller_obj.set_angle(-25)
    sleep(0.5)
    print("zrobienie zdjęcia")
    camera_controller_obj.take_picture()

def test_gripper_controller():
    gripper_controller_obj = GripperController()
    
    #działa
    gripper_controller_obj.open_gripper(0.4)
    time.sleep(2)
    gripper_controller_obj.close_gripper(0.4)
    time.sleep(2)

    print("czytanie z czujnika odbiciowego")
    distance_from_sensor = gripper_controller_obj.get_distance_claw_sensor()
    print(distance_from_sensor)
    
def take_pictures_continously():
    
    
    camera_controller_obj = CameraController()
    camera_controller_obj.take_picture()
    
    # array_pic = camera_controller_obj.take_pictures_continously()
        
        # #działa# patrzy w dół
        # print("kamera patrzy w dół")
        # # camera_controller_obj.set_angle(33)
        # # sleep żeby miało czas się ruszyć
        # sleep(0.2)
        # # strzel fote
        # print("zrobienie zdjęcia")
        # camera_controller_obj.take_picture()

        # # patrzy do przodu pod kątem lekko w dół
        # print("kamera patrzy na skos")
        # # camera_controller_obj.set_angle(-25)
        # sleep(0.2)
        # print("zrobienie zdjęcia")
        # camera_controller_obj.take_picture()

    #test ruchu kamery i zrobienia zdjęcia
    # test_gripper_controller()
    

if __name__ == "__main__":
    
    
    # test_camera_controller()

    #take_pictures_continously()

    status = send_data_to_flask("duuupa")
    print(status)
    


    
