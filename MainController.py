# from GripperController import GripperController
# from CameraController import CameraController
from CV_Controller import BallDetector
from FC_Controller import FC_Controller
import time

#TODO:
# 1. Handle keyboard interruption - stop threads and del objects when ctrl+c pushed
# 2. Convert mission_phase methods from FCC to be executed here and be started in start_command_thread 


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

# def mission_phase_one():
#     fcc.telemetry_event.wait()
#     fcc.telemetry_event.clear()
#     fcc.send_command(lambda: fcc.set_flight_mode(0), priority=1) 
#     time.sleep(1)
#     fcc.send_command(lambda: fcc.arm_disarm(1), priority=1)  
#     time.sleep(1)
#     fcc.send_command(lambda: fcc.set_flight_mode(4), priority=1)  
#     time.sleep(2)
#     fcc.send_command(lambda: fcc.takeoff(5), priority=1) 
#     time.sleep(5)
#     fcc.send_command(lambda: fcc.send_position(2, 0, 0), priority=1) 
#     time.sleep(4)

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


if __name__ == "__main__":
    # try:

        fcc = FC_Controller()
        detector = BallDetector()
        
        fcc.start_command_thread()

        # run_mission()

        # test_camera_controller()
        # detect_balls()
        #take_pictures_continously()
        # print(status)
    # except KeyboardInterrupt:
    #     print("\nManual interruption detected. Exiting the program safely.")
    # finally:

    #     print("Program has been safely terminated.")
