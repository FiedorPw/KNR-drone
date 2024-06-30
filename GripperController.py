


# DZIAŁAJĄCY KOD GRIPPERA JEST W /manual_control



from time import sleep
import sys

import os
import RPi.GPIO as GPIO
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
#from DFRobot_TMF8x01 import DFRobot_TMF8801 as tof
# from lib.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof
from gpiozero import PWMOutputDevice, DigitalOutputDevice


# Zadeklarowanie pinów dla mostka H
pinForward = 24  # Pin do przodu (GPIO 17)
pinBackward = 23 # Pin do tylu (GPIO 27)
pinPWM = 12     # Pin PWM (GPIO 22)

# Ustawienie pinów
forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)
speed = PWMOutputDevice(pinPWM)

#tof = tof(enPin = -1, intPin = -1, bus_id = 1)

# # Ustawienia pinów
# pin_pwm = 12  # pin PWM (można zmienić na dowolny obsługujący PWM)
# pin_in1 = 23  # pin IN1 (można zmienić)
# pin_in2 = 24  # pin IN2 (można zmienić)

# # Ustawienia GPIO
# GPIO.setmode(GPIO.BCM)  # Używamy numeracji BCM
# GPIO.setup(pin_pwm, GPIO.OUT)
# GPIO.setup(pin_in1, GPIO.OUT)
# GPIO.setup(pin_in2, GPIO.OUT)

# # Ustawienia PWM
# pwm_freq = 1000  # Częstotliwość PWM w Hz
# pwm = GPIO.PWM(pin_pwm, pwm_freq)
# pwm.start(0)  # Uruchom PWM z wypełnieniem 0%

class GripperController:
    def __init__(self):
        pass
    
    def open_gripper(self,speed_value):
        """Jedź do przodu z określoną prędkością."""
        print("otwieranie grippera")
        backward.off()
        forward.on()
        speed.value = speed_value  # Ustaw prędkość (0.0 do 1.0)

    def close_gripper(self,speed_value):
        """Jedź do tyłu z określoną prędkością."""
        print("zamykanie grippera")
        forward.off()
        backward.on()
        speed.value = speed_value  # Ustaw prędkość (0.0 do 1.0)

    def motor_stop(self):
        """Zatrzymaj silnik."""
        print("gripper silnik stop")
        forward.off()
        backward.off()
        speed.value = 0
    
        

    #def get_distance_claw_sensor(self, print_data=False):
        ##tof.begin()
        #time.sleep(1)
        # while tof.begin() != 0:
        #     print("Initialization failed")
        # print("Initialization done.")
        
        # print("Software Version: ", tof.get_software_version())
        # print("Unique ID: %X" % tof.get_unique_id())
        # print("Model: ", tof.get_sensor_model())
        
        #for i in range(200):
            #tof.start_measurement(calib_m=tof.eMODE_CALIB, mode=tof.ePROXIMITY)
            #tof.is_data_ready() == True 
            #if(print_data):
                #print("Distance = %d mm" % tof.get_distance_mm())    
        #while True:    
        #if tof.is_data_ready():    
        #return tof.get_distance_mm()
        

    def test_importu(self):
        print("test importu")


if __name__ == "__main__":
    try:
        gripper = GripperController()

        # Przykładowe użycie funkcji
        gripper.open_gripper(0.5)  # Jedź do przodu z połową prędkości
        
        time.sleep(2.5)  # Czekaj 2 sekundy
        #gripper.close_gripper(0.4)  # Jedź do tyłu z mniejszą prędkością
        #time.sleep(2)  # Czekaj 2 sekundy
        gripper.motor_stop()  # Zatrzymaj silnik
        
        #gripper.get_distance_claw_sensor(print_data=True)
        
    finally:
        GPIO.cleanup()
