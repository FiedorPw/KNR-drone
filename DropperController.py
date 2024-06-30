from gpiozero import AngularServo
from time import sleep
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
from PIL import Image
import io
import os


'''
Servo kable:
 niebieski GND
 czerwony 5V 
 zielony na GPIO 18
'''
# git
START_POSITION = 60
POSITION_1 = 30
POSITION_2 = 0
POSITION_3 = 90
POSITION_4 = -85
# nie git



Device.pin_factory = PiGPIOFactory()

class DropperController:
    def __init__(self):
        self.camera_controller_obj = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.min_angle = -85
        self.max_angle = 200
        self.current_angle = 0  # Assume starting at 0 degrees
        

    def go_up(self):
        if self.current_angle - 0.5 >= self.min_angle:
            self.current_angle -= 0.5
        else:
            self.current_angle = self.min_angle
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle

    

    def set_angle(self, x):
        if x < self.min_angle:
            self.current_angle = self.min_angle
        elif x > self.max_angle:
            self.current_angle = self.max_angle
        else:
            self.current_angle = x
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        return self.current_angle
    
    def max_positions_test(self):
        while True:
            self.set_angle(33)
            sleep(1)
            self.set_angle(-25)
            sleep(1)
        #sleep(2)

    def find_angle(self):
        while True:
            print("start position")
            self.set_angle(START_POSITION)
            sleep(5)

            print("position 1")
            self.set_angle(POSITION_1)
            sleep(3)
            
            print("position 2")
            self.set_angle(POSITION_2)
            sleep(3)

            print("position 3")
            self.set_angle(POSITION_3)
            sleep(3)

            print("position 4")
            self.set_angle(POSITION_4)
            sleep(3)

            # for i in range(-20,20,1):
            #     angle = i * 5 
            #     print(angle)
            #     self.set_angle(angle)
            #     sleep(1.2)
                


if __name__ == "__main__":
    camera_controller_obj = DropperController()
    # camera_controller_obj.max_positions_test()
    camera_controller_obj.find_angle()
#leci 4
# 
# 
# 
    
    
