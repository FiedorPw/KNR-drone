from time import sleep
import sys
import os
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# TODO - implement distance sensor logic and create is_ball_caught flag as class attribute

# Zadeklarowanie pinów dla mostka H
pinForward = 24  # Pin do przodu (GPIO 24)
pinBackward = 23 # Pin do tylu (GPIO 23)
pinPWM = 12     # Pin PWM (GPIO 12)

# Ustawienie pinów
forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)
speed = PWMOutputDevice(pinPWM)

class GripperController:
    def __init__(self):
        pass
    
    def open_gripper(self, speed_value):
        """Jedź do przodu z określoną prędkością."""
        print("otwieranie grippera")
        backward.off()
        forward.on()
        speed.value = speed_value  # Ustaw prędkość (0.0 do 1.0)

    def close_gripper(self, speed_value):
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
    
    def test_importu(self):
        print("test importu")

if __name__ == "__main__":
    try:
        gripper = GripperController()

        # Przykładowe użycie funkcji
        gripper.open_gripper(0.5)  # Jedź do przodu z połową prędkości
        sleep(2.5)  # Czekaj 2 sekundy
        gripper.motor_stop()  # Zatrzymaj silnik

    finally:
        # `gpiozero` automatycznie zwalnia piny, ale dodanie poniższego kodu
        # może pomóc w niektórych przypadkach, aby upewnić się, że piny są zwolnione
        forward.close()
        backward.close()
        speed.close()
