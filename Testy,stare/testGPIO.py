from gpiozero import DigitalOutputDevice
from time import sleep

pinForward = 24
pinBackward = 23

forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)

try:
    forward.on()
    print("Forward pin is on")
    sleep(2)
    forward.off()
    print("Forward pin is off")

    backward.on()
    print("Backward pin is on")
    sleep(2)
    backward.off()
    print("Backward pin is off")
finally:
    forward.close()
    backward.close()
