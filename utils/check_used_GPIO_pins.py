import RPi.GPIO as GPIO

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# List of GPIO pins to check
pins = [4, 17, 18, 27, 22, 23, 24, 25, 5, 6, 12, 13, 19, 16, 26, 20, 21]
pins = [4, 17, 18, 27, 22, 23, 24, 25, 5, 6, 12, 13, 19, 16, 26, 20, 21]

print("Pin status (1 means in use, 0 means not in use):")
status = {}
for pin in pins:
    try:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        status[pin] = GPIO.input(pin)
    except Exception as e:
        status[pin] = 'Error'

for pin, state in status.items():
    print(f"GPIO {pin}: {state}")

# Cleanup GPIO
GPIO.cleanup()
