import RPi.GPIO as GPIO
import time

# Ustawienia
GPIO_PIN = 13

# Ustawienie trybu numeracji pinów (BCM) i pinu jako wyjście
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)

# Ustawienie stanu pinu na HIGH
GPIO.output(GPIO_PIN, GPIO.HIGH)

# Poczekaj chwilę, aby pin pozostał w stanie HIGH
time.sleep(5)

# Wyczyść ustawienia GPIO
GPIO.cleanup()
