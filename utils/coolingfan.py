import RPi.GPIO as GPIO
import time

# Ustaw pin GPIO, który jest podłączony do wentylatora
FAN_PIN = 18

# Inicjalizuj GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)
GPIO.output(FAN_PIN, False)

# Funkcja do kontrolowania wentylatora
def control_fan(temperature):
    if temperature > 30:
        GPIO.output(FAN_PIN, True)  # Włącz wentylator, gdy temperatura przekroczy 50 stopni
    else:
        GPIO.output(FAN_PIN, False)  # Wyłącz wentylator, gdy temperatura spadnie poniżej 50 stopni

try:
    while True:
        # Tutaj dodaj kod do odczytu temperatury Raspberry Pi
        # Użyj instrukcji warunkowej, aby sterować wentylatorem na podstawie odczytanej temperatury
        # np. temp = read_temperature()
        # control_fan(temp)
        
        # Symulacja odczytu temperatury co 5 sekund
        temperature = 45  # Symulacja temperatury 45 stopni
        print("Temperature:", temperature)
        control_fan(temperature)
        time.sleep(5)

except KeyboardInterrupt:
    GPIO.cleanup()  # W przypadku przerwania zatrzymujemy skrypt i czyszczymy GPIO
