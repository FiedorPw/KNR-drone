import time
from rpi_ws281x import PixelStrip, Color

# czerwony do 5v niebieski do masy
# Konfiguracja parametrów
LED_COUNT = 4         # Liczba diod PL9823
LED_PIN = 12          # GPIO pin (zielony kabel) (musi wspierać PWM, w Raspberry Pi GPIO 12)
LED_FREQ_HZ = 800000  # Częstotliwość sygnału PWM
LED_DMA = 10          # Kanał DMA
LED_BRIGHTNESS = 255  # Jasność (0-255)
LED_INVERT = False    # Odwrócenie sygnału (jeśli używasz NPN transistora)

# Inicjalizacja diod
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
strip.begin()

def setColor(color):
    for i in range(LED_COUNT):
        strip.setPixelColor(i, color)
    strip.show()

try:
    while True:
        setColor(Color(255, 0, 0))  # Zielony
        time.sleep(1)
        setColor(Color(0, 0, 255))  # Wyłączenie diod
        time.sleep(1)
except KeyboardInterrupt:
    setColor(Color(0, 0, 0))  # Wyłączenie diod przy przerwaniu programu
 
