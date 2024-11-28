import time
from rpi_ws281x import PixelStrip, Color

# Konfiguracja LED
LED_COUNT = 10        # Liczba diod LED (5 na każdy pasek, łącznie 10)
LED_PIN = 18          # GPIO pin
LED_FREQ_HZ = 800000  # Częstotliwość w Hz
LED_DMA = 10          # Kanał DMA do użycia
LED_BRIGHTNESS = 255  # Jasność (0-255)
LED_INVERT = False    # Odwrócenie sygnału
LED_CHANNEL = 0       # Kanał dla ws281x

# Inicjalizacja paska LED
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

# Funkcja zapalająca diody po kolei
def light_up_leds(strip):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(200, 0, 0))  # Zapalanie na czerwono
        strip.show()
        time.sleep(0.2)  # Krótkie opóźnienie między diodami
    time.sleep(0.5)  # Opóźnienie przed wygaszeniem
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))  # Gaszenie diod
    strip.show()

try:
    # Program działa w nieskończonej pętli
    while True:
        light_up_leds(strip)

except KeyboardInterrupt:
    # Wyczyść ustawienia i wyłącz diody przed zakończeniem programu
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))  # Wyłącz diody (czarny kolor)
    strip.show()
