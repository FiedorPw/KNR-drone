import time
from rpi_ws281x import PixelStrip, Color

# Konfiguracja LED dla pierwszego paska
LED_COUNT_1 = 10        # Liczba diod LED (5 na każdy pasek, łącznie 10)
LED_PIN_1 = 18          # GPIO pin dla pierwszego paska
LED_FREQ_HZ = 800000    # Częstotliwość w Hz
LED_DMA = 10            # Kanał DMA do użycia
LED_BRIGHTNESS = 255    # Jasność (0-255)
LED_INVERT = False      # Odwrócenie sygnału
LED_CHANNEL_1 = 0       # Kanał dla ws281x

# Konfiguracja LED dla drugiego paska
LED_COUNT_2 = 10        # Liczba diod LED (5 na każdy pasek, łącznie 10)
LED_PIN_2 = 13          # Zmiana GPIO pin dla drugiego paska na obsługiwany pin
LED_CHANNEL_2 = 1       # Kanał dla ws281x

# Inicjalizacja pasków LED
strip1 = PixelStrip(LED_COUNT_1, LED_PIN_1, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL_1)
strip2 = PixelStrip(LED_COUNT_2, LED_PIN_2, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL_2)

# Rozpoczęcie pracy z paskami LED
strip1.begin()
strip2.begin()

# Funkcja zapalająca diody po kolei na obu paskach
def light_up_leds(strip1, strip2):
    for i in range(strip1.numPixels()):
        strip1.setPixelColor(i, Color(200, 0, 0))  # Czerwony na pierwszym pasku
        strip2.setPixelColor(i, Color(0, 200, 0))  # Zielony na drugim pasku
        strip1.show()
        strip2.show()
        time.sleep(0.2)  # Krótkie opóźnienie między diodami
    time.sleep(0.5)  # Opóźnienie przed wygaszeniem
    for i in range(strip1.numPixels()):
        strip1.setPixelColor(i, Color(0, 0, 0))  # Gaszenie diod na pierwszym pasku
        strip2.setPixelColor(i, Color(0, 0, 0))  # Gaszenie diod na drugim pasku
    strip1.show()
    strip2.show()

try:
    # Program działa w nieskończonej pętli
    while True:
        light_up_leds(strip1, strip2)

except KeyboardInterrupt:
    # Wyczyść ustawienia i wyłącz diody przed zakończeniem programu
    for i in range(strip1.numPixels()):
        strip1.setPixelColor(i, Color(0, 0, 0))  # Wyłącz diody na pierwszym pasku
        strip2.setPixelColor(i, Color(0, 0, 0))  # Wyłącz diody na drugim pasku
    strip1.show()
    strip2.show()
