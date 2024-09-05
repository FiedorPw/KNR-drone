# działający interfejs w terminalu po mavlinku
python3 /usr/local/bin/mavproxy.py --master=/dev/ttyACM0 --baudrate 57600 --aircraft MyCopter

set param ARMING_CHECK 0

# Skrypt do zbierania danych telemetrycznych - paramLogger.py

# Skrypt do wysyłania danych w formacie .json poporzez rsyslog - sendTelemetry.py
