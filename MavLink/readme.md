# komenda do odczytywania z rasberki telemtri z FC hujowa
sudo minicom -b 115200 -o -D /dev/ttyAMA0

# działający interfejs w terminalu po mavlinku
python3 /usr/local/bin/mavproxy.py --master=/dev/ttyACM0 --baudrate 57600 --aircraft MyCopter
