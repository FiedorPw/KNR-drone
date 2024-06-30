# Quick start
source /home/KNR/KNR-droniada/controller/myenv/bin/activate
# pobranie libek
# pip install -r REQUIREMENTS.txt
# aktywacja protokołów
# sudo raspi-config --> Interface Options --> I2C,SPI,Serial Port, Remote GPIO na YES
# aktywacja deamona od kamery
sudo pigpiod

/snap/bin/mjpg-streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so -w /usr/share/mjpg-streamer/www -p 8080" 2>/dev/null&

cd KNR-droniada/controller/

echo ""
