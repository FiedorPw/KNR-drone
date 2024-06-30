## Quick start
odpala python enviroment:
source /setup.sh &


## pobranie libek i stworzenie python virtual enviroment(venv)
python3 -m venv myenv
pip install -r REQUIREMENTS.txt
## aktywacja protokołów
sudo raspi-config --> Interface Options --> I2C,SPI,Serial Port, Remote GPIO na YES

## KAMERA
## aktywacja deamona od kamery
sudo pigpiod
## instalowanie serwisu od kamery
sudo apt install snapd
sudo systemctl enable --now snapd.socket
sudo ln -s /var/lib/snapd/snap /snap

/snap/bin/mjpg-streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so -w /usr/share/mjpg-streamer/www -p 8080"
