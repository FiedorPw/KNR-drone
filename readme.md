# Konfiguracja i przygotowanie do działania

## Quick start
odpala python enviroment:
source /setup.sh &

## serwisy

### 4G LTE

mmcli -L
mmcli -m 0

sudo mmcli -m 0 --enable
successfully enabled the modem

## porty

Wykorzystujemy tunelowanie za pomocą cloudflare argotunnel obsługiwane przez cloudflared.

80:/
443:/

5000:/api
https://dron.knr.edu.pl/api

8080:/camera
https://dron-kamera.knr.edu.pl/?action=stream

9090:/cockpit
https://dron-cockpit.knr.edu.pl/
nie działa tylko z firefoxa :(

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
## obsługa błedow
sudo systemctl restart mjpg-streamer.service    #restart kamery
# Dokumentacja
## Architektura Hardware'u
1. Elektronikę w postaci mikro-komputera RaspberyPI 4B - elemęt wykonawczy dla algorytmów lotu, CV i sterowania peryferiami(schemat kodu poniżej).
2. Kontroler lotu MATEK H743-SLIM V3 - odpowiada za kontrolę silnikóœ, plan misji drona. Stabilizacja w locie na podstawie czujników takich jak magnetrony, GPS, akceleratory, barometry i inne cuda wianki.
3. Rama - Holy bro kit X500 V2
4. Aktuatory
  - Dropper - do zrzucania dartów(konkurencja sztafeta
  - Gripper - do łapania kolorowych piłek(konkurencja kopalnie marsjańskie)
  - Serwo do ruszania kamerą
5. Sensory
  - Kamera szerokokątna
  - Czujnik odbiciowy


## Architektura Software'u
![Architecture](https://github.com/FiedorPw/KNR-drone/blob/main/schemat%20architektury%20drona.png)
