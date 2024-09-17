# Konfiguracja i przygotowanie do działania

## Quick start
odpala python enviroment:
source /setup.sh &

## serwisy



### Konfiguracja modemu 4G LTE Huawei

mmcli -L
    /org/freedesktop/ModemManager1/Modem/2 [huawei] E3276

   # stąd bierzemy id do następnej komendy, w tym przypadku 2

mmcli -m 2
sudo mmcli -m 2 --simple-connect="apn=internet,ip-type=ipv4"

mmcli -m 2
  --------------------------------
  General  |                 path: /org/freedesktop/ModemManager1/Modem/2
           |            device id: 31386a378d888b459070e8a2b8767c2d5469c360
  --------------------------------
  Hardware |         manufacturer: huawei
           |                model: E3276
           |    firmware revision: 21.260.05.00.618
           |            supported: gsm-umts
           |              current: gsm-umts
           |         equipment id: 863781013763835
  --------------------------------
  System   |               device: /sys/devices/pci0000:00/0000:00:14.0/usb1/1-1
           |              physdev: /sys/devices/pci0000:00/0000:00:14.0/usb1/1-1
           |              drivers: huawei_cdc_ncm, option
           |               plugin: huawei
           |         primary port: cdc-wdm0
           |                ports: cdc-wdm0 (at), ttyUSB0 (at), wwx0c5b8f279a64 (net)
  --------------------------------
  Status   |       unlock retries: sim-pin (3), sim-puk (10), sim-pin2 (3), sim-puk2 (10)
           |                state: connected
           |          power state: on
           |          access tech: lte
           |       signal quality: 93% (recent)
  --------------------------------
  Modes    |            supported: allowed: 2g; preferred: none
           |                       allowed: 3g; preferred: none
           |                       allowed: 4g; preferred: none
           |                       allowed: 2g, 3g, 4g; preferred: none
           |              current: allowed: 4g; preferred: none
  --------------------------------
  IP       |            supported: ipv4, ipv6, ipv4v6
  --------------------------------
  3GPP     |                 imei: 863781013763835
           |          operator id: 26006
           |        operator name: POL
           |         registration: home
           | packet service state: attached
  --------------------------------
  3GPP EPS | ue mode of operation: csps-2
  --------------------------------
  SIM      |     primary sim path: /org/freedesktop/ModemManager1/SIM/2
  --------------------------------
  Bearer   |                paths: /org/freedesktop/ModemManager1/Bearer/0

  z ostatniej lini bierzemy id bearera w tym przypadku 0, do następnego polecenia
# jesli nie ma
# dodanie bearera: 
sudo mmcli -m 5 --create-bearer="apn=internet"
# dezaktywiowanie barera:
sudo mmcli -b 0 --disconnect
sudo mmcli -b 0 --delete
# Jesli cos nie ziala mozna zresetowac:
sudo systemctl restart ModemManager

sudo mmcli -b 0

ip a
  stąd bierzemy nazwę interface'u do następnej komendy, w tmy przypadku wwx0c5b8f279a64

sudo ip link set wwx0c5b8f279a64 up

sudo dhclient wwx0c5b8f279a64

ping -I wwan0 8.8.8.8
# poprawny output
64 bytes from 8.8.8.8: icmp_seq=59 ttl=58 time=15.8 ms
powinno działać

można jeszcze przełączyć na defaultowy interface wwx0c5b8f279a64 

# Konfiguracja 4G

## porty

Wykorzystujemy tunelowanie za pomocą cloudflare argotunnel obsługiwane przez cloudflared.

80:/
443:/

5000:/api
https://dron.knr.edu.pl/api

8080:/camera
https://dron-kamera.knr.edu.pl/?action=stream

komenda do restartu serwisu kamery:
sudo systemctl restart mjpg-streamer.service 

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
