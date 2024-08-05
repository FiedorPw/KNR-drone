Komenda do odpalenia strima:
/snap/bin/mjpg-streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so -w /usr/share/mjpg-streamer/www -p 8080"
Może również działać taka komenda:
ffmpeg -i http://192.168.77.183:8080/?action=stream -c:v copy -f segment -segment_time 3600 -reset_timestamps 1 output%d.mkv

Adres strima:
http://192.168.77.183:8080/?action=stream

Mikołaj kurwa debilu to nie tak sie robi