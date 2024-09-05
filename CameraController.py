from gpiozero import AngularServo
from time import sleep
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
from PIL import Image
import io
import os
from datetime import datetime
import ffmpeg

Device.pin_factory = PiGPIOFactory()

class CameraController:
    def __init__(self):
        self.camera_controller_obj = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.min_angle = -25
        self.max_angle = 90
        self.current_angle = 0  # Assume starting at 0 degrees
        self.image_folder = 'images'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

    def set_angle(self, x):
        if x < self.min_angle:
            self.current_angle = self.min_angle
        elif x > self.max_angle:
            self.current_angle = self.max_angle
        else:
            self.current_angle = x
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        return self.current_angle
    
    def max_positions_test(self):
        #while True:
        self.set_angle(33)
        # sleep(1)
        self.set_angle(-25)
        # sleep(2)

    def take_picture(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        img_path = f'pic_at_angle{self.current_angle}_{timestamp}.jpg'
        print(f"Taking picture at angle: {self.current_angle}")
        try:
            stream_url = 'http://localhost:8080/?action=stream'
            out, _ = (
                ffmpeg
                .input(stream_url)
                .output('pipe:', vframes=1, format='image2', vcodec='mjpeg')
                .run(capture_stdout=True, capture_stderr=True)
            )
            image_np = np.frombuffer(out, np.uint8)
            image = Image.open(io.BytesIO(image_np))
            image.save(os.path.join(self.image_folder, img_path))
            return img_path
        except ffmpeg.Error as e:
            print("ffmpeg error:", e.stderr.decode('utf8'))

    def take_pictures_continously(self):
        img_array = []
        
        try:
            stream_url = 'http://localhost:8080/?action=stream'
            i = 0
            while True:
                out, _ = (
                    ffmpeg
                    .input(stream_url)
                    .output('pipe:', vframes=1, format='image2', vcodec='mjpeg')
                    .run(capture_stdout=True, capture_stderr=True)
                )
                i += 1
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                img_path = f'pic_at_angle{self.current_angle}_{timestamp}.jpg'
                print(f"Taking picture at angle: {self.current_angle}")

                image_np = np.frombuffer(out, np.uint8)
                image = Image.open(io.BytesIO(image_np))
                image.save(os.path.join(self.image_folder, img_path))
                img_array.append(img_path)
                
                sleep(0.1)

            return img_array
        except ffmpeg.Error as e:
            print("ffmpeg error:", e.stderr.decode('utf8'))

        def take_pictures_continously():


        camera_controller_obj = CameraController()
        camera_controller_obj.take_picture()

        # array_pic = camera_controller_obj.take_pictures_continously()
            
            # #działa# patrzy w dół
            # print("kamera patrzy w dół")
            # # camera_controller_obj.set_angle(33)
            # # sleep żeby miało czas się ruszyć
            # sleep(0.2)
            # # strzel fote
            # print("zrobienie zdjęcia")
            # camera_controller_obj.take_picture()

            # # patrzy do przodu pod kątem lekko w dół
            # print("kamera patrzy na skos")
            # # camera_controller_obj.set_angle(-25)
            # sleep(0.2)
            # print("zrobienie zdjęcia")
            # camera_controller_obj.take_picture()

        #test ruchu kamery i zrobienia zdjęcia
        # test_gripper_controller()


if __name__ == "__main__":
    camera_controller_obj = CameraController()
    camera_controller_obj.max_positions_test()
    camera_controller_obj.take_pictures_continously()
