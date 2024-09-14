import cv2
import numpy as np
import requests
import io
import os
import json
import sysv_ipc
import time

# Static threshold for size (area) of white squares
SIZE_THRESHOLD = 500  # Adjust this value as needed

class Ball:
    def __init__(self, color, lower_bound, upper_bound):
        self.color = color
        self.lower_bound = np.array(lower_bound)
        self.upper_bound = np.array(upper_bound)
        self.center = None
        self.size = 0

    def update(self, roi, x_offset, y_offset):
        # Create a mask for the specific ball color
        mask = cv2.inRange(roi, self.lower_bound, self.upper_bound)
        color_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if color_contours:
            # Find the largest contour for this color
            largest_contour = max(color_contours, key=cv2.contourArea)

            # Check if the contour is large enough to be considered a ball
            if cv2.contourArea(largest_contour) > 30:
                # Update center and size based on the largest contour
                (cx, cy), radius = cv2.minEnclosingCircle(largest_contour)
                self.center = (int(cx) + x_offset, int(cy) + y_offset)
                self.size = int(radius)

    def draw(self, frame, screen_center):
        if self.center:
            # Draw a circle around the detected ball
            color_dict = {
                'red': (0, 0, 255),  # Red in BGR
                'blue': (255, 0, 0),  # Blue in BGR
                'purple': (255, 0, 255)  # Purple in BGR
            }
            color = color_dict[self.color]

            # Draw the circle with the color
            cv2.circle(frame, self.center, self.size, color, 2)

            # Calculate the vector from the ball center to the screen center
            vector_to_center = (screen_center[0] - self.center[0], screen_center[1] - self.center[1])

            # Calculate the arrow length proportional to the distance to the center
            arrow_length = int(np.linalg.norm(vector_to_center) / 5)  # Adjust the divisor for scaling

            # Calculate the endpoint of the arrow
            end_point = (
                self.center[0] + int(vector_to_center[0] * arrow_length / np.linalg.norm(vector_to_center)),
                self.center[1] + int(vector_to_center[1] * arrow_length / np.linalg.norm(vector_to_center))
            )

            # Draw an arrow pointing to the center of the screen
            cv2.arrowedLine(frame, self.center, end_point, color, 2)

            # Display the size inside the circle
            text = f"Size: {self.size}"
            cv2.putText(frame, text, (self.center[0] - self.size, self.center[1] - self.size - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class BallDetector:
    def __init__(self):
        # Define color ranges for red, blue, and purple balls
        self.balls = {
            'red': Ball('red', (35, 40, 120), (90, 80, 180)),
            'blue': Ball('blue', (105, 65, 15), (230, 115, 45)),
            'purple': Ball('purple', (105, 50, 60), (170, 85, 90))
        }
        # self.mq = sysv_ipc.MessageQueue(128, sysv_ipc.IPC_CREAT)
        self.target_vector = None
        # self.frame_dims = None

    def fetch_frame(self, mjpeg_url):
        # Fetch a single JPEG frame from the MJPEG stream
        response = requests.get(mjpeg_url, stream=True)
        if response.status_code == 200:
            # Read the image from the response content
            image_bytes = io.BytesIO(response.content)
            image = np.asarray(bytearray(image_bytes.read()), dtype=np.uint8)
            frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
            # self.frame_dims=(frame.shape[1], frame.shape[0])
            return frame
        else:
            print("Failed to fetch frame")
            return None

    def process_frame(self, frame):
        if frame is None:
            print("No frame to process")
            return

        # Get the center of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to get a binary image
        _, binary_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Define lower and upper range for white color in BGR format
        lower_white = np.array([200, 200, 200])
        upper_white = np.array([255, 255, 255])

        # Create a mask for white regions using the BGR frame
        white_mask = cv2.inRange(frame, lower_white, upper_white)

        # Combine the binary mask with the white mask
        combined_mask = cv2.bitwise_and(binary_mask, white_mask)

        # Find contours of the detected white regions
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area (size) and keep the largest ones
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > SIZE_THRESHOLD]
        large_contours = sorted(large_contours, key=cv2.contourArea, reverse=True)[:9]  # Top 9 largest contours

        # Draw bounding boxes around the top 9 largest contours
        for contour in large_contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Draw bounding box on the original frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box

            # Region of interest within the bounding box
            roi = frame[y:y + h, x:x + w]

            # Update each ball's position and size
            for ball in self.balls.values():
                ball.update(roi, x, y)

        # Draw each ball with its updated properties
        for ball in self.balls.values():
            ball.draw(frame, screen_center)

        # Display the resulting frame with bounding boxes and circles
        cv2.imshow('Bounding Boxes and Ball Detection', frame)
        cv2.waitKey(0)  # Wait for a key press to close the window

        # PODŁĄCZENIE Z MAIN COTROLEREM
        print("red center: ", self.balls['red'].center)  # red center:  (99, 306)
        print("blue center: ", self.balls['blue'].center)  # red center:  (99, 306)
        print("purple center: ", self.balls['purple'].center)  # red center:  (99, 306)
        print("blue size: ", self.balls['blue'].size)  # blue size:  20

        cv2.destroyAllWindows()

        return self.balls

    def process_frame_debug(self, frame):
        '''
        pokazuje przetworzoną klatkę w oknie
        '''
        if frame is None:
            print("No frame to process")
            return

        # Get the center of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to get a binary image
        _, binary_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Define lower and upper range for white color in BGR format
        lower_white = np.array([200, 200, 200])
        upper_white = np.array([255, 255, 255])

        # Create a mask for white regions using the BGR frame
        white_mask = cv2.inRange(frame, lower_white, upper_white)

        # Combine the binary mask with the white mask
        combined_mask = cv2.bitwise_and(binary_mask, white_mask)

        # Find contours of the detected white regions
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area (size) and keep the largest ones
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > SIZE_THRESHOLD]
        large_contours = sorted(large_contours, key=cv2.contourArea, reverse=True)[:9]  # Top 9 largest contours

        # Draw bounding boxes around the top 9 largest contours
        for contour in large_contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Draw bounding box on the original frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box

            # Region of interest within the bounding box
            roi = frame[y:y + h, x:x + w]

            # Update each ball's position and size
            for ball in self.balls.values():
                ball.update(roi, x, y)

        # Draw each ball with its updated properties
        for ball in self.balls.values():
            ball.draw(frame, screen_center)

        # Calculate vector_to_center for the largest ball
        largest_ball = max(self.balls.values(), key=lambda b: b.size if b.size > 0 else 0)
        self.target_vector = (screen_center[0] - largest_ball.center[0], screen_center[1] - largest_ball.center[1])


        # # Dane do wysłania
        # data = {
        #     'vector_to_center': vector_to_center
        # }

        # # Konwertujemy obiekt na string w formacie JSON
        # json_data = json.dumps(data)

        # # Wysyłamy dane jako bajty
        # self.mq.send(json_data.encode('utf-8'))
        # print(f"Sent vector: {json_data}")

        # Display the resulting frame with bounding boxes and circles
        # cv2.imshow('Bounding Boxes and Ball Detection', frame)
        # cv2.waitKey(0)  # Wait for a key press to close the window

        # PODŁĄCZENIE Z MAIN COTROLEREM
        print("red center: ", self.balls['red'].center)  # red center:  (99, 306)
        print("red size: ", self.balls['red'].size)  # blue size:  20

        # print("blue size: ", self.balls['blue'].size)  # blue size:  20

        # cv2.destroyAllWindows()

        return self.balls

if __name__ == "__main__":
    detector = BallDetector()
    # stream URL
    # mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'

    # while True:
    #     # Fetch and process a single JPEG frame
    #     print("fetching")
    #     frame = detector.fetch_frame(mjpeg_url)
    #     if frame is not None:
    #         # Print resolution of the frame -> 1920x1080
    #         # print("Frame resolution: ", frame.shape[1], "x", frame.shape[0]) 
    #         print("processing")
    #         detector.process_frame_debug(frame)
    #         # print(detector.process_frame_debug(frame))
    #         print(detector.target_vector)
    #     else:
    #         print("Failed to fetch frame")
    #     time.sleep(1)
