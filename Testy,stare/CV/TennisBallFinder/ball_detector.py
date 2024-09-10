import cv2
import numpy as np
import json
import os

SIZE_THRESHOLD = 500  # Adjust this value as needed

# Ensure the named pipe exists
fifo_path = '/tmp/ball_detection_fifo'
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

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
        color_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours in the mask for the specific ball color)

        if color_contours:
            # Find the largest contour for this color
            largest_contour = max(color_contours, key=cv2.contourArea)

            # Check if the contour is large enough to be considered a ball
            if cv2.contourArea(largest_contour) > 30:  # Threshold to ignore small noise
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
    def __init__(self, video_source=0):
        self.video_source = video_source
        self.cap = cv2.VideoCapture(video_source)

        if not self.cap.isOpened():
            raise ValueError("Error: Could not open video source.")

        self.balls = {
            'red': Ball('red', (35, 40, 120), (90, 80, 180)),
            'blue': Ball('blue', (105, 65, 15), (230, 115, 45)),
            'purple': Ball('purple', (105, 50, 60), (170, 85, 90))
        }
        self.fifo_path = fifo_path

    def write_detection_data(self):
        detection_data = {
            ball_color: {
                'center': ball.center,
                'size': ball.size
            } for ball_color, ball in self.balls.items()
        }
        with open(self.fifo_path, 'w') as fifo:
            json.dump(detection_data, fifo)

    def show_bounding_boxes(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
            lower_white = np.array([200, 200, 200])
            upper_white = np.array([255, 255, 255])
            white_mask = cv2.inRange(frame, lower_white, upper_white)
            combined_mask = cv2.bitwise_and(binary_mask, white_mask)
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > SIZE_THRESHOLD]
            large_contours = sorted(large_contours, key=cv2.contourArea, reverse=True)[:9]

            for contour in large_contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                roi = frame[y:y + h, x:x + w]
                for ball in self.balls.values():
                    ball.update(roi, x, y)

            for ball in self.balls.values():
                ball.draw(frame, screen_center)

            cv2.imshow('Bounding Boxes and Ball Detection', frame)
            self.write_detection_data()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# Instantiate the BallDetector class and call the function
detector = BallDetector()
detector.show_bounding_boxes()

