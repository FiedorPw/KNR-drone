import cv2
import numpy as np
import requests
import io
import os
import json
import sysv_ipc
import time

# Static threshold for size (area) of white squares
SIZE_THRESHOLD = 6000  # NIE ZMIENIAĆ, NARAZIE DOBRZE DOSTROJONE

#TODO:
# 1. [DONE] add method to check if all platforms (9) are visible by camera and return position of left-down platform
# 2. Add method to check if any ball is visible, if true - return its color (Ball)
# 3. Add method to detect barrel
# 4. [DONE] Add method to return if any platform is detected
# 5. Right now get_position_one_platform cant detect 3 rows - nedd to change rowThreshold parameter to fit our purpose
# 6. [DONE] Add method to make platform_vector from frame center to platform center
# 7. In process_frame_debug platform_vector is made from largest platform - should it be changed to closest to frame center?
# 8. From some height platform will be not visible as square -> algorithm then needs to be switched to ball detection
# 9. Add class atribute isClose which tells if target/platform is near center

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
        self.aspect_ratio_tolerance = 0.6 # Parameter to detect more square-like or rectangular objects
        self.close_radius = 100
        self.row_threshold = 200 # Adjust based on expected platform size and spacing
        self.is_platform_close = False
        self.is_target_close = False
        self.balls = {
            'red': Ball('red', (35, 40, 120), (90, 80, 180)),
            'blue': Ball('blue', (105, 65, 15), (230, 115, 45)),
            'purple': Ball('purple', (105, 50, 60), (170, 85, 90))
        }
        self.target_vector = None
        self.platform_vector = None
        self.large_contours = [] # Number of detected platforms
        self.platform_rects = [] # Coordinates of detected platforms
        self.platforms_by_number = {}

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
    

    def detect_white_rectangles(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to get a binary image
        _, binary_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Define lower and upper range for white color in BGR format
        lower_white = np.array([230, 230, 230])
        upper_white = np.array([255, 255, 255])

        # Create a mask for white regions using the BGR frame
        white_mask = cv2.inRange(frame, lower_white, upper_white)

        # Combine the binary mask with the white mask
        combined_mask = cv2.bitwise_and(binary_mask, white_mask)

        # Find contours of the detected white regions
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area (size) and keep the largest ones
        self.large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > SIZE_THRESHOLD]
        self.large_contours = sorted(self.large_contours, key=cv2.contourArea, reverse=True)[:15]  # Top 9 largest contours    

        self.platform_rects = []
        for contour in self.large_contours:
            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = float(w)/h # when approaching 1 - its more square-like
            if (1-self.aspect_ratio_tolerance) <= aspect_ratio <= (1+self.aspect_ratio_tolerance):
                self.platform_rects.append({'x': x, 'y': y, 'w': w, 'h': h})
        # Update self.large_contours with the filtered contours
        self.large_contours = [cnt for cnt in self.large_contours if cv2.boundingRect(cnt)[2:4] == (w, h)]
    
    def get_all_platform_positions(self):
        if len(self.platform_rects) != 9:
            print("Not all 9 platforms are detected")
            return None
        
        #Sort platforms into a grid
        platforms = self.platform_rects.copy()

        # Calculate platform centers
        for platform in platforms:
            platform['center'] = (platform['x'] + platform['w']/2, platform['y'] + platform['h'] / 2)

        # Sort by y-coordinate (top to bottom)
        platforms.sort(key=lambda p: p['center'][1])

        rows=[]
        current_row=[]
        previous_y=None
        for platform in platforms:
            y=platform['center'][1]
            if previous_y is None or abs(y-previous_y) < self.row_threshold:
                current_row.append(platform)
            else:
                rows.append(current_row)
                current_row = [platform]
            previous_y = y
        if current_row:
            rows.append(current_row)

        # Check if we have exactly 3 rows
        if len(rows) != 3:
            print("Could not group platforms into 3 rows.")
            return None

        # Sort each row by x-coordinate (left to right)
        for row in rows:
            row.sort(key=lambda p: p['center'][0])

        # Assign platform numbers from 1 to 9 and collect their positions
        platform_positions = {}
        platform_number = 1
        self.platforms_by_number = {}  
        for row in rows:
            for platform in row:
                platform_positions[platform_number] = (platform['x'], platform['y'])
                platform['number']=platform_number
                self.platforms_by_number[platform_number] = platform
                platform_number += 1
        print("Platform positions:", platform_positions)
        return platform_positions

    def get_platform_vector(self, platform_number=None):
        num_platforms = len(self.platform_rects)
        if num_platforms == 0:
            print("No platforms detected.")
            return None

        frame = self.current_frame
        if frame is None:
            print("No frame available.")
            return None

        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        if num_platforms == 1:
            # If 1 platform is visible ignore platform number
            platform = self.platform_rects[0]
            platform_center = (platform['x'] + platform['w']/2, platform['y'] + platform['h']/2)
        elif num_platforms == 9:
            # If all platforms are visible
            if not hasattr(self, 'platforms_by_number') or not self.platforms_by_number:
                # Execute get_all_platform_positions() to get platforms_by_number
                self.get_all_platform_positions()
                if not hasattr(self, 'platforms_by_number') or not self.platforms_by_number:
                    print("Platforms not detected or numbered.")
                    return None

            if platform_number is None or platform_number not in self.platforms_by_number:
                print(f"Platform {platform_number} not detected.")
                return None

            platform = self.platforms_by_number[platform_number]
            platform_center = platform['center']
        else:
            # If there are visible more than 1 and less than 9 platforms, choose closest one
            platform = min(self.platform_rects, key=lambda p: (p['x'] + p['w']/2 - screen_center[0])**2 + (p['y'] + p['h']/2 - screen_center[1])**2)
            platform_center = (platform['x'] + platform['w']/2, platform['y'] + platform['h']/2)

        platform_vector = (screen_center[0]-platform_center[0], screen_center[1] - platform_center[1])

        return platform_vector


    def process_frame(self, frame):
        if frame is None:
            print("No frame to process")
            return

        # Get the center of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        self.detect_white_rectangles(frame)

        # Draw bounding boxes around the top 9 largest contours
        for contour in self.large_contours:
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

        # # PODŁĄCZENIE Z MAIN COTROLEREM
        # print("red center: ", self.balls['red'].center)  # red center:  (99, 306)
        # print("blue center: ", self.balls['blue'].center)  
        # print("purple center: ", self.balls['purple'].center)  
        # print("blue size: ", self.balls['blue'].size)  # blue size:  20

        cv2.destroyAllWindows()

        return self.balls

    def process_frame_debug(self, frame):
        '''
        pokazuje przetworzoną klatkę w oknie
        '''
        if frame is None:
            print("No frame to process")
            return

        self.current_frame = frame
        # Get the center of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        self.detect_white_rectangles(frame)

        self.get_all_platform_positions()

        # Draw bounding boxes around the top 9 largest contours
        for contour in self.large_contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Draw bounding box on the original frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box

            # Region of interest within the bounding box
            roi = frame[y:y + h, x:x + w]

            # Update each ball's position and size
            for ball in self.balls.values():
                ball.update(roi, x, y)


        # Calculate platform_vector
        if self.platform_rects:
            # Wybierz platformę najbliższą centrum obrazu
            platform = min(self.platform_rects, key=lambda p: (p['x'] + p['w']/2 - screen_center[0])**2 + (p['y'] + p['h']/2 - screen_center[1])**2)
            platform_center = (platform['x'] + platform['w'] / 2, platform['y'] + platform['h'] / 2)
            self.platform_vector = (platform_center[0] - screen_center[0], screen_center[1] - platform_center[1])

            platform_distance = np.linalg.norm(self.platform_vector)
            self.is_platform_close = platform_distance <= self.close_radius
            print("Platform vector:", self.platform_vector)
            print("Is platform close:", self.is_platform_close)
        else:
            self.platform_vector = None
            self.is_platform_close = False

        # # Rysowanie prostokątów i numerów platform
        # for platform_number, platform in self.platforms_by_number.items():
        #     x, y, w, h = platform['x'], platform['y'], platform['w'], platform['h']
        #     # Rysowanie prostokąta na oryginalnym obrazie
        #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Zielony prostokąt
        #     # Wyświetlanie numeru platformy
        #     cv2.putText(frame, str(platform_number), (int(x + w/2), int(y + h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        #     # Draw each ball with its updated properties
        #     for ball in self.balls.values():
        #         ball.draw(frame, screen_center)

        # Calculate vector_to_center for the largest ball
        largest_ball = max(self.balls.values(), key=lambda b: b.size if b.size > 0 else 0)
        if largest_ball.size > 0:
            self.target_vector = (largest_ball.center[0]-screen_center[0], screen_center[1] - largest_ball.center[1])
            # Calculate distance from center to ball center
            target_distance = np.linalg.norm(self.target_vector)
            self.is_target_close = target_distance <= self.close_radius

            print("Target vector:", self.target_vector)
            print("Target distance:", target_distance)
            print("Is target close:", self.is_target_close)
        else:
            self.target_vector = None
            self.is_target_close = False


        # Print information about balls
        print("Detected balls:")
        for color, ball in self.balls.items():
            print(f"{color.capitalize()} center: {ball.center}, size: {ball.size}")

        # Optionally, you can return the balls data

if __name__ == "__main__":
    detector = BallDetector()
    # stream URL
    mjpeg_url = 'http://127.0.0.1:8080/?action=snapshot'

    while True:
        # Fetch and process a single JPEG frame
        # print("Fetching frame...")
        frame = detector.fetch_frame(mjpeg_url)
        if frame is not None:
            # Process the frame and detect balls and white rectangles
            # print("Processing frame...")
            detector.process_frame_debug(frame)

            print(f"Detected platforms: {len(detector.large_contours)}")
            # Check if all platforms are detected
            platform_positions = detector.get_all_platform_positions()
            if platform_positions:
                print("All platforms detected. Positions:")
                for number, position in platform_positions.items():
                    print(f"Platform {number}: Position {position}")
            else:
                print("Not all platforms detected or could not determine platform positions.")
        else:
            print("Failed to fetch frame")

        time.sleep(2)

