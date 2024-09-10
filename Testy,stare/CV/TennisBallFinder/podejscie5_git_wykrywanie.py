import cv2
import numpy as np

# Static threshold for size (area) of white squares
SIZE_THRESHOLD = 500  # Adjust this value as needed

def is_ball(contour):
    # Dummy certainty value based on the contour area (this can be refined)
    # For demonstration, let's assume certainty is 80%
    return 80

def show_bounding_boxes(video_source=0):
    # Open the video source (0 for webcam, or path to video file)
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    while True:
        # Read a frame from the video source
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

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
            roi = frame[y:y+h, x:x+w]

            # Define color ranges for red, blue, and purple
            # Colour space: BGR
            # observed balls:
                # Ball - BLUE,GREEN,RED | pomiar 2
                # R - 47,52,164
                # B - 149, 83, 9 | 34,100,180
                # P - 138,47,61

            # R - 47,51,161-76,67,132
            # B - 179,103,30-120,82,30
            # P - 154,63,75-120,70,74

            # experymentalnie wyznaczone w 027(mogą być trochę ciemne)
            color_ranges = {
                'red': ((35, 40, 120), (90, 80, 180)),
                'blue': ((105, 65, 15), (230, 115, 45)),
                'purple': ((105, 50, 60), (170, 85, 90))
            }


            # color_ranges = {
            #     'red': ((0, 0, 100), (80, 80, 255)),
            #     'blue': ((100, 0, 0), (255, 80, 80)),
            #     'purple': ((100, 20, 40), (150, 60, 100))
            # }

            largest_balls = {}  # Dictionary to store the largest contour for each color

            for color_name, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(roi, np.array(lower), np.array(upper))
                color_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if color_contours:
                    # Find the largest contour for this color
                    largest_contour = max(color_contours, key=cv2.contourArea)

                    # Check if the contour is large enough to be considered a ball
                    if cv2.contourArea(largest_contour) > 30:  # Threshold to ignore small noise
                        largest_balls[color_name] = largest_contour

            # Draw circles for the largest ball of each color
            for color_name, contour in largest_balls.items():
                # Draw a circle around the detected ball
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                center = (int(cx) + x, int(cy) + y)
                radius = int(radius)

                # Color for the ball's circle
                color_dict = {
                    'red': (0, 0, 255),  # Red in BGR
                    'blue': (255, 0, 0),  # Blue in BGR
                    'purple': (255, 0, 255)  # Purple in BGR
                }
                color = color_dict[color_name]

                # Draw the circle with the color
                cv2.circle(frame, center, radius, color, 2)

                # Calculate certainty
                certainty = is_ball(contour)

                # Display the color name and certainty above the circle
                text = f"{color_name.capitalize()}: {certainty}%"
                cv2.putText(frame, text, (center[0] - radius, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Display the resulting frame with bounding boxes and circles
        cv2.imshow('Bounding Boxes and Ball Detection', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Call the function with default webcam
show_bounding_boxes()
