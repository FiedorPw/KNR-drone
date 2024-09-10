import cv2
import numpy as np

# Static threshold for size (area) of white squares
SIZE_THRESHOLD = 500  # Adjust this value as needed

def is_ball(contour):
    # Dummy certainty value based on the contour area (this can be refined)
    # For demonstration, let's assume certainty is 80%
    return 80

def nothing(x):
    pass

def create_hsv_trackbars(window_name):
    # Create trackbars for adjusting HSV ranges
    cv2.createTrackbar('Lower H', window_name, 0, 179, nothing)
    cv2.createTrackbar('Lower S', window_name, 100, 255, nothing)
    cv2.createTrackbar('Lower V', window_name, 100, 255, nothing)
    cv2.createTrackbar('Upper H', window_name, 10, 179, nothing)
    cv2.createTrackbar('Upper S', window_name, 255, 255, nothing)
    cv2.createTrackbar('Upper V', window_name, 255, 255, nothing)

def get_hsv_range(window_name):
    # Get current positions of trackbars for lower and upper HSV values
    lower_h = cv2.getTrackbarPos('Lower H', window_name)
    lower_s = cv2.getTrackbarPos('Lower S', window_name)
    lower_v = cv2.getTrackbarPos('Lower V', window_name)
    upper_h = cv2.getTrackbarPos('Upper H', window_name)
    upper_s = cv2.getTrackbarPos('Upper S', window_name)
    upper_v = cv2.getTrackbarPos('Upper V', window_name)

    lower_range = np.array([lower_h, lower_s, lower_v])
    upper_range = np.array([upper_h, upper_s, upper_v])

    return lower_range, upper_range

def show_bounding_boxes(video_source=0):
    # Open the video source (0 for webcam, or path to video file)
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    # Create windows for trackbars
    cv2.namedWindow('Red HSV Adjustments')
    cv2.namedWindow('Blue HSV Adjustments')
    cv2.namedWindow('Purple HSV Adjustments')

    # Create HSV trackbars for red, blue, and purple
    create_hsv_trackbars('Red HSV Adjustments')
    create_hsv_trackbars('Blue HSV Adjustments')
    create_hsv_trackbars('Purple HSV Adjustments')

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

            # Convert the ROI to HSV color space
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Get HSV ranges from trackbars for each color
            red_lower, red_upper = get_hsv_range('Red HSV Adjustments')
            blue_lower, blue_upper = get_hsv_range('Blue HSV Adjustments')
            purple_lower, purple_upper = get_hsv_range('Purple HSV Adjustments')

            # Define color ranges in HSV using trackbar values
            color_ranges = {
                'red': (red_lower, red_upper),  # Red range in HSV
                'blue': (blue_lower, blue_upper),  # Blue range in HSV
                'purple': (purple_lower, purple_upper)  # Purple range in HSV
            }

            largest_balls = {}  # Dictionary to store the largest contour for each color

            for color_name, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv_roi, lower, upper)
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
