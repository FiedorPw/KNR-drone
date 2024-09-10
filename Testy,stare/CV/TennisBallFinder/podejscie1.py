import cv2
import numpy as np

# Define initial color ranges for red, blue, and purple in HSV
color_ranges = {
    'red': ((0, 120, 70), (10, 255, 255)),
    'blue': ((100, 150, 0), (140, 255, 255)),
    'purple': ((130, 50, 50), (160, 255, 255))
}

# Function to calculate distance and direction from the center
def calculate_distance_and_direction(x, y, center_x, center_y):
    dx = center_x - x
    dy = center_y - y
    distance = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx)
    return distance, angle

# Function to create trackbars for HSV adjustment
def create_trackbars():
    def nothing(x):
        pass
    cv2.namedWindow('HSV Trackbars')
    for color in color_ranges:
        cv2.createTrackbar(f'{color}_H_low', 'HSV Trackbars', color_ranges[color][0][0], 179, nothing)
        cv2.createTrackbar(f'{color}_S_low', 'HSV Trackbars', color_ranges[color][0][1], 255, nothing)
        cv2.createTrackbar(f'{color}_V_low', 'HSV Trackbars', color_ranges[color][0][2], 255, nothing)
        cv2.createTrackbar(f'{color}_H_high', 'HSV Trackbars', color_ranges[color][1][0], 179, nothing)
        cv2.createTrackbar(f'{color}_S_high', 'HSV Trackbars', color_ranges[color][1][1], 255, nothing)
        cv2.createTrackbar(f'{color}_V_high', 'HSV Trackbars', color_ranges[color][1][2], 255, nothing)

# Initialize webcam
cap = cv2.VideoCapture(0)

# Create trackbars
create_trackbars()  # Added function call to create trackbars

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get the center of the frame
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Update color ranges from trackbars
    for color in color_ranges:
        h_low = cv2.getTrackbarPos(f'{color}_H_low', 'HSV Trackbars')
        s_low = cv2.getTrackbarPos(f'{color}_S_low', 'HSV Trackbars')
        v_low = cv2.getTrackbarPos(f'{color}_V_low', 'HSV Trackbars')
        h_high = cv2.getTrackbarPos(f'{color}_H_high', 'HSV Trackbars')
        s_high = cv2.getTrackbarPos(f'{color}_S_high', 'HSV Trackbars')
        v_high = cv2.getTrackbarPos(f'{color}_V_high', 'HSV Trackbars')
        color_ranges[color] = ((h_low, s_low, v_low), (h_high, s_high, v_high))

    for color_name, (lower, upper) in color_ranges.items():
        # Create mask for the color
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Filter out small contours
            if cv2.contourArea(contour) < 500:
                continue

            # Get the coordinates of the ball
            (x, y), radius = cv2.minEnclosingCircle(contour)
            x, y = int(x), int(y)

            # Draw the circle and the center point
            cv2.circle(frame, (x, y), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

            # Calculate distance and direction
            distance, angle = calculate_distance_and_direction(x, y, center_x, center_y)  # Changed to new function

            # Draw arrow pointing to center
            arrow_length = int(distance * 0.5)  # Adjust the factor to change arrow length
            end_x = int(x + arrow_length * np.cos(angle))
            end_y = int(y + arrow_length * np.sin(angle))
            cv2.arrowedLine(frame, (x, y), (end_x, end_y), (255, 0, 0), 2)  # Added arrow visualization

            # Display distance and direction
            cv2.putText(frame, f'{color_name} Dist: {int(distance)}', (x - 40, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Draw the center point of the frame
    cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)

    # Show the frame
    cv2.imshow('Ball Tracking', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
