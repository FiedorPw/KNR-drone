import cv2
import numpy as np

# Define color ranges for red, blue, and purple in HSV
color_ranges = {
    'red': ((0, 120, 70), (10, 255, 255)),  # Lower and upper range for red
    'blue': ((100, 150, 0), (140, 255, 255)),  # Lower and upper range for blue
    'purple': ((130, 50, 50), (160, 255, 255))  # Lower and upper range for purple
}

# Function to calculate distance from the center
def calculate_distance(x, y, center_x, center_y):
    return np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get the center of the frame
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

            # Calculate and display the distance
            distance = calculate_distance(x, y, center_x, center_y)
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
