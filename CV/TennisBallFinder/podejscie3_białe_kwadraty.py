import cv2
import numpy as np

# Static threshold for size (area) of white squares
SIZE_THRESHOLD = 500  # Adjust this value as needed

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

        # Display the resulting frame with bounding boxes
        cv2.imshow('Bounding Boxes on White Squares', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Call the function with default webcam
show_bounding_boxes()
