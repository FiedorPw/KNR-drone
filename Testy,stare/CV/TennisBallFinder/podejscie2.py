import cv2
import threading
from collections import Counter
import numpy as np


def get_platforms(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_threshold = 30
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= area_threshold]
    bounding_boxes = [cv2.boundingRect(contour) for contour in filtered_contours]
    return bounding_boxes, len(bounding_boxes)


def classify_color(color):
    max_comp = color.index(max(color))
    diff = max(color) - min(color)
    if color[max_comp] < 60:
        return 'black'
    if min(color) > 200:
        return 'empty'
    if diff < 10:
        return 'unknown'
    if max_comp == 0:
        return 'red'
    if max_comp == 1:
        return 'green'
    if max_comp == 2:
        return 'blue'
    return 'unknown'


def get_color_histogram(image, exclude_white=True):
    pixels = image.reshape(-1, 3)
    pixels = [tuple(pixel) for pixel in pixels]
    if exclude_white:
        pixels = [pixel for pixel in pixels if not (pixel[0] > 200 and pixel[1] > 200 and pixel[2] > 200)]
    color_counts = Counter(pixels)
    return color_counts


def classify_colors(image, bounding_boxes, center_crop_factor=0.5):
    color_classes = []
    confidences = []
    for x, y, w, h in bounding_boxes:
        center_w, center_h = int(w * center_crop_factor), int(h * center_crop_factor)
        center_x, center_y = x + w // 2 - center_w // 2, y + h // 2 - center_h // 2
        center_x = max(0, center_x)
        center_y = max(0, center_y)
        center_w = min(center_w, image.shape[1] - center_x)
        center_h = min(center_h, image.shape[0] - center_y)
        center_roi = image[center_y:center_y + center_h, center_x:center_x + center_w]
        color_histogram = get_color_histogram(center_roi)
        if not color_histogram:
            color_classes.append('Empty')
            confidences.append(0.0)
            continue
        colors, counts = zip(*color_histogram.items())
        max_count = max(counts)
        total_pixels = sum(counts)
        confidence = max_count / total_pixels
        color = colors[np.argmax(counts)]
        color_class = classify_color(color)
        color_classes.append(color_class)
        confidences.append(confidence)
    return color_classes, confidences


def process_video_from_camera():
    # Open the video stream from /dev/video0
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream from ")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        bounding_boxes, num = get_platforms(frame)
        colors, confidences = classify_colors(frame, bounding_boxes)

        for (x, y, w, h), color, confidence in zip(bounding_boxes, colors, confidences):
            label = f'{color} ({confidence:.2f})'
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Real-Time Video Stream', frame)

        # Press 'q' to exit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Start video processing from /dev/video0
    process_video_from_camera()
