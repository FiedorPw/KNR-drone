import cv2
import threading
from collections import Counter
import numpy as np
import matplotlib.pyplot as plt


"""
Extracts and displays multiple regions of interest (ROIs) from the image based on the given list of bounding boxes.

Parameters:
- image: The input image.
- bounding_boxes: A list of tuples where each tuple is (x, y, w, h) representing a bounding box.
"""

def show_regions_of_interest(image, bounding_boxes, titles=None):


    num_boxes = len(bounding_boxes)
    cols = 3
    rows = (num_boxes + cols - 1) // cols  # Calculate the number of rows needed

    plt.figure(figsize=(15, 5 * rows))
    
    for i, (x, y, w, h) in enumerate(bounding_boxes):
        # Extract the ROI
        roi = image[y:y+h, x:x+w]
        
        # Display the ROI
        plt.subplot(rows, cols, i + 1)
        plt.imshow(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        if titles is not None and i < len(titles):
            plt.title(titles[i])
        else:
            plt.title(f'Region {i + 1}')
    
    plt.tight_layout()
    plt.show()
   
    
# def show_regions_and_histograms(image, bounding_boxes, center_crop_factor=0.5):
#     """
#     Extracts and displays multiple regions of interest (ROIs) from the image based on the given list of bounding boxes.
#     Also shows the color histogram next to each ROI focusing on the center region.
    
#     Parameters:
#     - image: The input image.
#     - bounding_boxes: A list of tuples where each tuple is (x, y, w, h) representing a bounding box.
#     - center_crop_factor: Factor to crop the center region for histogram analysis (default is 0.5).
#     """
#     num_boxes = len(bounding_boxes)
#     cols = 2  # Each ROI will be displayed next to its histogram
#     rows = num_boxes  # One row for each bounding box

#     plt.figure(figsize=(15, 5 * rows))
    
#     for i, (x, y, w, h) in enumerate(bounding_boxes):
#         # Calculate the center crop dimensions
#         center_w, center_h = int(w * center_crop_factor), int(h * center_crop_factor)
#         center_x, center_y = x + w // 2 - center_w // 2, y + h // 2 - center_h // 2
        
#         # Ensure the center region is within bounds
#         center_x = max(0, center_x)
#         center_y = max(0, center_y)
#         center_w = min(center_w, image.shape[1] - center_x)
#         center_h = min(center_h, image.shape[0] - center_y)
        
#         # Extract the center region
#         center_roi = image[center_y:center_y+center_h, center_x:center_x+center_w]
        
#         if center_roi.size > 0:
#             # Get the color histogram for the center ROI excluding white
#             color_histogram = get_color_histogram(center_roi)
            
#             if color_histogram:
#                 colors, counts = zip(*color_histogram.items())
                
#                 # Normalize colors to [0, 1] range for plotting
#                 normalized_colors = [tuple(np.array(color) / 255.0) for color in colors]
                
#                 # Display the histogram
#                 plt.subplot(rows, cols, 2 * i + 2)
#                 plt.bar(range(len(counts)), counts, color=normalized_colors)
#                 plt.xlabel('Color')
#                 plt.ylabel('Frequency')
#                 plt.title(f'Color Histogram {i + 1}')
#                 plt.xticks([])  # Hide x-axis labels for clarity
#             else:
#                 plt.subplot(rows, cols, 2 * i + 2)
#                 plt.text(0.5, 0.5, 'No colors detected\n(excluding white)', 
#                          horizontalalignment='center', verticalalignment='center', 
#                          transform=plt.gca().transAxes)
#                 plt.axis('off')
#                 plt.title(f'Color Histogram {i + 1}')
#         else:
#             plt.subplot(rows, cols, 2 * i + 2)
#             plt.text(0.5, 0.5, 'Empty center ROI', 
#                      horizontalalignment='center', verticalalignment='center', 
#                      transform=plt.gca().transAxes)
#             plt.axis('off')
#             plt.title(f'Color Histogram {i + 1}')

#         # Display the ROI
#         plt.subplot(rows, cols, 2 * i + 1)
#         plt.imshow(center_roi)
#         plt.axis('off')
#         plt.title(f'Region {i + 1}')

#     plt.tight_layout()
#     plt.show()
    

def get_platforms(image):
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Thresholding to get binary image and detect white areas (platforms)
    _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    area_threshold = 30  # Adjust this value based on your requirements

    # Filter contours by area
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= area_threshold]


    # Calculate the bounding rectangles, which gives the boundaries of the platforms
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
    # Reshape the image to be a list of pixels
    pixels = image.reshape(-1, 3)
    # Convert to a list of tuples
    pixels = [tuple(pixel) for pixel in pixels]
    
    if exclude_white:
        # Remove white pixels (thresholds can be adjusted)
        pixels = [pixel for pixel in pixels if not (pixel[0] > 200 and pixel[1] > 200 and pixel[2] > 200)]
    
    # Count the occurrences of each color
    color_counts = Counter(pixels)
    
    return color_counts

def classify_colors(image, bounding_boxes, center_crop_factor=0.5):
    color_classes = []
    confidences = []
    for x, y, w, h in bounding_boxes:
         # Calculate the center crop dimensions
        center_w, center_h = int(w * center_crop_factor), int(h * center_crop_factor)
        center_x, center_y = x + w // 2 - center_w // 2, y + h // 2 - center_h // 2
        
        # Ensure the center region is within bounds
        center_x = max(0, center_x)
        center_y = max(0, center_y)
        center_w = min(center_w, image.shape[1] - center_x)
        center_h = min(center_h, image.shape[0] - center_y)
        
        # Extract the center region
        center_roi = image[center_y:center_y+center_h, center_x:center_x+center_w]
        
        color_histogram = get_color_histogram(center_roi)
        
        if not color_histogram:
            color_classes.append('Empty')
            confidences.append(0.0)
            continue
        
        # Classify the color based on the histogram
        colors, counts = zip(*color_histogram.items())
        max_count = max(counts)
        total_pixels = sum(counts)
        confidence = max_count / total_pixels
        
        color = colors[np.argmax(counts)]
        color_class = classify_color(color)
        color_classes.append(color_class)
        confidences.append(confidence)
        
    return color_classes, confidences
       
       
def find_colors(image_path, visualize=True):
    image = cv2.imread(image_path)
    if visualize:
        image_copy = image.copy()
    bounding_boxes, num = get_platforms(image)
    print(num, bounding_boxes)
    colors, confidences = classify_colors(image, bounding_boxes)
    if visualize:
        titles = [f'{color} (Confidence: {conf:.2f})' for color, conf in zip(colors, confidences)]
        # show_regions_of_interest(image_copy, bounding_boxes, titles=titles)
    return colors, confidences

def preprocess_image(image):
        resized_image = cv2.resize(image, (1000, 1000))
        return resized_image


class BallFinder:
    _instance = None
    _lock = threading.Lock()
    grid_size = (3, 3)
    image_size = (1000, 1000)

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(BallFinder, cls).__new__(cls)
                cls._instance.init()
        return cls._instance

    def init(self):
        self.grid_dict = {i: None for i in range(9)}
        self.grid = [[{'color': None, 'confidence': 0.0} for _ in range(self.grid_size[1])] for _ in range(self.grid_size[0])]
        self.is_finished = False

    def process_image(self, image_path) -> dict:
        image = cv2.imread(image_path)
        bounding_boxes, num = get_platforms(image)
        if num !=9:
            return {'status': 'error', 'desc': 'Number of platforms is not 9', 'grid': self.grid}
        colors, confidences = classify_colors(image, bounding_boxes)
        counts = {'blue': 0, 'green': 0, 'red': 0, 'black': 0, 'empty': 0, 'unknown': 0}
        for color in colors:
            counts[color.lower()] += 1
        if counts['blue'] == 1 and counts['red'] == 1 and counts['black'] == 1:
            for i, color in enumerate(colors):
                self.grid_dict[i] = color
                self.grid[i // 3][i % 3] = {'color': color, 'confidence': confidences[i]} 
            self.is_finished = True
            return {'status': 'success', 'desc': 'Grid is filled', 'grid': self.grid}
        else:
            for i, color in enumerate(colors):
                self.grid_dict[i] = color
                self.grid[i // 3][i % 3] = {'color': color, 'confidence': confidences[i]}
            self.is_finished = False
            return {'status': 'error', 'desc': 'Incorrect colors', 'grid': self.grid}
        
if __name__ == '__main__':
    finder = BallFinder()
    image_path = 'images/zdjecie66.jpg'
    find_colors(image_path)
    finder.process_image(image_path)
    
    if finder.is_finished:
        print(finder.grid)
        print(finder.grid[2][2]['color'])
    
    # image = cv2.imread(image_path)    
    # cv2.imshow('image', image)
    # cv2.waitKey(0)
