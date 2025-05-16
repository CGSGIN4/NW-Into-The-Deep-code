import cv2
import numpy as np

# Define color ranges in HSV
RED_RANGE = ((0, 88, 21), (13, 255, 255))
RED_RANGE1 = ((167, 88, 21), (180, 255, 255))
BLUE_RANGE = ((95, 40, 0), (139, 255, 255))
YELLOW_RANGE = ((15, 174, 69), (56, 255, 255))

CHECK_X, CHECK_Y = 315, 780

# Define the size of the square to check (odd number recommended)
SQUARE_SIZE = 31

# Define the minimum percentage for a color to be considered dominant
DOMINANCE_THRESHOLD = 0.6  # 60%

def color_in_range(color, range):
    return (color[0] >= range[0][0] and color[0] <= range[1][0] and
            color[1] >= range[0][1] and color[1] <= range[1][1] and
            color[2] >= range[0][2] and color[2] <= range[1][2])

def get_color_state(hsv_color):
    if color_in_range(hsv_color, RED_RANGE) or color_in_range(hsv_color, RED_RANGE1):
        return "RED"
    elif color_in_range(hsv_color, BLUE_RANGE):
        return "BLUE"
    elif color_in_range(hsv_color, YELLOW_RANGE):
        return "YELLOW"
    else:
        return "OTHER"

def get_dominant_color(hsv_image, center_x, center_y, size):
    # Calculate the bounds of the square
    half_size = size // 2
    x_start = max(0, center_x - half_size)
    x_end = min(hsv_image.shape[1], center_x + half_size + 1)
    y_start = max(0, center_y - half_size)
    y_end = min(hsv_image.shape[0], center_y + half_size + 1)

    # Extract the square region
    region = hsv_image[y_start:y_end, x_start:x_end]

    # Count occurrences of each color
    color_counts = {"RED": 0, "BLUE": 0, "YELLOW": 0, "OTHER": 0}
    total_pixels = region.shape[0] * region.shape[1]

    for row in region:
        for pixel in row:
            color_counts[get_color_state(pixel)] += 1

    # Find the most common color
    dominant_color = max(color_counts, key=color_counts.get)
    dominant_percentage = color_counts[dominant_color] / total_pixels

    if dominant_percentage >= DOMINANCE_THRESHOLD:
        return dominant_color, dominant_percentage
    else:
        return "MIXED", dominant_percentage

def runPipeline(image, llrobot):
    # Convert image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get the dimensions of the image
    height, width = image.shape[:2]

    # Get the center coordinates
    center_x, center_y = CHECK_X, CHECK_Y

    # Get the dominant color in the square region
    color_state, color_percentage = get_dominant_color(hsv_image, center_x, center_y, SQUARE_SIZE)

    # Visualize the result
    half_size = SQUARE_SIZE // 2
    cv2.rectangle(image,
                  (center_x - half_size, center_y - half_size),
                  (center_x + half_size, center_y + half_size),
                  (0, 255, 0), 2)
    cv2.putText(image, f"Color: {color_state}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(image, f"Percentage: {color_percentage:.2f}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Prepare llpython output
    color_code = {"MIXED": 0, "RED": 1, "BLUE": 2, "YELLOW": 3, "OTHER": 4}
    llpython = [color_code[color_state],  # Color state
                int(color_percentage * 100),  # Percentage (0-100)
                center_x,  # Center X coordinate
                center_y,  # Center Y coordinate
                SQUARE_SIZE,  # Size of the checked square
                0, 0, 0]

    # Return empty contour, the annotated image, and llpython data
    return np.array([]), image, llpython