import cv2
import numpy as np
import math

# Define the lower and upper bounds for yellow in HSV
YELLOW_MIN = np.array([15, 150, 245])
YELLOW_MAX = np.array([36, 255, 255])

RED_MIN = np.array([0, 160, 200])
RED_MAX = np.array([6, 255, 255])

RED_MIN1 = np.array([167, 160, 200])
RED_MAX1 = np.array([180, 255, 255])

TOLERANCE = 0.4


def drawDecorations(image, num_rectangles):
    # Add text to the image with detection information
    cv2.putText(image, f'Yellow Rectangles: {num_rectangles}', 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 5)

def runPipeline(image, llrobot):
    # Convert BGR image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask for yellow colors
    yellow_mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX)
    yellow_mask = cv2.GaussianBlur(yellow_mask, (5, 5), 0)

    red_mask1 = cv2.inRange(hsv, RED_MIN, RED_MAX)
    red_mask2 = cv2.inRange(hsv, RED_MIN1, RED_MAX1)

    red_mask = red_mask1 | red_mask2
    red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)


    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = yellow_contours + red_contours

    # Initialize variables
    yellow_rectangles = []
    largest_area = 0
    nearest_dist = 9999999
    bestest = 0
    hui = 0

    largest_contour = np.array([[]])
    nearest_contour = np.array([[]])
    bestest_contour = np.array([[]])

    global rect
    global box

    for contour in contours:
        # Calculate the area and perimeter of the contour
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # # Filter small contours to reduce false positives
        if area < 3000:
            continue

        rect = cv2.minAreaRect(contour)
        dist = 99999
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if area * cy > 1500000:
                continue
            dist = np.sqrt((421 - cx)**2 + (121 - cy)**2)
            if(cy > 250 or cy < 60 or cx < 300 or (cx < 900 and cx > 530 and cy > 0 and cy < 200)):
                continue
            else:
                yellow_rectangles.append(contour)



        box = np.int0(cv2.boxPoints(rect))
        cv2.drawContours(image, [box], 0, (0, 0, 0), 20)

        best = area / dist

        # Keep track of the largest contour
        if area > largest_area:
            largest_area = area
            largest_contour = contour

        if dist < nearest_dist:
            nearest_dist = dist
            nearest_contour = contour

        if best > bestest:
            bestest = best
            bestest_contour = contour
            hui = area * cy






    # Draw all detected yellow rectangles
    # cv2.drawContours(image, yellow_rectangles, -1, (0, 255, 0), 2)
    if len(yellow_rectangles) == 0:
        return nearest_contour, image, [0, 0, 0, 0, 0, 0, 0, 0]

    rect = cv2.minAreaRect(bestest_contour)
    box = np.int0(cv2.boxPoints(rect))
    #anglenaklona = math.degrees(math.atan((box[3][1] - box[1][1]) / (box[3][0] - box[1][0])))
    center, size, ang = rect
    w, h = size

    if w < h:
        ang += 90

    if (min(w, h) / max(w, h) > TOLERANCE):
        ang = 90

    # box.

    # Add decorations to the image

    # Prepare data to send back to the robot
    # [number of rectangles, largest rectangle center x, largest rectangle center y]

    llpython = [1, 1, 1, 1, 1, 1, 1, 1]

    if len(yellow_rectangles) > 0:
        # Calculate the center of the largest rectangle
        M = cv2.moments(bestest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            drawDecorations(image, ang)
            ang = 0.0001368513781 * ang * ang * ang - 0.0369498720967 * ang * ang + 3.2142848374793 * ang + 0.2436739500663
            llpython = [1, cx, cy, w, h, ang, 0, 0]
            #drawDecorations(image, w + h)
            #drawDecorations(image, rect)
            #drawDecorations(image, ang)
            #drawDecorations(image, ang)
            # cv2.putText(image, f'Yellow Rectangles: {-80 + ang * 8.0 / 9.0}',
            # (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
            
            # Draw the center point of the largest rectangle
    
    return bestest_contour, image, llpython
