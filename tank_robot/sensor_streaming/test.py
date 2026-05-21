#!/usr/bin/python3
# coding=utf-8

import cv2
import time

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cap = cv2.VideoCapture(CAMERA_INDEX)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("Could not open USB camera")
    exit()

# Detect strong edges and lines in the image
def detect_lines(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    edges = cv2.Canny(
        blurred,
        50,
        150
    )

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=3.14159 / 180,
        threshold=60,
        minLineLength=60,
        maxLineGap=15
    )

    output = frame.copy()

    line_count = 0

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line[0]

            cv2.line(
                output,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            line_count += 1

    return output, edges, line_count

# Save line detection debug images
def capture_photo(index=None):

    ret, img = cap.read()

    if not ret:
        print("Failed to capture photo.")
        return

    line_img, edge_img, line_count = detect_lines(img)

    timestamp = time.strftime("%Y%m%d_%H%M%S")

    suffix = f"_{index}" if index is not None else ""

    line_path = f"line_debug_{timestamp}{suffix}.jpg"
    edge_path = f"edge_debug_{timestamp}{suffix}.jpg"

    cv2.imwrite(line_path, line_img)
    cv2.imwrite(edge_path, edge_img)

    print(f"Line debug image saved at: {line_path}")
    print(f"Edge debug image saved at: {edge_path}")
    print(f"Lines detected: {line_count}")

try:

    while True:

        input("\nPress ENTER to capture line detection image...")

        capture_photo()

except KeyboardInterrupt:

    print("\nStopped")

finally:

    cap.release()