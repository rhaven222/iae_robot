import cv2
import numpy as np
import time
from functions import Robot

robot = Robot()

# -------------------------------
# Camera tracking tuning
# -------------------------------
X_DEADBAND = 25
Y_DEADBAND = 25

KP_PAN = 0.04
KP_TILT = 0.04

PAN_LIMIT_BUFFER = 12

SEARCH_STEP = 3
LOST_TIMEOUT = 1.0

MIN_CONTOUR_AREA = 800

# Optional distance behavior
TARGET_AREA_LOW = 4500
TARGET_AREA_HIGH = 12000

# -------------------------------
# Blue color range (HSV)
# -------------------------------
LOWER_BLUE = np.array([100, 120, 70])
UPPER_BLUE = np.array([130, 255, 255])

# -------------------------------
# Start positions
# Change these to match your robot
# -------------------------------
robot.camera.set_pan(90)
robot.camera.set_tilt(20)

last_seen_time = time.time()
search_direction = 1

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open camera")
    exit()

def clamp(value, low, high):
    return max(low, min(high, value))

def get_largest_color_blob(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_CONTOUR_AREA:
        return None, mask

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2
    cy = y + h // 2

    return {
        "contour": largest,
        "area": area,
        "bbox": (x, y, w, h),
        "center": (cx, cy)
    }, mask

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w // 2
        frame_cy = frame_h // 2

        blob, mask = get_largest_color_blob(frame)

        if blob is not None:
            last_seen_time = time.time()

            cx, cy = blob["center"]
            x, y, w, h = blob["bbox"]
            area = blob["area"]

            error_x = cx - frame_cx
            error_y = cy - frame_cy

            # Draw tracking info
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
            cv2.circle(frame, (frame_cx, frame_cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"err_x={error_x} err_y={error_y}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"area={int(area)}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # -------------------------------
            # Camera pan/tilt control
            # -------------------------------
            pan_step = 0
            tilt_step = 0

            if abs(error_x) > X_DEADBAND:
                pan_step = KP_PAN * error_x

            if abs(error_y) > Y_DEADBAND:
                tilt_step = KP_TILT * error_y

            # Reverse signs here if movement is backwards on your robot
            new_pan = round(robot.camera.pan_angle - pan_step)
            new_tilt = round(robot.camera.tilt_angle + tilt_step)

            new_pan = clamp(new_pan, robot.camera.PAN_MIN, robot.camera.PAN_MAX)
            new_tilt = clamp(new_tilt, robot.camera.TILT_MIN, robot.camera.TILT_MAX)

            if new_pan != robot.camera.pan_angle:
                robot.camera.set_pan(new_pan)

            if new_tilt != robot.camera.tilt_angle:
                robot.camera.set_tilt(new_tilt)

            # -------------------------------
            # Base turning assist
            # Only turn robot if camera is near pan limits
            # -------------------------------
            if robot.camera.pan_angle <= robot.camera.PAN_MIN + PAN_LIMIT_BUFFER and error_x < -X_DEADBAND:
                robot.left()
            elif robot.camera.pan_angle >= robot.camera.PAN_MAX - PAN_LIMIT_BUFFER and error_x > X_DEADBAND:
                robot.right()
            else:
                robot.stop()

            # -------------------------------
            # Optional forward/backward using size
            # Comment this out for first test
            # -------------------------------
            """
            if abs(error_x) < 40:
                if area < TARGET_AREA_LOW:
                    robot.forward()
                elif area > TARGET_AREA_HIGH:
                    robot.backward()
                else:
                    robot.stop()
            """

        else:
            # No object found
            robot.stop()

            if time.time() - last_seen_time > LOST_TIMEOUT:
                # Simple search sweep
                next_pan = robot.camera.pan_angle + (SEARCH_STEP * search_direction)

                if next_pan >= robot.camera.PAN_MAX:
                    next_pan = robot.camera.PAN_MAX
                    search_direction = -1
                elif next_pan <= robot.camera.PAN_MIN:
                    next_pan = robot.camera.PAN_MIN
                    search_direction = 1

                robot.camera.set_pan(round(next_pan))
                time.sleep(0.05)

        cv2.imshow("Color Tracking", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

finally:
    robot.stop()
    cap.release()
    cv2.destroyAllWindows()