import cv2
import numpy as np
import time
from functions import Robot

# -------------------------------
# SETTINGS
# -------------------------------
LOWER_BLUE = np.array([100, 120, 70])
UPPER_BLUE = np.array([130, 255, 255])

MIN_CONTOUR_AREA = 1200

X_DEADBAND = 25
Y_DEADBAND = 15

KP_PAN = 0.05
KP_TILT = 0.06

MAX_PAN_STEP = 6
MAX_TILT_STEP = 5

LOOP_DELAY = 0.03

ALPHA = 0.35
# -------------------------------
# INIT
# -------------------------------
robot = Robot()
cap = cv2.VideoCapture(0)

robot.camera.look_center()
robot.stop()

smooth_cx = None
smooth_cy = None


# -------------------------------
# HELPERS
# -------------------------------
def clamp(val, low, high):
    return max(low, min(high, val))


def get_blob(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area < MIN_CONTOUR_AREA:
        return None

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w // 2
    cy = y + h // 2

    return cx, cy, area


# -------------------------------
# MAIN LOOP
# -------------------------------
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h, w = frame.shape[:2]
        frame_cx = w // 2
        frame_cy = h // 2

        blob = get_blob(frame)

        if blob is None:
            robot.stop()
            print(f"NO BLUE | pan={robot.camera.pan_pos} tilt={robot.camera.tilt_pos}")
            time.sleep(LOOP_DELAY)
            continue

        cx, cy, area = blob

        # -------------------------------
        # SMOOTH TARGET POSITION
        # -------------------------------
        if smooth_cx is None:
            smooth_cx = cx
            smooth_cy = cy
        else:
            smooth_cx = int(ALPHA * cx + (1 - ALPHA) * smooth_cx)
            smooth_cy = int(ALPHA * cy + (1 - ALPHA) * smooth_cy)

        error_x = smooth_cx - frame_cx
        error_y = smooth_cy - frame_cy

        # -------------------------------
        # CONTROL
        # -------------------------------
        pan_step = 0
        tilt_step = 0

        if abs(error_x) > X_DEADBAND:
            pan_step = KP_PAN * error_x

        if abs(error_y) > Y_DEADBAND:
            tilt_step = KP_TILT * error_y

        pan_step = clamp(pan_step, -MAX_PAN_STEP, MAX_PAN_STEP)
        tilt_step = clamp(tilt_step, -MAX_TILT_STEP, MAX_TILT_STEP)

        new_pan = round(robot.camera.pan_pos - pan_step)
        new_tilt = round(robot.camera.tilt_pos + tilt_step)

        new_pan = clamp(new_pan, robot.camera.PAN_MIN, robot.camera.PAN_MAX)
        new_tilt = clamp(new_tilt, robot.camera.TILT_MIN, robot.camera.TILT_MAX)

        # move if changed by at least 1 degree
        if abs(new_pan - robot.camera.pan_pos) >= 1:
            robot.camera.set_pan_direct(new_pan)

        if abs(new_tilt - robot.camera.tilt_pos) >= 1:
            robot.camera.set_tilt_direct(new_tilt)
            
        # -------------------------------
        # SMOOTH SERVO MOVEMENT
        # -------------------------------
        if abs(new_pan - robot.camera.pan_pos) >= 2:
            robot.camera.set_pan(new_pan, delay=0.01)

        if abs(new_tilt - robot.camera.tilt_pos) >= 2:
            robot.camera.set_tilt(new_tilt, delay=0.01)

        robot.stop()

        print(
            f"BLUE | err_x={error_x} err_y={error_y} "
            f"pan={robot.camera.pan_pos} tilt={robot.camera.tilt_pos}"
        )

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    robot.stop()
    cap.release()