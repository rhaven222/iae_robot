import cv2
import numpy as np
import time

from functions import Robot


# -------------------------------
# SETTINGS
# -------------------------------
CAMERA_INDEX = 0

# Blue HSV range
LOWER_BLUE = np.array([100, 120, 70])
UPPER_BLUE = np.array([130, 255, 255])

# Minimum blob size to count as a valid target
MIN_CONTOUR_AREA = 800

# Deadband so the camera does not twitch near center
X_DEADBAND = 30
Y_DEADBAND = 30

# Proportional control gains
KP_PAN = 0.08
KP_TILT = 0.08


# Limit how much the camera can move per update
MAX_PAN_STEP = 12
MAX_TILT_STEP = 10

# Loop delay
LOOP_DELAY = 0.05

# Print status no faster than this
PRINT_INTERVAL = 0.25


# -------------------------------
# HELPERS
# -------------------------------
def clamp(value, low, high):
    return max(low, min(high, value))


def get_largest_blue_blob(frame):
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
        "center": (cx, cy),
    }, mask


# -------------------------------
# MAIN
# -------------------------------
robot = Robot()
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open camera")

# Start centered and still
robot.camera.look_center()
robot.stop()

last_print_time = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            time.sleep(0.1)
            continue

        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w // 2
        frame_cy = frame_h // 2

        blob, mask = get_largest_blue_blob(frame)

        if blob is None:
            # No blue found:
            # keep camera where it already is
            # keep robot still
            robot.stop()

            now = time.time()
            if now - last_print_time >= PRINT_INTERVAL:
                print(
                    f"NO BLUE | pan={robot.camera.pan_pos} "
                    f"tilt={robot.camera.tilt_pos}"
                )
                last_print_time = now

            time.sleep(LOOP_DELAY)
            continue

        cx, cy = blob["center"]
        area = blob["area"]

        error_x = cx - frame_cx
        error_y = cy - frame_cy

        pan_step = 0
        tilt_step = 0

        if abs(error_x) > X_DEADBAND:
            pan_step = KP_PAN * error_x

        if abs(error_y) > Y_DEADBAND:
            tilt_step = KP_TILT * error_y

        pan_step = clamp(pan_step, -MAX_PAN_STEP, MAX_PAN_STEP)
        tilt_step = clamp(tilt_step, -MAX_TILT_STEP, MAX_TILT_STEP)

        # Adjust signs here if motion is backwards
        new_pan = round(robot.camera.pan_pos - pan_step)
        new_tilt = round(robot.camera.tilt_pos - tilt_step)

        new_pan = clamp(new_pan, robot.camera.PAN_MIN, robot.camera.PAN_MAX)
        new_tilt = clamp(new_tilt, robot.camera.TILT_MIN, robot.camera.TILT_MAX)

        if new_pan != robot.camera.pan_pos:
            robot.camera.set_pan(new_pan)

        if new_tilt != robot.camera.tilt_pos:
            robot.camera.set_tilt(new_tilt)

        # Keep robot base still for now
        robot.stop()

        now = time.time()
        if now - last_print_time >= PRINT_INTERVAL:
            print(
                f"BLUE | cx={cx} cy={cy} area={int(area)} "
                f"err_x={error_x} err_y={error_y} "
                f"pan={robot.camera.pan_pos} tilt={robot.camera.tilt_pos}"
            )
            last_print_time = now

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    robot.stop()
    cap.release()