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

# motor-first thresholds
ERROR_MOTOR_THRESHOLD = 70
TURN_SPEED = 0.4

# camera tuning
KP_PAN = 0.035
KP_TILT = 0.06

MAX_PAN_STEP = 3
MAX_TILT_STEP = 5

LOOP_DELAY = 0.05
ALPHA = 0.35

PAN_UPDATE_THRESHOLD = 2
TILT_UPDATE_THRESHOLD = 1

SAFE_START_DELAY = 2.0

# -------------------------------
# INIT
# -------------------------------
robot = Robot()
cap = cv2.VideoCapture(0)

robot.camera.look_center()
robot.stop()

smooth_cx = None
smooth_cy = None

print("Safe startup delay...")
time.sleep(SAFE_START_DELAY)


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
            time.sleep(LOOP_DELAY)
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
        # MOTOR-FIRST HORIZONTAL CONTROL
        # continuous turning, no pulsing
        # -------------------------------
        turning = False

        if error_x < -ERROR_MOTOR_THRESHOLD:
            print("TURN LEFT")
            robot.motors.set_tank(TURN_SPEED, -TURN_SPEED)
            turning = True

        elif error_x > ERROR_MOTOR_THRESHOLD:
            print("TURN RIGHT")
            robot.motors.set_tank(-TURN_SPEED, TURN_SPEED)
            turning = True

        else:
            robot.stop()

        # -------------------------------
        # CAMERA CONTROL
        # tilt always active
        # pan only does fine correction when not turning
        # -------------------------------
        pan_step = 0
        tilt_step = 0

        if not turning and abs(error_x) > X_DEADBAND:
            pan_step = KP_PAN * error_x

        if abs(error_y) > Y_DEADBAND:
            tilt_step = KP_TILT * error_y

        pan_step = clamp(pan_step, -MAX_PAN_STEP, MAX_PAN_STEP)
        tilt_step = clamp(tilt_step, -MAX_TILT_STEP, MAX_TILT_STEP)

        new_pan = round(robot.camera.pan_pos - pan_step)
        new_tilt = round(robot.camera.tilt_pos - tilt_step)

        new_pan = clamp(new_pan, robot.camera.PAN_MIN, robot.camera.PAN_MAX)
        new_tilt = clamp(new_tilt, robot.camera.TILT_MIN, robot.camera.TILT_MAX)

        if abs(new_pan - robot.camera.pan_pos) >= PAN_UPDATE_THRESHOLD:
            robot.camera.set_pan_direct(new_pan)

        if abs(new_tilt - robot.camera.tilt_pos) >= TILT_UPDATE_THRESHOLD:
            robot.camera.set_tilt_direct(new_tilt)

        print(
            f"BLUE | err_x={error_x} err_y={error_y} area={area:.0f} "
            f"turning={turning} pan={robot.camera.pan_pos} tilt={robot.camera.tilt_pos}"
        )

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    robot.stop()
    cap.release()