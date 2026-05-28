# Robot camera visual navigation test
# The robot drives forward slowly while the camera estimates drift and heading change.
# It uses optical flow to track visual points between frames, then uses Essential Matrix
# pose estimation to decide if the robot is drifting left or right.

import sys
import time
import math
import cv2
import numpy as np

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")
from common.functions import *


robot = Robot()
motor_state = create_motor_state()

W, H = 640, 480
F = 500

K = np.array([
    [F, 0, W / 2],
    [0, F, H / 2],
    [0, 0, 1]
], dtype=np.float32)

FORWARD_SPEED = 0.22
SLOW_SPEED = 0.15
TEST_TIME = 30.0

MIN_TRACKED_POINTS = 60
MIN_POSE_INLIERS = 40
MAX_PIXEL_SHIFT = 120
MIN_PIXEL_SHIFT = 2.0

KP_HEADING = 0.30
MAX_CORRECTION = 0.05

FEATURE_REFRESH_COUNT = 80

heading = 0.0
desired_heading = 0.0
tracking_lost_count = 0
robot_is_moving = False

map_size = 600
map_img = np.zeros((map_size, map_size, 3), dtype=np.uint8)
map_center_x = map_size // 2
map_center_y = map_size // 2
last_draw_point = (map_center_x, map_center_y)

path_x = 0.0
path_z = 0.0


def set_robot_speed(left_speed, right_speed):
    global robot_is_moving

    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    drive_robot(robot, motor_state, left_speed, right_speed)

    robot_is_moving = abs(left_speed) > 0.01 or abs(right_speed) > 0.01


def stop():
    set_robot_speed(0, 0)


def detect_features(gray):
    points = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=500,
        qualityLevel=0.01,
        minDistance=8,
        blockSize=7
    )

    return points


def track_features(prev_gray, gray, prev_points):
    if prev_points is None or len(prev_points) == 0:
        return None, None, None

    next_points, status, error = cv2.calcOpticalFlowPyrLK(
        prev_gray,
        gray,
        prev_points,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            30,
            0.01
        )
    )

    if next_points is None or status is None:
        return None, None, None

    status = status.reshape(-1)

    good_prev = prev_points[status == 1]
    good_next = next_points[status == 1]

    return good_prev, good_next, status


def rotation_to_yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def estimate_motion(prev_points, current_points):
    if prev_points is None or current_points is None:
        return None

    if len(prev_points) < MIN_TRACKED_POINTS:
        return None

    pts1 = prev_points.reshape(-1, 2).astype(np.float32)
    pts2 = current_points.reshape(-1, 2).astype(np.float32)

    pixel_shift = np.mean(np.linalg.norm(pts2 - pts1, axis=1))

    if pixel_shift > MAX_PIXEL_SHIFT:
        return {
            "valid": False,
            "reason": "pixel shift too large",
            "pixel_shift": pixel_shift
        }

    if pixel_shift < MIN_PIXEL_SHIFT:
        return {
            "valid": False,
            "reason": "not enough motion",
            "pixel_shift": pixel_shift
        }

    E, mask = cv2.findEssentialMat(
        pts2,
        pts1,
        K,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=1.0
    )

    if E is None or mask is None:
        return {
            "valid": False,
            "reason": "essential matrix failed",
            "pixel_shift": pixel_shift
        }

    _, R, t, pose_mask = cv2.recoverPose(E, pts2, pts1, K)

    pose_inliers = int(np.sum(pose_mask > 0))

    if pose_inliers < MIN_POSE_INLIERS:
        return {
            "valid": False,
            "reason": "not enough pose inliers",
            "pixel_shift": pixel_shift,
            "inliers": pose_inliers
        }

    dx = float(t[0][0])
    dz = float(t[2][0])
    dtheta = rotation_to_yaw(R)

    return {
        "valid": True,
        "dx": dx,
        "dz": dz,
        "dtheta": dtheta,
        "pixel_shift": pixel_shift,
        "inliers": pose_inliers
    }


def run_heading_control(dtheta):
    global heading

    heading += dtheta

    heading_error = desired_heading - heading

    correction = KP_HEADING * heading_error
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left_speed = FORWARD_SPEED - correction
    right_speed = FORWARD_SPEED + correction

    set_robot_speed(left_speed, right_speed)

    return heading_error, left_speed, right_speed


def draw_path(dz):
    global path_x
    global path_z
    global last_draw_point

    scale = 30.0

    path_x += math.sin(heading) * dz * scale
    path_z += math.cos(heading) * dz * scale

    draw_x = int(map_center_x + path_x)
    draw_y = int(map_center_y - path_z)

    draw_x = max(0, min(map_size - 1, draw_x))
    draw_y = max(0, min(map_size - 1, draw_y))

    current_draw_point = (draw_x, draw_y)

    cv2.line(
        map_img,
        last_draw_point,
        current_draw_point,
        (0, 255, 0),
        2
    )

    last_draw_point = current_draw_point


def save_view(frame, points):
    display_frame = frame.copy()

    if points is not None:
        for point in points.reshape(-1, 2):
            x, y = point
            cv2.circle(display_frame, (int(x), int(y)), 2, (0, 255, 0), -1)

    path_view = map_img.copy()

    cv2.circle(path_view, last_draw_point, 6, (0, 0, 255), -1)

    cv2.putText(
        path_view,
        "Visual Odometry Path",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    frame_small = cv2.resize(display_frame, (640, 480))
    path_small = cv2.resize(path_view, (640, 480))

    viewer = np.hstack((frame_small, path_small))

    cv2.imwrite("robot_visual_navigation_view.jpg", viewer)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

if not cap.isOpened():
    print("Could not open camera")
    sys.exit()


try:
    print("Starting camera based robot navigation test")
    print(f"Forward speed: {FORWARD_SPEED}")
    print(f"Test time: {TEST_TIME} seconds")

    ret, frame = cap.read()

    if not ret:
        print("Could not read first camera frame")
        sys.exit()

    frame = cv2.resize(frame, (W, H))
    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    prev_points = detect_features(prev_gray)

    if prev_points is None:
        print("No features detected")
        sys.exit()

    print(f"Initial features: {len(prev_points)}")
    print("Robot starting forward motion")

    set_robot_speed(FORWARD_SPEED, FORWARD_SPEED)

    start_time = time.time()
    frame_count = 0

    while True:
        elapsed_time = time.time() - start_time

        if elapsed_time > TEST_TIME:
            print("Test complete")
            break

        ret, frame = cap.read()

        if not ret:
            print("Camera read failed")
            stop()
            time.sleep(0.1)
            continue

        frame = cv2.resize(frame, (W, H))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        prev_good, current_good, status = track_features(
            prev_gray,
            gray,
            prev_points
        )

        if current_good is None or len(current_good) < MIN_TRACKED_POINTS:
            tracking_lost_count += 1

            print(
                f"Tracking weak: points={0 if current_good is None else len(current_good)}, "
                f"lost_count={tracking_lost_count}"
            )

            if tracking_lost_count >= 3:
                stop()
                print("Tracking lost, robot stopped")
            else:
                set_robot_speed(SLOW_SPEED, SLOW_SPEED)
                print("Tracking weak, robot slowed")

            prev_points = detect_features(gray)
            prev_gray = gray.copy()
            frame_count += 1
            continue

        motion = estimate_motion(prev_good, current_good)

        if motion is None or not motion["valid"]:
            tracking_lost_count += 1

            reason = "unknown" if motion is None else motion["reason"]
            shift = 0.0 if motion is None else motion["pixel_shift"]

            print(
                f"Motion estimate failed: {reason}, "
                f"shift={shift:.2f}, "
                f"lost_count={tracking_lost_count}"
            )

            if tracking_lost_count >= 3:
                stop()
                print("Motion tracking lost, robot stopped")
            else:
                set_robot_speed(SLOW_SPEED, SLOW_SPEED)
                print("Motion weak, robot slowed")

        else:
            tracking_lost_count = 0

            dx = motion["dx"]
            dz = motion["dz"]
            dtheta = motion["dtheta"]

            heading_error, left_speed, right_speed = run_heading_control(dtheta)

            draw_path(dz)

            print(
                f"time={elapsed_time:.2f}s "
                f"tracked={len(current_good)} "
                f"inliers={motion['inliers']} "
                f"shift={motion['pixel_shift']:.2f} "
                f"dx={dx:.3f} "
                f"dz={dz:.3f} "
                f"dtheta={math.degrees(dtheta):.2f}deg "
                f"heading={math.degrees(heading):.2f}deg "
                f"heading_error={math.degrees(heading_error):.2f}deg "
                f"left={left_speed:.2f} "
                f"right={right_speed:.2f}"
            )

        if len(current_good) < FEATURE_REFRESH_COUNT:
            prev_points = detect_features(gray)
        else:
            prev_points = current_good.reshape(-1, 1, 2)

        prev_gray = gray.copy()

        if frame_count % 5 == 0:
            save_view(frame, prev_points)

        frame_count += 1
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping test")

finally:
    stop_robot(robot, motor_state)
    cap.release()
    print("Stopped")