
import sys
import time
import cv2
import numpy as np

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")

from common.functions import *

robot = Robot()
motor_state = create_motor_state()
camera_state = create_camera_state(robot)

# This script allows the robot to slowly rotate in place while
# using its camera to remember different views of the environment.
# As the robot turns, it saves visual keyframes and attempts to
# recognize when it has completed a full 360 degree rotation by
# finding the starting view again. The output includes stored
# keyframes, estimated heading information, and console data
# showing how well the current camera view matches previous views.

# =======================
# Settings
# =======================

TURN_SPEED = 0.2
RAMP_START = 0.08
RAMP_STEP = 0.02
RAMP_DELAY = 0.25

MIN_COMPARE_TIME = 15.0
MAX_TURN_TIME = 45.0

START_MATCH_THRESHOLD = 60
KEYFRAME_INTERVAL = 1.0

DEBUG_SAVE_IMAGES = False

# =======================
# Camera setup
# =======================

center_camera(robot, camera_state)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Could not open USB camera")
    sys.exit()

time.sleep(1)

# =======================
# ORB setup
# =======================

orb = cv2.ORB_create(nfeatures=1500)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

keyframes = []

# =======================
# Helpers
# =======================

def get_frame():
    ret, frame = cap.read()
    if not ret:
        return None
    return frame


def get_features(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors


def start_right_turn_smooth():
    speed = RAMP_START

    while speed < TURN_SPEED:
        drive_robot(robot, motor_state, speed, -speed)
        time.sleep(RAMP_DELAY)
        speed += RAMP_STEP

    drive_robot(robot, motor_state, TURN_SPEED, -TURN_SPEED)


def match_score(kp1, des1, kp2, des2):
    if des1 is None or des2 is None:
        return 0

    if len(des1) < 2 or len(des2) < 2:
        return 0

    matches = matcher.knnMatch(des1, des2, k=2)

    good_matches = []

    for pair in matches:
        if len(pair) < 2:
            continue

        m, n = pair

        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    if len(good_matches) < 8:
        return len(good_matches)

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    _, mask = cv2.findHomography(
        pts1,
        pts2,
        cv2.RANSAC,
        5.0
    )

    if mask is None:
        return len(good_matches)

    return int(mask.sum())


def add_keyframe(frame, keypoints, descriptors, elapsed_time):
    if descriptors is None:
        print("Skipped keyframe because descriptors were None")
        return

    keyframe_id = len(keyframes)

    keyframe = {
        "id": keyframe_id,
        "time": elapsed_time,
        "heading_estimate": None,
        "keypoints": keypoints,
        "descriptors": descriptors
    }

    keyframes.append(keyframe)

    print(f"Saved keyframe {keyframe_id} in memory at {elapsed_time:.2f}s")

    if DEBUG_SAVE_IMAGES:
        path = f"keyframe_{keyframe_id:03d}.jpg"
        cv2.imwrite(path, frame)
        print(f"Debug image saved: {path}")


def estimate_heading_from_keyframes(current_keypoints, current_descriptors):
    if len(keyframes) == 0:
        return None, 0

    best_keyframe = None
    best_score = 0

    for keyframe in keyframes:
        score = match_score(
            keyframe["keypoints"],
            keyframe["descriptors"],
            current_keypoints,
            current_descriptors
        )

        if score > best_score:
            best_score = score
            best_keyframe = keyframe

    if best_keyframe is None:
        return None, 0

    estimated_heading = best_keyframe["heading_estimate"]

    return estimated_heading, best_score


def assign_keyframe_headings(total_turn_time):
    if total_turn_time <= 0:
        return

    for keyframe in keyframes:
        heading = (keyframe["time"] / total_turn_time) * 360.0
        heading = heading % 360.0
        keyframe["heading_estimate"] = heading



try:
    print("Camera centered and pointing straight.")
    time.sleep(0.5)

    print("Taking starting frame...")

    start_frame = get_frame()

    if start_frame is None:
        print("Could not read starting camera frame.")
        sys.exit()

    start_keypoints, start_descriptors = get_features(start_frame)

    if start_descriptors is None:
        print("Could not find enough features in starting image.")
        stop_robot(robot, motor_state)
        cap.release()
        sys.exit()

    add_keyframe(start_frame, start_keypoints, start_descriptors, 0.0)

    print("Starting automatic 360 visual compass scan.")
    print("The robot will save keyframes in memory and stop when it sees the starting view again.")

    start_time = time.time()
    last_keyframe_time = 0.0

    start_right_turn_smooth()

    while True:
        elapsed_time = time.time() - start_time

        current_frame = get_frame()

        if current_frame is None:
            print("Camera frame read failed.")
            time.sleep(0.1)
            continue

        current_keypoints, current_descriptors = get_features(current_frame)

        if current_descriptors is None:
            print("No descriptors found in current frame.")
            time.sleep(0.1)
            continue

        if elapsed_time - last_keyframe_time >= KEYFRAME_INTERVAL:
            add_keyframe(
                current_frame,
                current_keypoints,
                current_descriptors,
                elapsed_time
            )

            last_keyframe_time = elapsed_time

        if elapsed_time < MIN_COMPARE_TIME:
            print(f"Time: {elapsed_time:.2f}s | building keyframes")
            time.sleep(0.1)
            continue

        start_score = match_score(
            start_keypoints,
            start_descriptors,
            current_keypoints,
            current_descriptors
        )

        print(f"Time: {elapsed_time:.2f}s | start match score: {start_score}")

        if start_score >= START_MATCH_THRESHOLD:
            print("Starting view found again. 360 scan complete.")
            break

        if elapsed_time > MAX_TURN_TIME:
            print("Max turn time reached. Stopping anyway.")
            break

        time.sleep(0.1)

    stop_robot(robot, motor_state)

    total_turn_time = time.time() - start_time
    assign_keyframe_headings(total_turn_time)

    print("")
    print("Visual compass scan complete.")
    print(f"Total keyframes stored: {len(keyframes)}")
    print(f"Estimated full turn time: {total_turn_time:.2f}s")

    print("")
    print("Testing heading estimate from current view...")

    test_frame = get_frame()

    if test_frame is not None:
        test_keypoints, test_descriptors = get_features(test_frame)

        heading, score = estimate_heading_from_keyframes(
            test_keypoints,
            test_descriptors
        )

        if heading is not None:
            print(f"Estimated heading: {heading:.1f} degrees | score: {score}")
        else:
            print("Could not estimate heading.")

except KeyboardInterrupt:
    print("Stopping visual compass test...")

finally:
    stop_robot(robot, motor_state)
    cap.release()
    print("Stopped")