#!/usr/bin/python3
# coding=utf-8

import sys
import time
import cv2

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")

from common.functions import *

robot = Robot()
motor_state = create_motor_state()
camera_state = create_camera_state(robot)

TURN_SPEED = 0.12
RAMP_STEPS = [0.04, 0.06, 0.08, 0.10, 0.12]
RAMP_DELAY = 0.2

MIN_TURN_TIME = 6.0
MAX_TURN_TIME = 22.0
MATCH_THRESHOLD = 45

center_camera(robot, camera_state)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Could not open USB camera")
    sys.exit()

time.sleep(1)

orb = cv2.ORB_create(nfeatures=1000)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Slowly start the turn to reduce brownout current spikes
def start_smooth_right_turn():
    for speed in RAMP_STEPS:
        drive_robot(robot, motor_state, speed, -speed)
        time.sleep(RAMP_DELAY)

# Get a frame from the USB camera
def get_frame():
    ret, frame = cap.read()

    if not ret:
        return None

    return frame

# Get ORB visual features from a frame
def get_features(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors

# Compare the current image to the starting image
def compare_frames(start_descriptors, current_descriptors):
    if start_descriptors is None or current_descriptors is None:
        return 0

    matches = matcher.match(start_descriptors, current_descriptors)
    matches = sorted(matches, key=lambda x: x.distance)

    good_matches = []

    for match in matches:
        if match.distance < 60:
            good_matches.append(match)

    return len(good_matches)

try:
    print("Camera centered and pointing straight.")
    time.sleep(0.5)

    print("Taking starting picture...")
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

    print("Starting visual 360 turn with smooth ramp...")

    start_time = time.time()

    start_smooth_right_turn()

    while True:
        elapsed_time = time.time() - start_time

        current_frame = get_frame()

        if current_frame is None:
            print("Camera frame read failed.")
            continue

        current_keypoints, current_descriptors = get_features(current_frame)

        match_score = compare_frames(start_descriptors, current_descriptors)

        print(f"Time: {elapsed_time:.2f}s | Match score: {match_score}")

        if elapsed_time > MIN_TURN_TIME and match_score >= MATCH_THRESHOLD:
            print("Starting view found again. 360 turn complete.")
            break

        if elapsed_time > MAX_TURN_TIME:
            print("Max turn time reached. Stopping anyway.")
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping visual 360 test...")

finally:
    stop_robot(robot, motor_state)
    cap.release()
    print("Stopped")