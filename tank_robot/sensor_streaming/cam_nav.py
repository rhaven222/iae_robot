#!/usr/bin/python3
# coding=utf-8

import sys
import time
import math
import cv2
import numpy as np

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")
from common.functions import *

robot = Robot()
motor_state = create_motor_state()

FORWARD_SPEED = 0.10
TEST_TIME = 20.0

W, H = 640, 480
F = 500

K = np.array([
    [F, 0, W / 2],
    [0, F, H / 2],
    [0, 0, 1]
], dtype=np.float32)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

if not cap.isOpened():
    print("Could not open camera")
    sys.exit()

sift = cv2.SIFT_create(nfeatures=1500)

matcher = cv2.FlannBasedMatcher(
    {"algorithm": 1, "trees": 5},
    {"checks": 50}
)

map_size = 600
map_img = np.zeros((map_size, map_size, 3), dtype=np.uint8)
sparse_map = np.zeros((map_size, map_size, 3), dtype=np.uint8)

map_center_x = map_size // 2
map_center_y = map_size // 2

path_x = 0.0
path_z = 0.0
heading = 0.0

last_draw_point = (map_center_x, map_center_y)

prev_kp = None
prev_des = None

frame_count = 0
keyframes = []
map_points = []

global_R = np.eye(3)
global_t = np.zeros((3, 1))


def get_features(gray):
    keypoints, descriptors = sift.detectAndCompute(gray, None)

    if descriptors is not None:
        descriptors = np.float32(descriptors)

    return keypoints, descriptors


def get_good_matches(des1, des2):
    if des1 is None or des2 is None:
        return []

    if len(des1) < 2 or len(des2) < 2:
        return []

    matches = matcher.knnMatch(des1, des2, k=2)
    good = []

    for pair in matches:
        if len(pair) < 2:
            continue

        m, n = pair

        if m.distance < 0.70 * n.distance:
            good.append(m)

    return good


def rotation_to_yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def add_keyframe(kp, des, R_world, t_world):
    keyframes.append({
        "keypoints": kp,
        "descriptors": des,
        "R": R_world.copy(),
        "t": t_world.copy()
    })

    print(f"Saved keyframe {len(keyframes)}")


def triangulate_from_keyframe(current_kp, current_des, current_R, current_t):
    if len(keyframes) == 0:
        return

    keyframe = keyframes[-1]

    good = get_good_matches(keyframe["descriptors"], current_des)

    if len(good) < 20:
        return

    pts1 = np.float32([keyframe["keypoints"][m.queryIdx].pt for m in good])
    pts2 = np.float32([current_kp[m.trainIdx].pt for m in good])

    P1 = K @ np.hstack((keyframe["R"], keyframe["t"]))
    P2 = K @ np.hstack((current_R, current_t))

    points_4d = cv2.triangulatePoints(
        P1,
        P2,
        pts1.T,
        pts2.T
    )

    points_3d = points_4d[:3] / points_4d[3]

    for i in range(points_3d.shape[1]):
        x3d = float(points_3d[0, i])
        y3d = float(points_3d[1, i])
        z3d = float(points_3d[2, i])

        if not np.isfinite(x3d) or not np.isfinite(z3d):
            continue

        if abs(x3d) > 50 or abs(z3d) > 50:
            continue

        map_points.append((x3d, y3d, z3d))


def redraw_sparse_map():
    sparse_map[:] = 0

    cv2.putText(
        sparse_map,
        "Persistent Sparse Map",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2
    )

    for point in map_points[-3000:]:
        x3d, y3d, z3d = point

        x = int(x3d * 20 + 300)
        z = int(z3d * 20 + 300)

        if 0 <= x < map_size and 0 <= z < map_size:
            cv2.circle(sparse_map, (x, z), 1, (255, 255, 255), -1)


def save_view(feature_frame):
    display_path = map_img.copy()
    display_sparse = sparse_map.copy()

    cv2.circle(display_path, last_draw_point, 6, (0, 0, 255), -1)

    cv2.putText(
        display_path,
        "VO Path",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    path_small = cv2.resize(display_path, (426, 480))
    sparse_small = cv2.resize(display_sparse, (426, 480))
    camera_small = cv2.resize(feature_frame, (426, 480))

    viewer = np.hstack((path_small, sparse_small, camera_small))

    cv2.imwrite("sift_flann_keyframe_map_view.jpg", viewer)
    print("Saved sift_flann_keyframe_map_view.jpg")


try:
    print("Starting autonomous SIFT + FLANN VO with persistent keyframe mapping.")
    print(f"Forward speed: {FORWARD_SPEED}")
    print(f"Test time: {TEST_TIME} seconds")

    time.sleep(1)

    start_time = time.time()
    drive_robot(robot, motor_state, FORWARD_SPEED, FORWARD_SPEED)

    while True:
        elapsed_time = time.time() - start_time

        if elapsed_time > TEST_TIME:
            print("Test time complete.")
            break

        ret, frame = cap.read()

        if not ret:
            print("Camera read failed")
            time.sleep(0.1)
            continue

        frame = cv2.resize(frame, (W, H))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp, des = get_features(gray)

        feature_frame = cv2.drawKeypoints(
            frame,
            kp,
            None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        if len(keyframes) == 0 and des is not None:
            add_keyframe(kp, des, global_R, global_t)

        if prev_kp is not None and prev_des is not None:
            good = get_good_matches(prev_des, des)

            if len(good) > 30:
                pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good])

                pixel_shift = np.mean(np.linalg.norm(pts2 - pts1, axis=1))

                if pixel_shift > 2.0:
                    E, mask = cv2.findEssentialMat(
                        pts2,
                        pts1,
                        K,
                        method=cv2.RANSAC,
                        prob=0.999,
                        threshold=1.0
                    )

                    if E is not None:
                        _, R, t, pose_mask = cv2.recoverPose(E, pts2, pts1, K)

                        global_R = R @ global_R
                        global_t = global_t + global_R @ t

                        dx = float(t[0][0])
                        dz = float(t[2][0])
                        dtheta = rotation_to_yaw(R)

                        heading += dtheta

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

                        triangulate_from_keyframe(kp, des, global_R, global_t)

                        if len(good) < 80 or pixel_shift > 25:
                            add_keyframe(kp, des, global_R, global_t)

                        redraw_sparse_map()

                        print(
                            f"time={elapsed_time:.2f}s "
                            f"matches={len(good)} "
                            f"shift={pixel_shift:.2f} "
                            f"dx={dx:.3f} "
                            f"dz={dz:.3f} "
                            f"heading={math.degrees(heading):.1f} "
                            f"keyframes={len(keyframes)} "
                            f"map_points={len(map_points)}"
                        )

                else:
                    print(
                        f"time={elapsed_time:.2f}s "
                        f"matches={len(good)} "
                        f"shift={pixel_shift:.2f} "
                        f"not enough motion"
                    )
            else:
                print(
                    f"time={elapsed_time:.2f}s "
                    f"matches={len(good)} "
                    f"not enough matches"
                )

        frame_count += 1

        if frame_count % 5 == 0:
            save_view(feature_frame)

        prev_kp = kp
        prev_des = des

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping SIFT + FLANN keyframe mapping test...")

finally:
    stop_robot(robot, motor_state)
    cap.release()
    print("Stopped")