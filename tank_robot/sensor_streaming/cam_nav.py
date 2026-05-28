
#the robot will drive forward slowly while using it camera to estimate its movement

# This script uses the robot's camera to estimate
# how it is moving while building a simple map of the environment.
# As the robot drives forward, it tracks visual details between
# camera frames to estimate movement, heading, and position changes.
# The output includes a path map, sparse environmental map points,
# camera feature visualization, and console data showing how the
# robot believes it is moving through the environment.


import sys
import time
import math
import cv2
import numpy as np

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")
from common.functions import *


robot = Robot()
motor_state = create_motor_state()

FORWARD_SPEED = 0.20
SLOW_SPEED = 0.10
TEST_TIME = 40.0

MIN_MATCHES = 40
MIN_INLIERS = 35
MAX_PIXEL_SHIFT = 120
MIN_PIXEL_SHIFT = 2.0

RELOCALIZE_EVERY = 10
RELOCALIZE_INLIERS = 50
POSE_BLEND = 0.20

TARGET_FORWARD_DISTANCE = 6.0
TARGET_TURN_ANGLE = math.radians(90)

TURN_SPEED = 0.07
SLOW_TURN_SPEED = 0.04

KP_HEADING = 0.35
MAX_CORRECTION = 0.04

desired_heading = 0.0

motion_goal = "MOVE_FORWARD"
forward_progress = 0.0
turn_progress = 0.0


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
prev_track_ids = []

frame_count = 0
keyframes = []
map_points = []

next_track_id = 0
tracks = {}
MIN_TRACK_LENGTH = 4

global_R = np.eye(3)
global_t = np.zeros((3, 1))

tracking_lost_count = 0
robot_is_moving = False


def set_robot_speed(left_speed, right_speed):
    global robot_is_moving

    drive_robot(robot, motor_state, left_speed, right_speed)
    robot_is_moving = abs(left_speed) > 0 or abs(right_speed) > 0


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


def relocalize(kp_current, des_current):
    best_index = -1
    best_inliers = 0

    for i, keyframe in enumerate(keyframes):
        des_kf = keyframe["descriptors"]
        kp_kf = keyframe["keypoints"]

        matches = get_good_matches(des_kf, des_current)

        if len(matches) < 30:
            continue

        pts_kf = np.float32([kp_kf[m.queryIdx].pt for m in matches])
        pts_current = np.float32([kp_current[m.trainIdx].pt for m in matches])

        E, mask = cv2.findEssentialMat(
            pts_current,
            pts_kf,
            K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if mask is None:
            continue

        inliers = int(mask.sum())

        if inliers > best_inliers:
            best_inliers = inliers
            best_index = i

    return best_index, best_inliers


def rotation_to_yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def add_keyframe(kp, des, R_world, t_world):
    if des is None:
        return

    keyframes.append({
        "keypoints": kp,
        "descriptors": des,
        "R": R_world.copy(),
        "t": t_world.copy()
    })

    print(f"Saved keyframe {len(keyframes)}")


def update_tracks(kp, good_matches):
    global next_track_id

    current_track_ids = [-1] * len(kp)

    for match in good_matches:
        if match.queryIdx >= len(prev_track_ids):
            continue

        track_id = prev_track_ids[match.queryIdx]

        if track_id == -1:
            track_id = next_track_id
            next_track_id += 1

        current_track_ids[match.trainIdx] = track_id

        if track_id not in tracks:
            tracks[track_id] = []

        tracks[track_id].append({
            "frame": frame_count,
            "pt": kp[match.trainIdx].pt
        })

    return current_track_ids


def get_stable_matches(good_matches):
    stable = []

    for match in good_matches:
        if match.queryIdx >= len(prev_track_ids):
            continue

        track_id = prev_track_ids[match.queryIdx]

        if track_id in tracks and len(tracks[track_id]) >= MIN_TRACK_LENGTH:
            stable.append(match)

    return stable


def triangulate_stable_points(R, t, pts1, pts2, pose_mask, world_R, world_t):
    inlier_mask = pose_mask.ravel() > 0

    pts1_inliers = pts1[inlier_mask]
    pts2_inliers = pts2[inlier_mask]

    if len(pts1_inliers) < 8:
        return

    P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
    P2 = K @ np.hstack((R, t))

    points_4d = cv2.triangulatePoints(
        P1,
        P2,
        pts1_inliers.T,
        pts2_inliers.T
    )

    points_3d = points_4d[:3] / points_4d[3]

    for i in range(points_3d.shape[1]):
        x3d = float(points_3d[0, i])
        y3d = float(points_3d[1, i])
        z3d = float(points_3d[2, i])

        if not np.isfinite(x3d) or not np.isfinite(z3d):
            continue

        if z3d <= 0:
            continue

        if abs(x3d) > 50 or abs(z3d) > 50:
            continue

        local_point = np.array([[x3d], [y3d], [z3d]])
        world_point = world_R @ local_point + world_t

        wx = float(world_point[0][0])
        wy = float(world_point[1][0])
        wz = float(world_point[2][0])

        if not np.isfinite(wx) or not np.isfinite(wz):
            continue

        if abs(wx) > 100 or abs(wz) > 100:
            continue

        map_points.append((wx, wy, wz))


def redraw_sparse_map():
    sparse_map[:] = 0

    cv2.putText(
        sparse_map,
        "Stable Sparse Map",
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

    cv2.imwrite("sift_flann_tracked_map_view.jpg", viewer)
    print("Saved sift_flann_tracked_map_view.jpg")

def run_visual_goal_control(dz, dtheta):
    global motion_goal
    global forward_progress
    global turn_progress
    global desired_heading
    global heading

    if motion_goal == "MOVE_FORWARD":
        forward_progress += abs(dz)

        heading_error = desired_heading - heading
        correction = KP_HEADING * heading_error
        correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

        remaining = TARGET_FORWARD_DISTANCE - forward_progress

        if remaining <= 0:
            set_robot_speed(0, 0)
            motion_goal = "TURN_RIGHT"
            turn_progress = 0.0
            print("Forward goal reached -> starting turn")
            return

        if remaining < 1.0:
            base_speed = SLOW_SPEED
        else:
            base_speed = FORWARD_SPEED

        left_speed = base_speed - correction
        right_speed = base_speed + correction

        set_robot_speed(left_speed, right_speed)

        print(
            f"FORWARD_CONTROL "
            f"progress={forward_progress:.2f} "
            f"heading_error={math.degrees(heading_error):.2f} "
            f"left={left_speed:.2f} "
            f"right={right_speed:.2f}"
        )

    elif motion_goal == "TURN_RIGHT":
        turn_progress += abs(dtheta)

        remaining = TARGET_TURN_ANGLE - turn_progress

        if remaining <= 0:
            set_robot_speed(0, 0)
            motion_goal = "DONE"

            desired_heading = heading

            print("Turn goal reached -> movement complete")
            return

        if remaining < math.radians(15):
            set_robot_speed(SLOW_TURN_SPEED, -SLOW_TURN_SPEED)
        else:
            set_robot_speed(TURN_SPEED, -TURN_SPEED)

    else:
        set_robot_speed(0, 0)
        
try:
    print("Starting.")
    print(f"Forward speed: {FORWARD_SPEED}")
    print(f"Test time: {TEST_TIME} seconds")

    time.sleep(1)

    start_time = time.time()
    set_robot_speed(FORWARD_SPEED, FORWARD_SPEED)
    print("Visual goal started -> moving forward")

    while True:
        elapsed_time = time.time() - start_time

        if elapsed_time > TEST_TIME:
            print("Test time complete.")
            break

        ret, frame = cap.read()

        if not ret:
            print("Camera read failed")
            set_robot_speed(0, 0)
            time.sleep(0.1)
            continue

        frame = cv2.resize(frame, (W, H))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp, des = get_features(gray)

        if frame_count % RELOCALIZE_EVERY == 0 and len(keyframes) > 0:
            best_kf, best_inliers = relocalize(kp, des)

            if best_inliers > RELOCALIZE_INLIERS and best_kf >= 0:
                matched_keyframe = keyframes[best_kf]

                global_t = (1.0 - POSE_BLEND) * global_t + POSE_BLEND * matched_keyframe["t"]
                global_R = matched_keyframe["R"].copy()

                print(
                    f"RELOCALIZED -> smoothed correction using keyframe {best_kf} "
                    f"inliers={best_inliers}"
                )

        feature_frame = cv2.drawKeypoints(
            frame,
            kp,
            None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        if prev_kp is None:
            prev_kp = kp
            prev_des = des
            prev_track_ids = list(range(len(kp)))
            next_track_id = len(kp)

            add_keyframe(kp, des, global_R, global_t)

            frame_count += 1
            continue

        good = get_good_matches(prev_des, des)

        if len(good) > MIN_MATCHES:
            current_track_ids = update_tracks(kp, good)
            stable_matches = get_stable_matches(good)

            pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
            pts2 = np.float32([kp[m.trainIdx].pt for m in good])

            pixel_shift = np.mean(np.linalg.norm(pts2 - pts1, axis=1))

            if pixel_shift > MAX_PIXEL_SHIFT:
                tracking_lost_count += 1
                print(
                    f"Tracking unstable -> shift={pixel_shift:.2f}, "
                    f"lost_count={tracking_lost_count}"
                )

                set_robot_speed(0, 0)
                current_track_ids = [-1] * len(kp)

            elif pixel_shift > MIN_PIXEL_SHIFT:
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

                    pose_inliers = int(pose_mask.sum())

                    if pose_inliers < MIN_INLIERS:
                        tracking_lost_count += 1
                        print(
                            f"Tracking weak -> inliers={pose_inliers}, "
                            f"lost_count={tracking_lost_count}"
                        )

                        set_robot_speed(SLOW_SPEED, SLOW_SPEED)
                        current_track_ids = [-1] * len(kp)

                    else:
                        tracking_lost_count = 0

                        if not robot_is_moving:
                            set_robot_speed(FORWARD_SPEED, FORWARD_SPEED)
                        else:
                            set_robot_speed(FORWARD_SPEED, FORWARD_SPEED)

                        global_R = R @ global_R
                        global_t = global_t + global_R @ t

                        dx = float(t[0][0])
                        dz = float(t[2][0])
                        dtheta = rotation_to_yaw(R)
                        heading += dtheta

                        run_visual_goal_control(dz, dtheta)

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

                        if len(stable_matches) > 20:
                            stable_pts1 = np.float32([prev_kp[m.queryIdx].pt for m in stable_matches])
                            stable_pts2 = np.float32([kp[m.trainIdx].pt for m in stable_matches])

                            stable_E, stable_mask = cv2.findEssentialMat(
                                stable_pts2,
                                stable_pts1,
                                K,
                                method=cv2.RANSAC,
                                prob=0.999,
                                threshold=1.0
                            )

                            if stable_E is not None:
                                _, stable_R, stable_t, stable_pose_mask = cv2.recoverPose(
                                    stable_E,
                                    stable_pts2,
                                    stable_pts1,
                                    K
                                )

                                triangulate_stable_points(
                                    stable_R,
                                    stable_t,
                                    stable_pts1,
                                    stable_pts2,
                                    stable_pose_mask,
                                    global_R,
                                    global_t
                                )

                        if len(good) < 80 or pixel_shift > 25:
                            add_keyframe(kp, des, global_R, global_t)

                        redraw_sparse_map()

                        print(
                            f"time={elapsed_time:.2f}s "
                            f"matches={len(good)} "
                            f"inliers={pose_inliers} "
                            f"stable={len(stable_matches)} "
                            f"shift={pixel_shift:.2f} "
                            f"dx={dx:.3f} "
                            f"dz={dz:.3f} "
                            f"heading={math.degrees(heading):.1f} "
                            f"keyframes={len(keyframes)} "
                            f"tracks={len(tracks)} "
                            f"map_points={len(map_points)}"
                        )

                else:
                    tracking_lost_count += 1
                    print(f"Tracking failed -> no essential matrix, lost_count={tracking_lost_count}")
                    set_robot_speed(SLOW_SPEED, SLOW_SPEED)
                    current_track_ids = [-1] * len(kp)

            else:
                current_track_ids = [-1] * len(kp)
                print(
                    f"time={elapsed_time:.2f}s "
                    f"matches={len(good)} "
                    f"shift={pixel_shift:.2f} "
                    f"not enough motion"
                )

        else:
            tracking_lost_count += 1
            current_track_ids = [-1] * len(kp)

            print(
                f"Tracking weak -> matches={len(good)}, "
                f"lost_count={tracking_lost_count}"
            )

            if tracking_lost_count >= 3:
                set_robot_speed(0, 0)
                print("Tracking lost -> robot stopped")
            else:
                set_robot_speed(SLOW_SPEED, SLOW_SPEED)
                print("Tracking weak -> robot slowed")

        frame_count += 1

        if frame_count % 5 == 0:
            save_view(feature_frame)

        prev_kp = kp
        prev_des = des
        prev_track_ids = current_track_ids

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping test...")

finally:
    stop_robot(robot, motor_state)
    cap.release()
    print("Stopped")