import time
from Libraries.functions import Robot
from Libraries.controller_map import PS5Controller

robot = Robot()
controller = PS5Controller()

import sys
import threading
from pathlib import Path

sys.path.append("/home/megan/ACP/python")
sys.path.append("/home/megan/MasterPi/masterpi_sdk/common_sdk")

from acpcomms.messenger import Publisher
import common.sonar as Sonar

DASH_PORT = 5556
SONAR_TOPIC = "sonar"

sonar = Sonar.Sonar()
robot_pub = Publisher(f"tcp://*:{DASH_PORT}")

left_motor = 0.0
right_motor = 0.0

CAMERA_STEP = 8.0
ARM_STEP = 4.0
MID_STEP = 4.0

def speed_to_direction(speed):
    if speed > 0:
        return "forward"
    elif speed < 0:
        return "reverse"
    else:
        return "stopped"


def robot_publish_loop():
    global left_motor, right_motor

    while True:
        try:
            dist_cm = round(sonar.getDistance() / 10.0, 2)

            robot_pub.post_message(
                {
                    "sensor_id": "Robot_001_TANK",
                    "sensor_type": "SONAR",
                    "data_type": "distance",
                    "timestamp": time.time(),
                    "units": "cm",
                    "data": {"distance": dist_cm}
                },
                SONAR_TOPIC
            )

            robot_pub.post_message(
                {
                    "left_motor": {
                        "direction": speed_to_direction(left_motor),
                        "speed": abs(left_motor)
                    },
                    "right_motor": {
                        "direction": speed_to_direction(right_motor),
                        "speed": abs(right_motor)
                    }
                },
                "motor_state"
            )

            robot_pub.post_message(
                {
                    "base_servo": robot.arm.base_pos,
                    "mid_servo": robot.arm.mid_pos,
                    "claw_orientation": robot.arm.orient_pos,
                    "claw_angle": robot.arm.claw_pos,
                    "claw_state": "closed" if robot.arm.claw_pos >= 100 else "open"
                },
                "arm_state"
            )

            robot_pub.post_message(
                {
                    "pan_angle": robot.camera.pan_pos,
                    "tilt_angle": robot.camera.tilt_pos
                },
                "camera_state"
            )

            print(f"Sonar: {dist_cm:.2f} cm")

        except Exception as e:
            print(f"Publish error: {e}")

        time.sleep(0.25)

def get_drive_label(forward, turn, move_thresh=0.15, turn_thresh=0.15):
    if abs(forward) < move_thresh and abs(turn) < turn_thresh:
        return "Stopped"

    if forward > move_thresh:
        if turn > turn_thresh:
            return "Forward left"
        elif turn < -turn_thresh:
            return "Forward right"
        return "Forward"

    if forward < -move_thresh:
        if turn > turn_thresh:
            return "Reverse left"
        elif turn < -turn_thresh:
            return "Reverse right"
        return "Reverse"

    if turn > turn_thresh:
        return "Turn left"
    if turn < -turn_thresh:
        return "Turn right"

    return "Stopped"

last_drive_label = None
arm_moving = False
camera_moving = False

threading.Thread(target=robot_publish_loop, daemon=True).start()

try:
    while True:
        state = controller.get_state()

        # -----------------------
        # DRIVE
        # -----------------------
        forward = -state["ly"]
        turn = -state["lx"]

        left_motor, right_motor = controller.get_drive()
        robot.motors.set_tank(left_motor, right_motor)

        drive_label = get_drive_label(forward, turn)
        if drive_label != last_drive_label:
            if drive_label != "Stopped":
                print(drive_label)
            last_drive_label = drive_label

        # -----------------------
        # CAMERA
        # -----------------------
        camera_is_moving = (state["rx"] != 0 or state["ry"] != 0)

        if camera_is_moving and not camera_moving:
            print("Moving camera")
            camera_moving = True

        if state["rx"] != 0:
            robot.camera.step_pan(-state["rx"] * CAMERA_STEP)

        if state["ry"] != 0:
            robot.camera.step_tilt(-state["ry"] * CAMERA_STEP)

        if not camera_is_moving and camera_moving:
            print(f"Camera angles: pan={robot.camera.pan_pos:.1f}, tilt={robot.camera.tilt_pos:.1f}")
            camera_moving = False

        # -----------------------
        # ARM
        # -----------------------
        hat_x, hat_y = state["hat"]
        arm_is_moving = False

        if hat_y == 1:
            robot.arm.step_up(ARM_STEP)
            arm_is_moving = True
        elif hat_y == -1:
            robot.arm.step_down(ARM_STEP)
            arm_is_moving = True

        if hat_x == -1:
            robot.arm.mid_down_step(MID_STEP)
            arm_is_moving = True
        elif hat_x == 1:
            robot.arm.mid_up_step(MID_STEP)
            arm_is_moving = True

        if state["l1"]:
            robot.arm.rotate_left_step(ARM_STEP)
            arm_is_moving = True

        if state["r1"]:
            robot.arm.rotate_right_step(ARM_STEP)
            arm_is_moving = True

        if state["r2"] > 0:
            if robot.arm.claw_pos != 120:
                robot.arm.close_claw()
                arm_is_moving = True

        elif state["l2"] > 0:
            if robot.arm.claw_pos != 20:
                robot.arm.open_claw()
                arm_is_moving = True

        if arm_is_moving and not arm_moving:
            print("Moving arm")
            arm_moving = True

        if not arm_is_moving and arm_moving:
            print(
                f"Arm angles: base={robot.arm.base_pos:.1f}, "
                f"mid={robot.arm.mid_pos:.1f}, "
                f"orient={robot.arm.orient_pos:.1f}, "
                f"claw={robot.arm.claw_pos:.1f}"
            )
            arm_moving = False

        time.sleep(0.03)

except KeyboardInterrupt:
    print("Stopping robot...")
    robot.motors.stop()
    controller.close()