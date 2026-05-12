#!/usr/bin/python3
# coding=utf8

import sys
import time
import threading
from pathlib import Path

import sys
sys.path.append("/home/megan/ACP/python")

from acpcomms.messenger import Publisher, Subscriber, Listener

controller_path = Path('/home/megan/iae_robot')
sys.path.append(str(controller_path))

from Libraries.functions import Robot
from Libraries.controller_map import PS5Controller

robot = Robot()


# =======================
# Add common SDK path
# =======================
common_sdk_root = Path('/home/megan/MasterPi/masterpi_sdk/common_sdk')
sys.path.append(str(common_sdk_root))

from common.ros_robot2_controller_sdk import Board
import common.sonar as Sonar

# =======================
# Network config
# =======================
LAPTOP_IP = "172.20.7.135"
DASH_IP = "172.20.7.135"
STREAM_HOST = "172.20.7.135"

DASH_PORT = 5556
SONAR_TOPIC = "sonar"

# =======================
# Robot setup
# =======================
board = Board()
sonar = Sonar.Sonar()

# =======================
# Constants
# =======================
MOVE_SPEED = 70
TURN_SPEED = 55
ANGLE_STEP = 5
MIN_ANGLE = 0
MAX_ANGLE = 180

servo_angle = {
    1: 40,    # claw open
    2: 82,    # camera pan center
    3: 175,   # base straight up
    4: 102,   # mid right angle
    5: 90,    # claw orientation level
    6: 15     # camera tilt straight
}

arm_state = {
    "base_servo": 90,
    "mid_servo": 90,
    "claw_orientation": 90,
    "claw_angle": 40,
    "claw_state": "open"
}

camera_state = {
    "pan_angle": 82,
    "tilt_angle": 15
}

motor_state = {
    "left_motor": {
        "direction": "stopped",
        "speed": 0.0
    },
    "right_motor": {
        "direction": "stopped",
        "speed": 0.0
    }
}
active_motion = None
motion_end_time = 0.0

# =======================
# Motor helpers
# =======================

def stop_motors():
    """Stops both motors."""
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

def move_forward():
    """Forward (both motors same direction)."""
    update_motor_state(MOVE_SPEED, MOVE_SPEED)
    board.set_motor_duty([
        [1, MOVE_SPEED],
        [2, MOVE_SPEED],
        [3, MOVE_SPEED],
        [4, MOVE_SPEED],
    ])

def move_backward():
    """Backward."""
    update_motor_state(-MOVE_SPEED, -MOVE_SPEED)
    board.set_motor_duty([
        [1, -MOVE_SPEED],
        [2, -MOVE_SPEED],
        [3, -MOVE_SPEED],
        [4, -MOVE_SPEED],
    ])

def turn_left():
    """Left turn (tank drive)."""
    update_motor_state(-TURN_SPEED, TURN_SPEED)
    board.set_motor_duty([
        [1, -TURN_SPEED],
        [2, TURN_SPEED],
        [3, -TURN_SPEED],
        [4, TURN_SPEED],
    ])

def turn_right():
    """Right turn."""
    update_motor_state(TURN_SPEED, -TURN_SPEED)
    board.set_motor_duty([
        [1, TURN_SPEED],
        [2, -TURN_SPEED],
        [3, TURN_SPEED],
        [4, -TURN_SPEED],
    ])

def slide_left():
    """No strafing → treat as turn."""
    turn_left()

def slide_right():
    """No strafing → treat as turn."""
    turn_right()

# =======================
# Helper Functions
# =======================

def angle_to_pulse(angle):
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    return int(500 + angle * (2000 / 180))

def update_servos():
    update_arm_state(
        servo_angle[3],   # base
        servo_angle[4],   # mid
        servo_angle[5],   # claw orientation
        servo_angle[1]    # claw
    )

    update_camera_state(
        servo_angle[2],   # camera pan
        servo_angle[6]    # camera tilt
    )

    board.pwm_servo_set_position(
        0.05,
        [[s, angle_to_pulse(servo_angle[s])] for s in servo_angle]
    )

def reset_servos():
    update_servos()

def speed_to_direction(speed):
    if speed > 0:
        return "forward"
    elif speed < 0:
        return "reverse"
    else:
        return "stopped"


def update_motor_state(left_speed, right_speed):
    motor_state["left_motor"]["direction"] = speed_to_direction(left_speed)
    motor_state["left_motor"]["speed"] = abs(left_speed)

    motor_state["right_motor"]["direction"] = speed_to_direction(right_speed)
    motor_state["right_motor"]["speed"] = abs(right_speed)


def update_arm_state(base, mid, orientation, claw):
    arm_state["base_servo"] = base
    arm_state["mid_servo"] = mid
    arm_state["claw_orientation"] = orientation
    arm_state["claw_angle"] = claw

    if claw >= 100:
        arm_state["claw_state"] = "closed"
    else:
        arm_state["claw_state"] = "open"


def update_camera_state(pan, tilt):
    camera_state["pan_angle"] = pan
    camera_state["tilt_angle"] = tilt

# =======================
# Message handlers
# =======================

def handle_sonar(msg):
    print("\n--- New Sonar Data Received ---")
    try:
        sensor_id = msg["sensor_id"]
        dist = msg["data"]["distance"]
        unit = msg["units"]
        ts = msg["timestamp"]
        print(f"ID: {sensor_id} | Distance: {dist} {unit} | Time: {ts}")
    except KeyError as e:
        print("Missing sonar field:", e)

def handle_robot_command(msg):
    global active_motion, motion_end_time

    command = msg.get("data", {}).get("command")
    action = msg.get("data", {}).get("action")
    key_pressed = msg.get("data", {}).get("key_pressed", "unknown")
    duration = float(msg.get("data", {}).get("duration", 0) or 0)
    timestamp = msg.get("timestamp", 0.0)

    movement_targets = {
        "toggle_forward": "forward",
        "toggle_backward": "backward",
        "toggle_slide_left": "left",
        "toggle_slide_right": "right",
        "turn_left": "turn_left",
        "turn_right": "turn_right",
    }

    print("\n" + "=" * 60)
    print("KEY INPUT RECEIVED:")
    print(f"  Key pressed : {key_pressed}")
    print(f"  Command     : {command}")
    print(f"  Action      : {action}")
    print(f"  Timestamp   : {timestamp:.4f}")
    print("=" * 60)

    if not command:
        return

    if command in movement_targets:
        target = movement_targets[command]

        if duration > 0:
            active_motion = target
            motion_end_time = time.time() + duration
        else:
            active_motion = None if active_motion == target else target
            motion_end_time = 0.0

    if action == "servo":
        servo_cmds = {
            "servo_1_decrease": (1, -1), "servo_1_increase": (1, +1),
            "servo_2_decrease": (2, -1), "servo_2_increase": (2, +1),
            "servo_3_decrease": (3, -1), "servo_3_increase": (3, +1),
            "servo_4_decrease": (4, -1), "servo_4_increase": (4, +1),
            "servo_5_decrease": (5, -1), "servo_5_increase": (5, +1),
            "servo_6_decrease": (6, -1), "servo_6_increase": (6, +1),
        }

        if command in servo_cmds:
            sid, direction = servo_cmds[command]
            servo_angle[sid] += direction * ANGLE_STEP
            servo_angle[sid] = max(MIN_ANGLE, min(MAX_ANGLE, servo_angle[sid]))
            update_servos()

def handle_message(msg):
    if not isinstance(msg, dict):
        print("Unexpected message format:", msg)
        return

    if "sensor_id" in msg and "distance" in msg.get("data", {}):
        handle_sonar(msg)
    elif "data" in msg and "command" in msg["data"]:
        handle_robot_command(msg)
    else:
        print("Unknown message type:", msg)

def dispatch_loop(listener):
    while True:
        if listener.data_ready():
            for topic, msg in listener.get_buffer():
                handle_message(msg)
        time.sleep(0.05)

# =======================
# Robot publish loop
# =======================

robot_pub = Publisher(f"tcp://*:{DASH_PORT}")

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

            print(f"\nSonar: {dist_cm:.2f} cm")

            robot_pub.post_message(
                {
                    "sensor_id": "Robot_001_TANK",
                    "sensor_type": "ROBOT_STATE",
                    "data_type": "telemetry",
                    "timestamp": time.time(),
                    "units": "mixed",
                    "data": {
                        "sonar_cm": dist_cm,

                        "motors": {
                            "left_motor": {
                                "direction": speed_to_direction(left_motor),
                                "speed": abs(left_motor)
                            },
                            "right_motor": {
                                "direction": speed_to_direction(right_motor),
                                "speed": abs(right_motor)
                            }
                        },

                        "arm": {
                            "base_servo": robot.arm.base_pos,
                            "mid_servo": robot.arm.mid_pos,
                            "claw_orientation": robot.arm.orient_pos,
                            "claw_angle": robot.arm.claw_pos,
                            "claw_state": "closed" if robot.arm.claw_pos >= 100 else "open"
                        },

                        "camera": {
                            "pan_angle": robot.camera.pan_pos,
                            "tilt_angle": robot.camera.tilt_pos
                        }
                    }
                },
                "robot_state"
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
                    "base_servo": arm_state["base_servo"],
                    "mid_servo": arm_state["mid_servo"],
                    "claw_orientation": arm_state["claw_orientation"],
                    "claw_angle": arm_state["claw_angle"],
                    "claw_state": "closed" if arm_state["claw_angle"] >= 100 else "open"
                },
                "arm_state"
            )

            robot_pub.post_message(
                {
                    "pan_angle": camera_state["pan_angle"],
                    "tilt_angle": camera_state["tilt_angle"]
                },
                "camera_state"
            )

            print(
                f"Base: {arm_state['Base_servo']} "
                f"\nMid: {arm_state['Mid_servo']}"
                f"\nClaw Orientation: {arm_state['claw_orientation']} "
                f"\nClaw: {arm_state['claw_state']} "
                f"at {arm_state['claw_angle']}°"
            )

            print(
                f"Camera: pan={camera_state['pan_angle']} "
                f"tilt={camera_state['tilt_angle']}"
            )

            print(
                f"Left Motor: "
                f"{motor_state['left_motor']['direction']} "
                f"{motor_state['left_motor']['speed']:.2f}"
            )

            print(
                f"Right Motor: "
                f"{motor_state['right_motor']['direction']} "
                f"{motor_state['right_motor']['speed']:.2f}"
            )
    
        except Exception as e:
            print(f"Sonar read error: {e}")

        time.sleep(1)

# =======================
# Comms setup
# =======================

sub = Subscriber(f"tcp://{LAPTOP_IP}:5555", "")
sub_listener = Listener(sub)

dash_sub = Subscriber(f"tcp://{DASH_IP}:{DASH_PORT}", "")
dash_listener = Listener(dash_sub)

robot_pub = Publisher(f"tcp://*:{DASH_PORT}")

# =======================
# Start
# =======================

reset_servos()

threading.Thread(target=robot_publish_loop, daemon=True).start()
threading.Thread(target=dispatch_loop, args=(sub_listener,), daemon=True).start()
threading.Thread(target=dispatch_loop, args=(dash_listener,), daemon=True).start()

print(f"""
========================================
 Robot Stream Started
========================================
keyboard_pub   <- tcp://{LAPTOP_IP}:5555
DASH commands  <- tcp://{DASH_IP}:{DASH_PORT}
Sonar PUB       -> tcp://*:5556
========================================
""")

try:
    while True:
        time.sleep(0.1)

        if motion_end_time and time.time() >= motion_end_time:
            active_motion = None
            motion_end_time = 0.0

        if active_motion == "forward":
            move_forward()
        elif active_motion == "backward":
            move_backward()
        elif active_motion == "left":
            slide_left()
        elif active_motion == "right":
            slide_right()
        elif active_motion == "turn_left":
            turn_left()
        elif active_motion == "turn_right":
            turn_right()
        else:
            stop_motors()

except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    sub_listener.stop()
    sub.close()

    dash_listener.stop()
    dash_sub.close()

    robot_pub.close()

    stop_motors()
    reset_servos()

    print("Disconnected cleanly")