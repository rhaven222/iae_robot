#!/usr/bin/python3
# coding=utf-8

import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

__all__ = [
    "Robot",
    "drive_robot",
    "stop_robot",
    "move_arm_servos",
    "move_arm_to_position",
    "center_arm",
    "move_camera_servos",
    "move_camera_to_position",
    "center_camera",
    "read_sonar_distance",
    "stream_to_dash",
    "stream_autonomous_state",
    "create_motor_state",
    "create_arm_state",
    "create_camera_state"
]

ARM_PRESETS = {
    "up": {"base": 180, "mid": 180, "orient": 90, "claw": 40},
    "ready": {"base": 175, "mid": 102, "orient": 90, "claw": 40},
    "forward_angle": {"base": 95, "mid": 165, "orient": 90, "claw": 40},
    "forward": {"base": 40, "mid": 180, "orient": 90, "claw": 40},
    "floor": {"base": 20, "mid": 180, "orient": 90, "claw": 40},
    "folded": {"base": 170, "mid": 0, "orient": 90, "claw": 40}
}

CAMERA_PRESETS = {
    "forward": {"pan": 82, "tilt": 15},
    "left": {"pan": 180, "tilt": 15},
    "right": {"pan": 3.5, "tilt": 15},
    "up": {"pan": 82, "tilt": 100}
}

# Set up the Waveshare servo hat
class ServoHat:
    def __init__(self, frequency=50):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency

# Control the two drive motors
class Motors:
    def __init__(self, dir1=6, pwm1=12, dir2=26, pwm2=13):
        self.dir1 = DigitalOutputDevice(dir1)
        self.pwm1 = PWMOutputDevice(pwm1)
        self.dir2 = DigitalOutputDevice(dir2)
        self.pwm2 = PWMOutputDevice(pwm2)

    # Stop both motors
    def stop(self):
        self.pwm1.value = 0
        self.pwm2.value = 0

    # Move forward for a set time
    def move_forward(self, duration, speed):
        self.dir1.off()
        self.dir2.on()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    # Move backward for a set time
    def move_reverse(self, duration, speed):
        self.dir1.on()
        self.dir2.off()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    # Turn right for a set time
    def turn_right(self, duration, speed):
        self.dir1.off()
        self.dir2.off()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    # Turn left for a set time
    def turn_left(self, duration, speed):
        self.dir1.on()
        self.dir2.on()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    # Set the left motor speed
    def set_left_motor(self, speed):
        speed = max(-1.0, min(1.0, speed))

        if speed > 0:
            self.dir1.off()
            self.pwm1.value = speed
        elif speed < 0:
            self.dir1.on()
            self.pwm1.value = abs(speed)
        else:
            self.pwm1.value = 0

    # Set the right motor speed
    def set_right_motor(self, speed):
        speed = max(-1.0, min(1.0, speed))

        if speed > 0:
            self.dir2.on()
            self.pwm2.value = speed
        elif speed < 0:
            self.dir2.off()
            self.pwm2.value = abs(speed)
        else:
            self.pwm2.value = 0

    # Drive using separate left and right motor speeds
    def set_tank(self, left_speed, right_speed):
        self.set_left_motor(left_speed)
        self.set_right_motor(right_speed)

# Base class for smooth servo motion
class SmoothServoGroup:
    def __init__(self, pca):
        self.pca = pca

    # Move a servo smoothly from start angle to end angle
    def move_smooth(self, servo_obj, start, end, delay=0.02):
        start = round(start)
        end = round(end)
        current = start
        servo_obj.angle = current
        time.sleep(delay)

        while current != end:
            distance = abs(end - current)

            if distance > 40:
                step = 4
            elif distance > 20:
                step = 2
            else:
                step = 1

            if end > current:
                current += step
                if current > end:
                    current = end
            else:
                current -= step
                if current < end:
                    current = end

            servo_obj.angle = current
            time.sleep(delay)

# Control the robot arm servos
class Arm(SmoothServoGroup):
    def __init__(self, pca):
        super().__init__(pca)
        self.claw = servo.Servo(self.pca.channels[12])
        self.orient = servo.Servo(self.pca.channels[13])
        self.mid = servo.Servo(self.pca.channels[14])
        self.base = servo.Servo(self.pca.channels[15])
        self.claw_pos = 90
        self.orient_pos = 90
        self.mid_pos = 90
        self.base_pos = 90
        self.set_all(90, 90, 90, 90)

    # Set all arm servos directly
    def set_all(self, claw, orient, mid, base):
        self.claw.angle = claw
        self.orient.angle = orient
        self.mid.angle = mid
        self.base.angle = base
        self.claw_pos = claw
        self.orient_pos = orient
        self.mid_pos = mid
        self.base_pos = base

    # Center the arm servos
    def center(self):
        self.set_all(90, 90, 90, 90)

    # Smoothly set the claw servo
    def set_claw(self, angle, delay=0.02):
        angle = max(0, min(180, round(angle)))
        self.move_smooth(self.claw, self.claw_pos, angle, delay)
        self.claw_pos = angle

    # Smoothly set the claw orientation servo
    def set_orient(self, angle, delay=0.02):
        angle = max(0, min(180, round(angle)))
        self.move_smooth(self.orient, self.orient_pos, angle, delay)
        self.orient_pos = angle

    # Smoothly set the middle arm servo
    def set_mid(self, angle, delay=0.02):
        angle = max(0, min(180, round(angle)))
        self.move_smooth(self.mid, self.mid_pos, angle, delay)
        self.mid_pos = angle

    # Smoothly set the base arm servo
    def set_base(self, angle, delay=0.02):
        angle = max(0, min(180, round(angle)))
        self.move_smooth(self.base, self.base_pos, angle, delay)
        self.base_pos = angle

    # Open the claw smoothly
    def open_claw_smooth(self, delay=0.02):
        self.set_claw(20, delay)

    # Close the claw smoothly
    def close_claw_smooth(self, delay=0.02):
        self.set_claw(120, delay)

    # Rotate the claw left smoothly
    def rotate_left(self, delay=0.02):
        self.set_orient(40, delay)

    # Rotate the claw right smoothly
    def rotate_right(self, delay=0.02):
        self.set_orient(140, delay)

    # Raise the arm smoothly
    def raise_arm(self, delay=0.02):
        self.set_base(120, delay)
        self.set_mid(120, delay)

    # Lower the arm smoothly
    def lower_arm(self, delay=0.02):
        self.set_base(80, delay)
        self.set_mid(80, delay)

    # Fold the arm smoothly
    def fold_arm(self, delay=0.02):
        self.set_base(160, delay)
        self.set_mid(30, delay)

    # Directly set the claw servo
    def set_claw_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.claw.angle = angle
        self.claw_pos = angle

    # Directly set the claw orientation servo
    def set_orient_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.orient.angle = angle
        self.orient_pos = angle

    # Directly set the middle arm servo
    def set_mid_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.mid.angle = angle
        self.mid_pos = angle

    # Directly set the base arm servo
    def set_base_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.base.angle = angle
        self.base_pos = angle

    # Open the claw directly
    def open_claw(self):
        self.set_claw_direct(20)

    # Close the claw directly
    def close_claw(self):
        self.set_claw_direct(120)

    # Step the middle arm servo up
    def mid_up_step(self, step=2):
        self.set_mid_direct(self.mid_pos + step)

    # Step the middle arm servo down
    def mid_down_step(self, step=2):
        self.set_mid_direct(self.mid_pos - step)

    # Step the base arm servo up
    def step_up(self, step=2):
        self.set_base_direct(self.base_pos + step)

    # Step the base arm servo down
    def step_down(self, step=2):
        self.set_base_direct(self.base_pos - step)

    # Rotate claw orientation left by one step
    def rotate_left_step(self, step=4):
        self.set_orient_direct(self.orient_pos - step)

    # Rotate claw orientation right by one step
    def rotate_right_step(self, step=4):
        self.set_orient_direct(self.orient_pos + step)

# Control the camera pan and tilt servos
class CameraServos(SmoothServoGroup):
    PAN_MIN = 3.5
    PAN_CENTER = 82
    PAN_MAX = 180
    TILT_MIN = 15
    TILT_STRAIGHT = 15
    TILT_UP_MAX = 100
    TILT_MAX = TILT_UP_MAX

    def __init__(self, pca):
        super().__init__(pca)
        self.pan = servo.Servo(self.pca.channels[1])
        self.tilt = servo.Servo(self.pca.channels[0])
        self.pan_pos = self.PAN_CENTER
        self.tilt_pos = self.TILT_STRAIGHT
        self.center()

    # Center the camera servos
    def center(self):
        self.pan.angle = self.PAN_CENTER
        self.tilt.angle = self.TILT_STRAIGHT
        self.pan_pos = self.PAN_CENTER
        self.tilt_pos = self.TILT_STRAIGHT

    # Smoothly set the camera pan servo
    def set_pan(self, angle, delay=0.02):
        angle = round(angle)
        angle = max(self.PAN_MIN, min(self.PAN_MAX, angle))
        self.move_smooth(self.pan, self.pan_pos, angle, delay)
        self.pan_pos = angle

    # Smoothly set the camera tilt servo
    def set_tilt(self, angle, delay=0.02):
        angle = round(angle)
        angle = max(self.TILT_MIN, min(self.TILT_UP_MAX, angle))
        self.move_smooth(self.tilt, self.tilt_pos, angle, delay)
        self.tilt_pos = angle

    # Look straight ahead
    def look_center(self, delay=0.02):
        self.set_pan(self.PAN_CENTER, delay)
        self.set_tilt(self.TILT_STRAIGHT, delay)

    # Look left
    def look_left(self, delay=0.02):
        self.set_pan(self.PAN_MAX, delay)

    # Look right
    def look_right(self, delay=0.02):
        self.set_pan(self.PAN_MIN, delay)

    # Look upward
    def look_up(self, delay=0.02):
        self.set_tilt(self.TILT_UP_MAX, delay)

    # Set tilt to straight ahead
    def look_straight(self, delay=0.02):
        self.set_tilt(self.TILT_STRAIGHT, delay)

    # Directly set the camera pan servo
    def set_pan_direct(self, angle):
        angle = round(angle)
        angle = max(self.PAN_MIN, min(self.PAN_MAX, angle))
        self.pan.angle = angle
        self.pan_pos = angle

    # Directly set the camera tilt servo
    def set_tilt_direct(self, angle):
        angle = round(angle)
        angle = max(self.TILT_MIN, min(self.TILT_UP_MAX, angle))
        self.tilt.angle = angle
        self.tilt_pos = angle

    # Step the camera pan servo
    def step_pan(self, step=2):
        self.set_pan_direct(self.pan_pos + step)

    # Step the camera tilt servo
    def step_tilt(self, step=2):
        self.set_tilt_direct(self.tilt_pos + step)

# Main robot class that combines motors, arm, and camera
class Robot:
    def __init__(self):
        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)

    # Stop robot motors
    def stop(self):
        self.motors.stop()

# Keep a value inside a safe range
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

# Convert speed into forward, reverse, or stopped
def speed_to_direction(speed):
    if speed > 0:
        return "forward"
    elif speed < 0:
        return "reverse"
    return "stopped"

# Create the motor telemetry dictionary
def create_motor_state():
    return {
        "left_motor": {"direction": "stopped", "speed": 0.0},
        "right_motor": {"direction": "stopped", "speed": 0.0}
    }

# Create the arm telemetry dictionary
def create_arm_state(robot):
    return {
        "base_servo": robot.arm.base_pos,
        "mid_servo": robot.arm.mid_pos,
        "claw_orientation": robot.arm.orient_pos,
        "claw_angle": robot.arm.claw_pos,
        "claw_state": "closed" if robot.arm.claw_pos >= 100 else "open"
    }

# Create the camera telemetry dictionary
def create_camera_state(robot):
    return {
        "pan_angle": robot.camera.pan_pos,
        "tilt_angle": robot.camera.tilt_pos
    }

# Update the motor telemetry dictionary
def update_motor_state(motor_state, left_speed, right_speed):
    motor_state["left_motor"]["direction"] = speed_to_direction(left_speed)
    motor_state["left_motor"]["speed"] = abs(left_speed)
    motor_state["right_motor"]["direction"] = speed_to_direction(right_speed)
    motor_state["right_motor"]["speed"] = abs(right_speed)

# Drive the robot using left and right motor speeds
def drive_robot(robot, motor_state, left_speed, right_speed):
    left_speed = clamp(left_speed, -1.0, 1.0)
    right_speed = clamp(right_speed, -1.0, 1.0)
    robot.motors.set_tank(left_speed, right_speed)
    update_motor_state(motor_state, left_speed, right_speed)

# Stop the robot and update motor telemetry
def stop_robot(robot, motor_state):
    robot.motors.stop()
    update_motor_state(motor_state, 0.0, 0.0)

# Update the arm telemetry dictionary
def update_arm_state(robot, arm_state):
    arm_state["base_servo"] = robot.arm.base_pos
    arm_state["mid_servo"] = robot.arm.mid_pos
    arm_state["claw_orientation"] = robot.arm.orient_pos
    arm_state["claw_angle"] = robot.arm.claw_pos
    arm_state["claw_state"] = "closed" if robot.arm.claw_pos >= 100 else "open"

# Move one or more arm servos smoothly
def move_arm_servos(
    robot,
    arm_state,
    base_angle=None,
    mid_angle=None,
    claw_orientation=None,
    claw_angle=None
):
    if base_angle is not None:
        robot.arm.set_base(base_angle)
    if mid_angle is not None:
        robot.arm.set_mid(mid_angle)
    if claw_orientation is not None:
        robot.arm.set_orient(claw_orientation)
    if claw_angle is not None:
        robot.arm.set_claw(claw_angle)
    update_arm_state(robot, arm_state)

# Move the arm to a preset position
def move_arm_to_position(robot, arm_state, position_name):
    if position_name not in ARM_PRESETS:
        print(f"Unknown arm position: {position_name}")
        return
    preset = ARM_PRESETS[position_name]
    move_arm_servos(
        robot,
        arm_state,
        base_angle=preset["base"],
        mid_angle=preset["mid"],
        claw_orientation=preset["orient"],
        claw_angle=preset["claw"]
    )

# Move the arm to the ready position
def center_arm(robot, arm_state):
    move_arm_to_position(robot, arm_state, "ready")

# Update the camera telemetry dictionary
def update_camera_state(robot, camera_state):
    camera_state["pan_angle"] = robot.camera.pan_pos
    camera_state["tilt_angle"] = robot.camera.tilt_pos

# Move one or more camera servos smoothly
def move_camera_servos(
    robot,
    camera_state,
    pan_angle=None,
    tilt_angle=None
):
    if pan_angle is not None:
        robot.camera.set_pan(pan_angle)
    if tilt_angle is not None:
        robot.camera.set_tilt(tilt_angle)
    update_camera_state(robot, camera_state)

# Move the camera to a preset position
def move_camera_to_position(robot, camera_state, position_name):
    if position_name not in CAMERA_PRESETS:
        print(f"Unknown camera position: {position_name}")
        return
    preset = CAMERA_PRESETS[position_name]
    move_camera_servos(
        robot,
        camera_state,
        pan_angle=preset["pan"],
        tilt_angle=preset["tilt"]
    )

# Move the camera to the forward position
def center_camera(robot, camera_state):
    move_camera_to_position(robot, camera_state, "forward")

# Read sonar distance in centimeters
def read_sonar_distance(sonar):
    try:
        return round(sonar.getDistance() / 10.0, 2)
    except Exception as e:
        print(f"Sonar read error: {e}")
        return 0.0

# Stream sonar, motor, arm, and camera telemetry to DASH
def stream_to_dash(
    robot_pub,
    sonar,
    robot,
    motor_state,
    arm_state,
    camera_state
):
    distance_cm = read_sonar_distance(sonar)
    update_arm_state(robot, arm_state)
    update_camera_state(robot, camera_state)
    robot_pub.post_message(
        {
            "sensor_id": "Robot_001_TANK",
            "sensor_type": "SONAR",
            "data_type": "distance",
            "timestamp": time.time(),
            "units": "cm",
            "data": {"distance": distance_cm}
        },
        "sonar"
    )
    robot_pub.post_message(motor_state, "motor_state")
    robot_pub.post_message(arm_state, "arm_state")
    robot_pub.post_message(camera_state, "camera_state")

# Stream autonomous navigation state to DASH
def stream_autonomous_state(
    robot_pub,
    mode,
    last_distance_cm,
    scan_state
):
    robot_pub.post_message(
        {
            "mode": mode,
            "last_distance_cm": last_distance_cm,
            "scan": scan_state,
            "timestamp": time.time()
        },
        "autonomous_state"
    )