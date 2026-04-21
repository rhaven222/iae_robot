# iae_robot/irap_robot_code/common/ros_robot_controller_sdk.py

import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class ServoHat:
    def __init__(self, frequency=50):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency


class Motors:
    def __init__(self, dir1=6, pwm1=12, dir2=26, pwm2=13):
        self.dir1 = DigitalOutputDevice(dir1)
        self.pwm1 = PWMOutputDevice(pwm1)

        self.dir2 = DigitalOutputDevice(dir2)
        self.pwm2 = PWMOutputDevice(pwm2)

    def stop(self):
        self.pwm1.value = 0
        self.pwm2.value = 0

    def set_tank(self, left_speed, right_speed):
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Left motor
        if left_speed > 0:
            self.dir1.off()
            self.pwm1.value = left_speed
        elif left_speed < 0:
            self.dir1.on()
            self.pwm1.value = abs(left_speed)
        else:
            self.pwm1.value = 0

        # Right motor
        if right_speed > 0:
            self.dir2.on()
            self.pwm2.value = right_speed
        elif right_speed < 0:
            self.dir2.off()
            self.pwm2.value = abs(right_speed)
        else:
            self.pwm2.value = 0


class Arm:
    def __init__(self, pca):
        self.claw = servo.Servo(pca.channels[12])
        self.orient = servo.Servo(pca.channels[13])
        self.mid = servo.Servo(pca.channels[14])
        self.base = servo.Servo(pca.channels[15])

        self.claw_pos = 90
        self.orient_pos = 90
        self.mid_pos = 90
        self.base_pos = 90

        self.center()

    def center(self):
        self.set_claw_direct(90)
        self.set_orient_direct(90)
        self.set_mid_direct(90)
        self.set_base_direct(90)

    def set_claw_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.claw.angle = angle
        self.claw_pos = angle

    def set_orient_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.orient.angle = angle
        self.orient_pos = angle

    def set_mid_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.mid.angle = angle
        self.mid_pos = angle

    def set_base_direct(self, angle):
        angle = max(0, min(180, round(angle)))
        self.base.angle = angle
        self.base_pos = angle


class CameraServos:
    PAN_MIN = 3.5
    PAN_CENTER = 82
    PAN_MAX = 180

    TILT_MIN = 15
    TILT_STRAIGHT = 15
    TILT_UP_MAX = 100

    def __init__(self, pca):
        self.pan = servo.Servo(pca.channels[1])   # horizontal
        self.tilt = servo.Servo(pca.channels[0])  # vertical

        self.center()

    def center(self):
        self.set_pan_direct(self.PAN_CENTER)
        self.set_tilt_direct(self.TILT_STRAIGHT)

    def set_pan_direct(self, angle):
        angle = max(self.PAN_MIN, min(self.PAN_MAX, round(angle)))
        self.pan.angle = angle
        self.pan_pos = angle

    def set_tilt_direct(self, angle):
        angle = max(self.TILT_MIN, min(self.TILT_UP_MAX, round(angle)))
        self.tilt.angle = angle
        self.tilt_pos = angle


class Board:
    def __init__(self):
        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)

    def set_motor_duty(self, dutys):
        vals = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        for motor_id, duty in dutys:
            vals[int(motor_id)] = float(duty)

        # Convert 4-wheel command style to 2-motor tank drive
        left = (vals[1] + vals[3]) / 2.0
        right = (vals[2] + vals[4]) / 2.0

        # ACP script uses values around +/-100
        left /= 100.0
        right /= 100.0

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.motors.set_tank(left, right)

    def pwm_servo_set_position(self, duration, positions):
        # duration is accepted to match original interface
        # we are not using smooth timed motion yet
        for servo_id, pulse in positions:
            angle = self._pulse_to_angle(pulse)

            if servo_id == 1:
                self.arm.set_claw_direct(angle)
            elif servo_id == 3:
                self.arm.set_base_direct(angle)
            elif servo_id == 4:
                self.arm.set_mid_direct(angle)
            elif servo_id == 5:
                self.arm.set_orient_direct(angle)
            elif servo_id == 6:
                self.arm.set_orient_direct(angle)

    @staticmethod
    def _pulse_to_angle(pulse):
        # Approximate 500–2500 us -> 0–180 degrees
        angle = (float(pulse) - 500.0) * 180.0 / 2000.0
        return max(0, min(180, round(angle)))