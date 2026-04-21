#!/usr/bin/env python3
# encoding: utf-8

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

        self.pan_pos = self.PAN_CENTER
        self.tilt_pos = self.TILT_STRAIGHT

        self.center()

    def center(self):
        self.set_pan_direct(self.PAN_CENTER)
        self.set_tilt_direct(self.TILT_STRAIGHT)

    def set_pan_direct(self, angle):
        angle = round(angle)
        angle = max(self.PAN_MIN, min(self.PAN_MAX, angle))
        self.pan.angle = angle
        self.pan_pos = angle

    def set_tilt_direct(self, angle):
        angle = round(angle)
        angle = max(self.TILT_MIN, min(self.TILT_UP_MAX, angle))
        self.tilt.angle = angle
        self.tilt_pos = angle


class Board:
    def __init__(self):
        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)

    def set_motor_duty(self, dutys):
        """
        Original SDK expects 4 motor duty commands like:
        [[1, val], [2, val], [3, val], [4, val]]

        Your robot is tank drive, so map 4-wheel style commands
        into left/right motor values.
        """
        vals = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        for motor_id, duty in dutys:
            vals[int(motor_id)] = float(duty)

        left = (vals[1] + vals[3]) / 2.0
        right = (vals[2] + vals[4]) / 2.0

        # Original script uses values around +/-100
        left /= 100.0
        right /= 100.0

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.motors.set_tank(left, right)

    def pwm_servo_set_position(self, duration, positions):
        """
        positions looks like:
        [[servo_id, pulse], ...]

        The stream script converts angle -> pulse before calling here.
        We convert pulse -> angle, then move all requested servos together.
        """
        moves = []

        for servo_id, pulse in positions:
            angle = self._pulse_to_angle(pulse)

            if servo_id == 1:
                moves.append(("claw", self.arm.claw, self.arm.claw_pos, angle, 0, 180))
            elif servo_id == 3:
                moves.append(("base", self.arm.base, self.arm.base_pos, angle, 0, 180))
            elif servo_id == 4:
                moves.append(("mid", self.arm.mid, self.arm.mid_pos, angle, 0, 180))
            elif servo_id == 5:
                moves.append(("orient", self.arm.orient, self.arm.orient_pos, angle, 0, 180))
            elif servo_id == 6:
                pass  # not used on your robot

        self._move_servos_together(duration, moves)

    def _move_servos_together(self, duration, moves):
        if not moves:
            return

        cleaned = []
        for name, servo_obj, start, end, min_angle, max_angle in moves:
            start = round(start)
            end = round(end)
            end = max(min_angle, min(max_angle, end))
            cleaned.append((name, servo_obj, start, end, min_angle, max_angle))

        if all(start == end for _, _, start, end, _, _ in cleaned):
            for name, servo_obj, _, end, _, _ in cleaned:
                servo_obj.angle = end
                self._update_servo_pos(name, end)
            return

        max_distance = max(abs(end - start) for _, _, start, end, _, _ in cleaned)

        if max_distance > 40:
            step = 4
        elif max_distance > 20:
            step = 2
        else:
            step = 1

        steps = max(1, (max_distance + step - 1) // step)
        delay = max(0.005, duration / steps) if duration > 0 else 0.005

        for i in range(steps + 1):
            progress = i / steps

            for name, servo_obj, start, end, min_angle, max_angle in cleaned:
                angle = round(start + (end - start) * progress)
                angle = max(min_angle, min(max_angle, angle))
                servo_obj.angle = angle

            time.sleep(delay)

        for name, servo_obj, _, end, _, _ in cleaned:
            servo_obj.angle = end
            self._update_servo_pos(name, end)

    def _update_servo_pos(self, name, angle):
        if name == "claw":
            self.arm.claw_pos = angle
        elif name == "base":
            self.arm.base_pos = angle
        elif name == "mid":
            self.arm.mid_pos = angle
        elif name == "orient":
            self.arm.orient_pos = angle

    @staticmethod
    def _pulse_to_angle(pulse):
        """
        Approximate 500–2500 us -> 0–180 degrees
        """
        angle = (float(pulse) - 500.0) * 180.0 / 2000.0
        return max(0, min(180, round(angle)))