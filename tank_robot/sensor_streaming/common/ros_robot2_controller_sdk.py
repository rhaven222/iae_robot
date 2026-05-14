#!/usr/bin/env python3
# encoding: utf-8

import enum
import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class PacketControllerState(enum.IntEnum):
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1
    PACKET_CONTROLLER_STATE_LENGTH = 2
    PACKET_CONTROLLER_STATE_FUNCTION = 3
    PACKET_CONTROLLER_STATE_ID = 4
    PACKET_CONTROLLER_STATE_DATA = 5
    PACKET_CONTROLLER_STATE_CHECKSUM = 6


class PacketFunction(enum.IntEnum):
    PACKET_FUNC_SYS = 0
    PACKET_FUNC_LED = 1
    PACKET_FUNC_BUZZER = 2
    PACKET_FUNC_MOTOR = 3
    PACKET_FUNC_PWM_SERVO = 4
    PACKET_FUNC_BUS_SERVO = 5
    PACKET_FUNC_KEY = 6
    PACKET_FUNC_IMU = 7
    PACKET_FUNC_GAMEPAD = 8
    PACKET_FUNC_SBUS = 9
    PACKET_FUNC_OLED = 10
    PACKET_FUNC_RGB = 11
    PACKET_FUNC_NONE = 12


class PacketReportKeyEvents(enum.IntEnum):
    KEY_EVENT_PRESSED = 0x01
    KEY_EVENT_LONGPRESS = 0x02
    KEY_EVENT_LONGPRESS_REPEAT = 0x04
    KEY_EVENT_RELEASE_FROM_LP = 0x08
    KEY_EVENT_RELEASE_FROM_SP = 0x10
    KEY_EVENT_CLICK = 0x20
    KEY_EVENT_DOUBLE_CLICK = 0x40
    KEY_EVENT_TRIPLE_CLICK = 0x80


class SBusStatus:
    def __init__(self):
        self.channels = [0] * 16
        self.channel_17 = False
        self.channel_18 = False
        self.signal_loss = True
        self.fail_safe = False


def checksum_crc8(data):
    return 0


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

        if left_speed > 0:
            self.dir1.off()
            self.pwm1.value = left_speed
        elif left_speed < 0:
            self.dir1.on()
            self.pwm1.value = abs(left_speed)
        else:
            self.pwm1.value = 0

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
        self.pan = servo.Servo(pca.channels[1])
        self.tilt = servo.Servo(pca.channels[0])

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

    def step_pan(self, step):
        self.set_pan_direct(self.pan_pos + step)

    def step_tilt(self, step):
        self.set_tilt_direct(self.tilt_pos + step)


class Board:
    def __init__(self, device="/dev/ttyAMA0", baudrate=1000000, timeout=5):
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.enable_recv = False

        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def set_motor_duty(self, dutys):
        vals = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        for motor_id, duty in dutys:
            vals[int(motor_id)] = float(duty)

        left = (vals[1] + vals[3]) / 2.0
        right = (vals[2] + vals[4]) / 2.0

        left /= 100.0
        right /= 100.0

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.motors.set_tank(left, right)

    def set_motor_speed(self, speeds):
        dutys = []

        for motor_id, speed in speeds:
            speed = float(speed)

            if -1.0 <= speed <= 1.0:
                speed *= 100.0

            dutys.append([motor_id, speed])

        self.set_motor_duty(dutys)

    def pwm_servo_set_position(self, duration, positions):
        moves = []

        for servo_id, pulse in positions:
            servo_id = int(servo_id)
            angle = self._pulse_to_angle(pulse)

            if servo_id == 1:
                moves.append(("claw", self.arm.claw, self.arm.claw_pos, angle, 0, 180))
            elif servo_id == 3:
                moves.append(("base", self.arm.base, self.arm.base_pos, angle, 0, 180))
            elif servo_id == 4:
                moves.append(("mid", self.arm.mid, self.arm.mid_pos, angle, 0, 180))
            elif servo_id == 5:
                moves.append(("orient", self.arm.orient, self.arm.orient_pos, angle, 0, 180))
            elif servo_id == 2:
                moves.append(
                    (
                        "camera_pan",
                        self.camera.pan,
                        self.camera.pan_pos,
                        angle,
                        self.camera.PAN_MIN,
                        self.camera.PAN_MAX,
                    )
                )
            elif servo_id == 6:
                moves.append(
                    (
                        "camera_tilt",
                        self.camera.tilt,
                        self.camera.tilt_pos,
                        angle,
                        self.camera.TILT_MIN,
                        self.camera.TILT_UP_MAX,
                    )
                )

        self._move_servos_together(duration, moves)

    def pwm_servo_set_offset(self, servo_id, offset):
        pass

    def pwm_servo_read_position(self, servo_id):
        servo_id = int(servo_id)

        if servo_id == 1:
            return self._angle_to_pulse(self.arm.claw_pos)
        elif servo_id == 2:
            return self._angle_to_pulse(self.camera.pan_pos)
        elif servo_id == 3:
            return self._angle_to_pulse(self.arm.base_pos)
        elif servo_id == 4:
            return self._angle_to_pulse(self.arm.mid_pos)
        elif servo_id == 5:
            return self._angle_to_pulse(self.arm.orient_pos)
        elif servo_id == 6:
            return self._angle_to_pulse(self.camera.tilt_pos)

        return None

    def pwm_servo_read_offset(self, servo_id):
        return 0

    def get_battery(self):
        return None

    def get_button(self):
        return None

    def get_imu(self):
        return None

    def get_gamepad(self):
        return None

    def get_sbus(self):
        return None

    def set_led(self, on_time, off_time, repeat=1, led_id=1):
        pass

    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        pass

    def set_oled_text(self, line, text):
        pass

    def set_rgb(self, pixels):
        pass

    def buf_write(self, func, data):
        pass

    def bus_servo_enable_torque(self, servo_id, enable):
        pass

    def bus_servo_set_id(self, servo_id_now, servo_id_new):
        pass

    def bus_servo_set_offset(self, servo_id, offset):
        pass

    def bus_servo_save_offset(self, servo_id):
        pass

    def bus_servo_set_angle_limit(self, servo_id, limit):
        pass

    def bus_servo_set_vin_limit(self, servo_id, limit):
        pass

    def bus_servo_set_temp_limit(self, servo_id, limit):
        pass

    def bus_servo_stop(self, servo_id):
        pass

    def bus_servo_set_position(self, duration, positions):
        self.pwm_servo_set_position(duration, positions)

    def bus_servo_read_id(self, servo_id=254):
        return None

    def bus_servo_read_offset(self, servo_id):
        return None

    def bus_servo_read_position(self, servo_id):
        return None

    def bus_servo_read_vin(self, servo_id):
        return None

    def bus_servo_read_temp(self, servo_id):
        return None

    def bus_servo_read_temp_limit(self, servo_id):
        return None

    def bus_servo_read_angle_limit(self, servo_id):
        return None

    def bus_servo_read_vin_limit(self, servo_id):
        return None

    def bus_servo_read_torque_state(self, servo_id):
        return None

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

        steps = max(1, int((max_distance + step - 1) // step))
        delay = max(0.005, float(duration) / steps) if duration > 0 else 0.005

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
        elif name == "camera_pan":
            self.camera.pan_pos = angle
        elif name == "camera_tilt":
            self.camera.tilt_pos = angle

    @staticmethod
    def _pulse_to_angle(pulse):
        angle = (float(pulse) - 500.0) * 180.0 / 2000.0
        return max(0, min(180, round(angle)))

    @staticmethod
    def _angle_to_pulse(angle):
        return int(500 + (float(angle) / 180.0) * 2000.0)


if __name__ == "__main__":
    board = Board()

    print("Testing motors...")
    board.set_motor_duty([[1, -50], [2, 50], [3, 50], [4, -50]])
    time.sleep(1)
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

    print("Testing servos...")
    board.pwm_servo_set_position(1.0, [
        [1, 1500],
        [2, 1500],
        [3, 1700],
        [4, 1300],
        [5, 1600],
        [6, 1500],
    ])

    print("DONE")