
import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo



#PCA9685 SETUP

class ServoHat:
    def __init__(self, frequency=50):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency



# MOTOR CLASS

class Motors:
    def __init__(self, dir1=6, pwm1=12, dir2=26, pwm2=13):
        self.dir1 = DigitalOutputDevice(dir1)
        self.pwm1 = PWMOutputDevice(pwm1)

        self.dir2 = DigitalOutputDevice(dir2)
        self.pwm2 = PWMOutputDevice(pwm2)

    def stop(self):
        self.pwm1.value = 0
        self.pwm2.value = 0

    def move_forward(self, duration, speed):
        self.dir1.off()
        self.dir2.on()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    def move_reverse(self, duration, speed):
        self.dir1.on()
        self.dir2.off()
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    def turn_right(self, duration, speed):
        self.dir1.off()
        self.dir2.off()   
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    def turn_left(self, duration, speed):
        self.dir1.on()
        self.dir2.on()   
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

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


# BASE SERVO CLASS

class SmoothServoGroup:
    def __init__(self, pca):
        self.pca = pca

    def move_smooth(self, servo_obj, start, end, delay=0.02):
        current = start
        servo_obj.angle = current
        time.sleep(delay)

        while current != end:
            distance = abs(end - current)

            # fast when far, slow when near
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



# ARM CLASS

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

    # -----------------------
    # BASIC POSITIONING
    # -----------------------
    def set_all(self, claw, orient, mid, base):
        self.claw.angle = claw
        self.orient.angle = orient
        self.mid.angle = mid
        self.base.angle = base

        self.claw_pos = claw
        self.orient_pos = orient
        self.mid_pos = mid
        self.base_pos = base

    def center(self):
        self.set_all(90, 90, 90, 90)

    # -----------------------
    # SMOOTH SETTERS (stops twiching and quick uncontrolled movements)
    # for autonomous / presets
    # -----------------------
    def set_claw(self, angle, delay=0.02):
        angle = max(0, min(180, angle))
        self.move_smooth(self.claw, self.claw_pos, angle, delay)
        self.claw_pos = angle

    def set_orient(self, angle, delay=0.02):
        angle = max(0, min(180, angle))
        self.move_smooth(self.orient, self.orient_pos, angle, delay)
        self.orient_pos = angle

    def set_mid(self, angle, delay=0.02):
        angle = max(0, min(180, angle))
        self.move_smooth(self.mid, self.mid_pos, angle, delay)
        self.mid_pos = angle

    def set_base(self, angle, delay=0.02):
        angle = max(0, min(180, angle))
        self.move_smooth(self.base, self.base_pos, angle, delay)
        self.base_pos = angle

    # -----------------------
    # DIRECT SETTERS
    # for controller
    # -----------------------
    def set_claw_direct(self, angle):
        angle = max(0, min(180, angle))
        self.claw.angle = angle
        self.claw_pos = angle

    def set_orient_direct(self, angle):
        angle = max(0, min(180, angle))
        self.orient.angle = angle
        self.orient_pos = angle

    def set_mid_direct(self, angle):
        angle = max(0, min(180, angle))
        self.mid.angle = angle
        self.mid_pos = angle

    def set_base_direct(self, angle):
        angle = max(0, min(180, angle))
        self.base.angle = angle
        self.base_pos = angle

    # -----------------------
    # SMOOTH PRESET ACTIONS
    # for autonomous use
    # -----------------------
    def open_claw_smooth(self, delay=0.02):
        self.set_claw(20, delay)

    def close_claw_smooth(self, delay=0.02):
        self.set_claw(120, delay)

    def rotate_left(self, delay=0.02):
        self.set_orient(40, delay)

    def rotate_right(self, delay=0.02):
        self.set_orient(140, delay)

    def raise_arm(self, delay=0.02):
        self.set_base(120, delay)
        self.set_mid(120, delay)

    def lower_arm(self, delay=0.02):
        self.set_base(80, delay)
        self.set_mid(80, delay)

    def fold_arm(self, delay=0.02):
        self.set_base(160, delay)
        self.set_mid(30, delay)

    # -----------------------
    # DIRECT ACTIONS
    # for controller use
    # -----------------------
    def open_claw(self):
        self.set_claw_direct(20)

    def close_claw(self):
        self.set_claw_direct(120)

    def mid_up_step(self, step=2):
        self.set_mid_direct(self.mid_pos + step)

    def mid_down_step(self, step=2):
        self.set_mid_direct(self.mid_pos - step)

    def step_up(self, step=2):
        self.set_base_direct(self.base_pos + step)

    def step_down(self, step=2):
        self.set_base_direct(self.base_pos - step)

    def rotate_left_step(self, step=4):
        self.set_orient_direct(self.orient_pos - step)

    def rotate_right_step(self, step=4):
        self.set_orient_direct(self.orient_pos + step)



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

        self.pan = servo.Servo(self.pca.channels[0])   # horizontal
        self.tilt = servo.Servo(self.pca.channels[1])  # vertical

        self.pan_pos = self.PAN_CENTER
        self.tilt_pos = self.TILT_STRAIGHT

        self.center()

    def center(self):
        self.pan.angle = self.PAN_CENTER
        self.tilt.angle = self.TILT_STRAIGHT
        self.pan_pos = self.PAN_CENTER
        self.tilt_pos = self.TILT_STRAIGHT

    def set_pan(self, angle, delay=0.02):
        angle = round(angle)
        angle = max(self.PAN_MIN, min(self.PAN_MAX, angle))
        self.move_smooth(self.pan, self.pan_pos, angle, delay)
        self.pan_pos = angle

    def set_tilt(self, angle, delay=0.02):
        angle = round(angle)
        angle = max(self.TILT_MIN, min(self.TILT_UP_MAX, angle))
        self.move_smooth(self.tilt, self.tilt_pos, angle, delay)
        self.tilt_pos = angle

    def look_center(self, delay=0.02):
        self.set_pan(self.PAN_CENTER, delay)
        self.set_tilt(self.TILT_STRAIGHT, delay)

    def look_left(self, delay=0.02):
        self.set_pan(self.PAN_MAX, delay)

    def look_right(self, delay=0.02):
        self.set_pan(self.PAN_MIN, delay)

    def look_up(self, delay=0.02):
        self.set_tilt(self.TILT_UP_MAX, delay)

    def look_straight(self, delay=0.02):
        self.set_tilt(self.TILT_STRAIGHT, delay)

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

    def step_pan(self, step=2):
        self.set_pan_direct(self.pan_pos + step)

    def step_tilt(self, step=2):
        self.set_tilt_direct(self.tilt_pos + step)


# ROBOT SYSTEM CLASS

class Robot:
    def __init__(self):
        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)

    def stop(self):
        self.motors.stop()