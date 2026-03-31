
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
        self.dir2.off()   # change if turn direction is wrong
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    def turn_left(self, duration, speed):
        self.dir1.on()
        self.dir2.on()   # change if turn direction is wrong
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()



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

    def open_claw(self, delay=0.02):
        self.move_smooth(self.claw, self.claw_pos, 40, delay)
        self.claw_pos = 40

    def close_claw(self, delay=0.02):
        self.move_smooth(self.claw, self.claw_pos, 110, delay)
        self.claw_pos = 110

    def rotate_left(self, delay=0.02):
        self.move_smooth(self.orient, self.orient_pos, 40, delay)
        self.orient_pos = 40

    def rotate_right(self, delay=0.02):
        self.move_smooth(self.orient, self.orient_pos, 140, delay)
        self.orient_pos = 140

    def raise_arm(self, delay=0.02):
        self.move_smooth(self.base, self.base_pos, 120, delay)
        self.base_pos = 120
        self.move_smooth(self.mid, self.mid_pos, 120, delay)
        self.mid_pos = 120

    def lower_arm(self, delay=0.02):
        self.move_smooth(self.base, self.base_pos, 80, delay)
        self.base_pos = 80
        self.move_smooth(self.mid, self.mid_pos, 80, delay)
        self.mid_pos = 80
    
    def fold_arm(self, delay=0.02):
        self.move_smooth(self.base, self.base_pos, 160, delay)
        self.base_pos = 160
        self.move_smooth(self.mid, self.mid_pos, 30, delay)
        self.mid_pos = 30

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



# CAMERA SERVO CLASS


class CameraServos(SmoothServoGroup):
    def __init__(self, pca):
        super().__init__(pca)

        self.tilt = servo.Servo(self.pca.channels[0])  # vertical
        self.pan = servo.Servo(self.pca.channels[1])   # horizontal

        # correct "straight ahead"
        self.tilt_pos = 20
        self.pan_pos = 90

        self.tilt.angle = self.tilt_pos
        self.pan.angle = self.pan_pos

    def center(self, delay=0.02):
        self.move_smooth(self.pan, self.pan_pos, 90, delay)
        self.pan_pos = 90

        self.move_smooth(self.tilt, self.tilt_pos, 20, delay)
        self.tilt_pos = 20

    def look_left(self, delay=0.02):
        self.move_smooth(self.pan, self.pan_pos, 0, delay)
        self.pan_pos = 0

    def look_right(self, delay=0.02):
        self.move_smooth(self.pan, self.pan_pos, 180, delay)
        self.pan_pos = 180

    def look_straight(self, delay=0.02):
        self.move_smooth(self.pan, self.pan_pos, 90, delay)
        self.pan_pos = 90

    def look_up(self, delay=0.02):
        self.move_smooth(self.tilt, self.tilt_pos, 145, delay)
        self.tilt_pos = 145

    def look_forward(self, delay=0.02):
        self.move_smooth(self.tilt, self.tilt_pos, 20, delay)
        self.tilt_pos = 20

    def set_pan(self, angle, delay=0.02):
        angle = max(0, min(180, angle))
        self.move_smooth(self.pan, self.pan_pos, angle, delay)
        self.pan_pos = angle

    def set_tilt(self, angle, delay=0.02):
        angle = max(15, min(145, angle))  # protect servo
        self.move_smooth(self.tilt, self.tilt_pos, angle, delay)
        self.tilt_pos = angle



# ROBOT SYSTEM CLASS

class Robot:
    def __init__(self):
        self.servo_hat = ServoHat()
        self.motors = Motors()
        self.arm = Arm(self.servo_hat.pca)
        self.camera = CameraServos(self.servo_hat.pca)