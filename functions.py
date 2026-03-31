import time
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


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

    # MOVEMENT FUNCTIONS 
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

    def turn_left(self, duration, speed):
        self.dir1.off()
        self.dir2.off()   
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()

    def turn_right(self, duration, speed):
        self.dir1.on()
        self.dir2.on()   
        self.pwm1.value = speed
        self.pwm2.value = speed
        time.sleep(duration)
        self.stop()



# ARM CLASS

class Arm:
    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.claw = servo.Servo(self.pca.channels[12])
        self.orient = servo.Servo(self.pca.channels[13])
        self.mid = servo.Servo(self.pca.channels[14])
        self.base = servo.Servo(self.pca.channels[15])

        # Track positions
        self.claw_pos = 90
        self.orient_pos = 90
        self.mid_pos = 90
        self.base_pos = 90

        self.set_all(90, 90, 90, 90)

    def move_smooth(self, servo_obj, start, end, delay):
        current = start

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

    def set_all(self, claw, orient, mid, base):
        self.claw.angle = claw
        self.orient.angle = orient
        self.mid.angle = mid
        self.base.angle = base

        self.claw_pos = claw
        self.orient_pos = orient
        self.mid_pos = mid
        self.base_pos = base

    # ARM ACTIONS 
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
        self.move_smooth(self.base, self.base_pos, 120, delay)
        self.base_pos = 120
        self.move_smooth(self.mid, self.mid_pos, 40, delay)
        self.mid_pos = 40

    def center(self):
        self.set_all(90, 90, 90, 90)