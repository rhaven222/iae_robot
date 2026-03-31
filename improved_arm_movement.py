import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#Improve arm movment

# I2C setup
i2c = busio.I2C(SCL, SDA)

# PCA9685 setup
pca = PCA9685(i2c)
pca.frequency = 50

# Servos
claw = servo.Servo(pca.channels[12])
claw_orient = servo.Servo(pca.channels[13])
arm_mid = servo.Servo(pca.channels[14])
arm_base = servo.Servo(pca.channels[15])

def move_servo_smooth(servo_obj, start, end, delay=0.02):
    current = start
    servo_obj.angle = current
    time.sleep(delay)

    while current != end:
        distance = abs(end - current)

        # Fast when far, slow when near
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

print("Arm demo starting")
time.sleep(2)

# Start in a middle position
print("Setting all servos to starting position")
claw.angle = 90
claw_orient.angle = 90
arm_mid.angle = 90
arm_base.angle = 90
time.sleep(4)

# Test claw open/close
print("Testing claw on channel 12")
print("Claw open")
move_servo_smooth(claw, 90, 40)
time.sleep(1)

print("Claw close")
move_servo_smooth(claw, 40, 110)
time.sleep(1)

# Test claw orientation
print("Testing claw orientation on channel 13")
print("Rotate one direction to the left")
move_servo_smooth(claw_orient, 90, 40)
time.sleep(1)

print("Rotate other to the right")
move_servo_smooth(claw_orient, 40, 140)
time.sleep(1)

# Test upper arm joint
print("Testing arm joint on channel 14")
print("Move joint one way - DOWN")
move_servo_smooth(arm_mid, 90, 60)
time.sleep(1)

print("Move joint other way - UP")
move_servo_smooth(arm_mid, 60, 120)
time.sleep(1)

# Test base arm joint
print("Testing base joint on channel 15")
print("Move base one way - DOWN")
move_servo_smooth(arm_base, 90, 60)
time.sleep(1)

print("Move base other way - UP")
move_servo_smooth(arm_base, 60, 120)
time.sleep(1)

# Return to center
print("Returning all servos to center")
move_servo_smooth(claw, 110, 90)
move_servo_smooth(claw_orient, 140, 90)
move_servo_smooth(arm_mid, 120, 90)
move_servo_smooth(arm_base, 120, 90)
time.sleep(2)