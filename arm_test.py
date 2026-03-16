import time
from board import SCL, SDA
#allows communication through I2C
import busio
#identifies Chip used on waveshare servo driver hat
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

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

print("Arm demo starting")
time.sleep(2)

# Start in a middle position
print("Setting all servos to starting position")
claw.angle = 90
claw_orient.angle = 90
arm_mid.angle = 90
arm_base.angle = 90
time.sleep(2)

# Test claw open/close
print("Testing claw on channel 12")
print("Claw open")
claw.angle = 40
time.sleep(2)
print("Claw close")
claw.angle = 110
time.sleep(2)

# Test claw orientation
print("Testing claw orientation on channel 13")
print("Rotate one direction")
claw_orient.angle = 40
time.sleep(2)
print("Rotate other direction")
claw_orient.angle = 140
time.sleep(2)

# Test upper arm joint
print("Testing arm joint on channel 14")
print("Move joint one way")
arm_mid.angle = 60
time.sleep(2)
print("Move joint other way")
arm_mid.angle = 120
time.sleep(2)

# Test base arm joint
print("Testing base joint on channel 15")
print("Move base one way")
arm_base.angle = 60
time.sleep(2)
print("Move base other way")
arm_base.angle = 120
time.sleep(2)

# Return to center
print("Returning all servos to center")
claw.angle = 90
claw_orient.angle = 90
arm_mid.angle = 90
arm_base.angle = 90
time.sleep(2)

