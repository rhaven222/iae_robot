import time
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# ===== MOTOR SETUP =====
DIR1 = 6
PWM1 = 12
DIR2 = 26
PWM2 = 13

dir1 = DigitalOutputDevice(DIR1)
pwm1 = PWMOutputDevice(PWM1)

dir2 = DigitalOutputDevice(DIR2)
pwm2 = PWMOutputDevice(PWM2)

speed = 0.4



i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

claw = servo.Servo(pca.channels[12])
arm_mid = servo.Servo(pca.channels[14])

# ===== SMOOTH MOVE FUNCTION =====
def move_servo_smooth(servo_obj, start, end, delay=0.02):
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

# ===== INITIAL POSITIONS =====
claw_pos = 90
arm_pos = 90

claw.angle = claw_pos
arm_mid.angle = arm_pos

time.sleep(2)

# ===== 1. MOVE FORWARD =====
print("Moving forward")
dir1.off()
dir2.on()
pwm1.value = speed
pwm2.value = speed
time.sleep(3)
pwm1.value = 0
pwm2.value = 0

# ===== 2. RAISE ARM + OPEN CLAW =====
print("Raising arm and opening claw")
move_servo_smooth(arm_mid, arm_pos, 120)
arm_pos = 120

move_servo_smooth(claw, claw_pos, 40)
claw_pos = 40

time.sleep(1)

# ===== 3. TURN 180 =====
print("Turning 180")
dir1.on()
dir2.on()   # adjust if wrong direction
pwm1.value = speed
pwm2.value = speed
time.sleep(2.5)   # adjust for true 180
pwm1.value = 0
pwm2.value = 0

# ===== 4. LOWER ARM =====
print("Lowering arm")
move_servo_smooth(arm_mid, arm_pos, 80)
arm_pos = 80

time.sleep(1)

# ===== 5. MOVE FORWARD AGAIN =====
print("Moving forward again")
dir1.off()
dir2.on()
pwm1.value = speed
pwm2.value = speed
time.sleep(3)
pwm1.value = 0
pwm2.value = 0

print("Done")