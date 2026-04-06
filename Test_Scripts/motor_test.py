#Test motors direction and Identity
# moves right motor(1) forward and reverese then does the same for the left motor(2)

from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

DIR1 = 6
PWM1 = 12

DIR2 = 26
PWM2 = 13

dir1 = DigitalOutputDevice(6)
pwm1 = PWMOutputDevice(12)

dir2 = DigitalOutputDevice(26)
pwm2 = PWMOutputDevice(13)

speed = 0.4

time.sleep(3)

#motor 1 = right
print("testing motor 1 forward")
dir1.off() #on/off changes direction
pwm1.value = speed
time.sleep(2)
pwm1.value = 0
time.sleep(2)


print("testing motor 1 reverse")
dir1.on() #on/off changes direction
pwm1.value = speed
time.sleep(2)
pwm1.value = 0
time.sleep(2)


#motor 2 is left
print("testing motor 2 forward")
dir2.on() #on/off changes direction
pwm2.value = speed
time.sleep(2)
pwm2.value = 00
time.sleep(2)

print("testing motor 2 reverse")
dir2.off() #on/off changes direction
pwm2.value = speed
time.sleep(2)
pwm2.value = 0
time.sleep(2)
