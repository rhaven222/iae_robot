import RPi.GPIO as GPIO
import time

TRIG = 25
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # ensure clean trigger
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # send 10us pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # wait for echo start
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # wait for echo end
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # speed of sound = 34300 cm/s
    distance = pulse_duration * 17150
    return round(distance, 2)

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist} cm")
        time.sleep(2)

except KeyboardInterrupt:
    print("Stopped")
    GPIO.cleanup()