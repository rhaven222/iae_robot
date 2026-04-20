from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=4, trigger=23, max_distance=2)

while True:
    print("meters:", sensor.distance)
    print("cm:", sensor.distance * 100)
    sleep(1)