from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=24, trigger=23, max_distance=2, queue_len=5)

while True:
    print(f"{sensor.distance * 100:.2f} cm")
    sleep(0.2)