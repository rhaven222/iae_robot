from gpiozero import DistanceSensor
from time import sleep

# echo=24, trigger=25
sensor = DistanceSensor(echo=18, trigger=23)

while True:
    distance = sensor.distance * 100  # convert to cm
    print(f"Distance: {distance:.2f} cm")
    sleep(2)