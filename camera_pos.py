from functions import Robot
import time

robot = Robot()
camera = robot.camera

print("Testing camera movement")

print("Centering camera")
camera.center()
time.sleep(2)

print("Looking left")
camera.look_left(0.02) 
time.sleep(2)

print("returning to center")
camera.center(0.02)
time.sleep(2)

print("Looking right")
camera.look_right(0.02)
time.sleep(2)

print("returning to center")
camera.center(0.02)
time.sleep(2)