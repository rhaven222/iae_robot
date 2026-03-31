from functions import Robot
import time

robot = Robot()
camera = robot.camera

print("Testing camera movement")

print("Centering camera")
camera.center()
time.sleep(2)
print("Looking left")
camera.look_left()
time.sleep(2)
print("Looking right")
camera.look_right()
time.sleep(2)
print("Looking Up")
camera.look_up()
time.sleep(2)
print("Looking Forward")
camera.look_forward()
time.sleep(2)