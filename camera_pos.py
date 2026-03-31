from functions import Robot
import time

robot = Robot()
camera = robot.camera

print("Testing camera movement")


camera.set_tilt(150)
time.sleep(2)

camera.set_pan(30)#look right
time.sleep(2)