from functions import Robot
import time

robot = Robot()
camera = robot.camera

print("Testing camera movement")

camera.center()
camera.look_left()
camera.look_right()
camera.look_up()
camera.look_forward()