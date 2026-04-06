from Libraries.functions import Robot
import time

robot = Robot()
motors = robot.motors
arm = robot.arm
camera = robot.camera

arm.center()
camera.center()
time.sleep(2)

motors.move_forward(3, 0.5)

camera.look_left(0.02)
time.sleep(1)
camera.look_right(0.02)
time.sleep(1)
camera.center(0.02)

arm.raise_arm(0.02)
arm.open_claw(0.02)

camera.look_up(0.02)
time.sleep(1)

motors.turn_right(3, 0.3)  # 180

time.sleep(1)

arm.fold_arm(0.02)
arm.close_claw(0.02)

camera.center(0.04)

motors.move_forward(3, 0.5)