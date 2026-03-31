from functions import Robot
import time

robot = Robot()
motors = robot.motors
arm = robot.arm

arm.center()
time.sleep(2)
motors.move_forward(3, 0.5)

arm.raise_arm(0.02)
arm.open_claw(0.02)

motors.turn_right(3, 0.3) #180

arm.fold_arm(0.02)
arm.close_claw(0.02)

motors.move_forward(3, 0.5)