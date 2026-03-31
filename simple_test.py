from functions import Motors, Arm

motors = Motors()
arm = Arm()

motors.move_forward(4, 0.5)

arm.raise_arm(0.02)
arm.open_claw(0.02)

motors.turn_right(4, 0.4)

arm.lower_arm()

motors.move_forward(4, 0.5)