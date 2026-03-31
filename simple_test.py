from functions import Motors, Arm

motors = Motors()
arm = Arm()

motors.move_forward(2, 0.4)

arm.raise_arm(0.03)
arm.open_claw(0.03)

motors.turn_right(2, 0.4)

arm.lower_arm()

motors.move_forward(2, 0.3)