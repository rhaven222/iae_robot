from functions import Motors, Arm
import time

motors = Motors()
arm = Arm()

arm.center()
time.sleep(2)
motors.move_forward(3, 0.5)

arm.raise_arm(0.02)
arm.open_claw(0.02)

motors.turn_right(3, 0.4) #180

arm.fold_arm(0.02)
arm.close_claw(0.02)

motors.move_forward(3, 0.5)