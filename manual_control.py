import time
from functions import Robot
from controller_map import PS5Controller


robot = Robot()
controller = PS5Controller()

CAMERA_STEP = 8.0
ARM_STEP = 3.0
MID_STEP = 3.0

try:
    while True:
        # Drive
        left_motor, right_motor = controller.get_drive()
        robot.motors.set_tank(left_motor, right_motor)

        # Camera
        cam_x, cam_y = controller.get_camera()

        if cam_x != 0:
            robot.camera.step_pan(cam_x * CAMERA_STEP)

        if cam_y != 0:
            robot.camera.step_tilt(cam_y * CAMERA_STEP)

        # Arm with D-pad
        dpad_x, dpad_y = controller.get_dpad()

        # base servo
        if dpad_y == 1:
            robot.arm.step_up(ARM_STEP)
        elif dpad_y == -1:
            robot.arm.step_down(ARM_STEP)

        # middle servo
        if dpad_x == -1:
            robot.arm.mid_down_step(MID_STEP)
        elif dpad_x == 1:
            robot.arm.mid_up_step(MID_STEP)

        # Claw: closed only while R2 is held
        if controller.r2_pressed():
            if robot.arm.claw_pos != 110:
                robot.arm.close_claw()
        else:
            if robot.arm.claw_pos != 40:
                robot.arm.open_claw()

        # Arm rotate
        if controller.l1_pressed():
            robot.arm.rotate_left_step(ARM_STEP)

        if controller.r1_pressed():
            robot.arm.rotate_right_step(ARM_STEP)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping robot...")
    robot.motors.stop()
    controller.close()