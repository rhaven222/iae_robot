import time
from gyro import Gyro
from functions import Robot

robot = Robot()
gyro = Gyro()
gyro.calibrate()

while True:
    data = gyro.get_sensor_data()

    print(
        f"Heading: {data['heading']:.2f} deg | "
        f"Gyro Z: {data['gyro']['z']:.2f} | "
        f"Accel X: {data['accel']['x']:.2f} "
        f"Y: {data['accel']['y']:.2f} "
        f"Z: {data['accel']['z']:.2f}"
    )
    robot.turn_angle_gyro(90)

    robot.drive_straight_gyro(speed=0.3, duration=5)

    robot.turn_angle_gyro(-90)
    time.sleep(0.1)