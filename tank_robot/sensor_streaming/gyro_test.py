import sys
import time
sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")
from common.functions import *
from common.sensors import Gyro


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
    turn_angle_gyro(robot, 90)

    drive_straight_gyro(robot, speed=0.3, duration=5)

    turn_angle_gyro(robot, -90)
    time.sleep(0.1)