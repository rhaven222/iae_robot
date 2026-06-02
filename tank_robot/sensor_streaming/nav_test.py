"""
nav_fusion_test.py

Uses your existing robot functions:
- sonar distance
- camera frame/navigation check
- gyro heading
- motor tank control

Robot behavior:
Drive forward while clear, use gyro to stay straight, stop near obstacles,
use camera to choose a turn direction, then continue.
"""

import time

from common.sensors import Sonar, Gyro
from common.functions import *


OBSTACLE_CM = 35
CAUTION_CM = 55

FORWARD_SPEED = 0.28
SLOW_SPEED = 0.16
TURN_ANGLE = 55


def main():
    robot = Robot()

    print("Starting sonar + camera + gyro navigation...")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            distance = Sonar.getDistance()

            if distance is None:
                print("No sonar reading")
                robot.motors.stop()
                time.sleep(0.1)
                continue

            if distance <= OBSTACLE_CM:
                robot.motors.stop()
                print(f"Obstacle detected: {distance:.1f} cm")

                direction = camera_open_direction()

                print(f"Camera chose: {direction}")

                if direction == "left":
                    turn_degrees_gyro(robot, angle=TURN_ANGLE, speed=0.25)
                else:
                    turn_degrees_gyro(robot, angle=-TURN_ANGLE, speed=0.25)

                time.sleep(0.2)

            elif distance <= CAUTION_CM:
                drive_straight_gyro(robot, speed=SLOW_SPEED, duration=0.1)

            else:
                drive_straight_gyro(robot, speed=FORWARD_SPEED, duration=0.1)

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("Stopping.")

    finally:
        robot.motors.stop()


if __name__ == "__main__":
    main()