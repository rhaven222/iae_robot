#!/usr/bin/python3
# coding=utf-8

import sys
import time

sys.path.append("/home/megan/iae_robot/tank_robot/sensor_streaming")

from common.functions import *

robot = Robot()
motor_state = create_motor_state()

TURN_SPEED = 0.25
turn_times = []

try:
    print("Turn 90 calibration test")
    print("Place the robot on the floor with space around it.")
    print("Press ENTER to start turning.")
    print("Press ENTER again when the robot reaches 90 degrees.")

    while True:
        input("\nPress ENTER to start turn...")

        start_time = time.time()

        drive_robot(
            robot,
            motor_state,
            TURN_SPEED,
            -TURN_SPEED
        )

        input("Press ENTER when robot reaches 90 degrees...")

        stop_robot(robot, motor_state)

        end_time = time.time()

        turn_time = end_time - start_time
        turn_times.append(turn_time)

        avg_time = sum(turn_times) / len(turn_times)

        print(f"Last 90 degree turn time: {turn_time:.3f} seconds")
        print(f"Average 90 degree turn time: {avg_time:.3f} seconds")
        print(f"Tests completed: {len(turn_times)}")

except KeyboardInterrupt:
    stop_robot(robot, motor_state)
    print("\nStopped")
    if turn_times:
        avg_time = sum(turn_times) / len(turn_times)
        print(f"Final average 90 degree turn time: {avg_time:.3f} seconds")