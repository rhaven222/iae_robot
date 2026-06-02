#!/usr/bin/python3
# coding=utf8

import sys
import time
from gpiozero import DistanceSensor
from mpu6050 import mpu6050


class Sonar:
    """
    Compatibility sonar class.

    This emulates the original I2C Sonar class, but uses gpiozero
    DistanceSensor for the HC-SR04.
    """

    __units = {"mm": 0, "cm": 1}

    def __init__(self, echo=24, trigger=23, max_distance=2, queue_len=5):
        self.sensor = DistanceSensor(
            echo=echo,
            trigger=trigger,
            max_distance=max_distance,
            queue_len=queue_len
        )

        self.Pixels = [0, 0]
        self.RGBMode = 0

    def __getattr__(self, attr):
        if attr in self.__units:
            return self.__units[attr]
        if attr == "Distance":
            return self.getDistance()
        raise AttributeError("Unknown attribute: %s" % attr)

    def getDistance(self):
        """
        Return distance in millimeters, matching the original script.

        Your streaming script does:
            dist_cm = sonar.getDistance() / 10.0

        So this returns mm.
        """
        try:
            dist_mm = self.sensor.distance * 1000
            return dist_mm
        except BaseException as e:
            print(e)
            return 99999

    # -------------------------
    # RGB compatibility stubs
    # -------------------------

    def setRGBMode(self, mode):
        self.RGBMode = mode

    def show(self):
        pass

    def numPixels(self):
        return 2

    def setPixelColor(self, index, rgb):
        if index != 0 and index != 1:
            return

        color = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
        self.Pixels[index] = color

    def getPixelColor(self, index):
        if index != 0 and index != 1:
            raise ValueError("Invalid pixel index", index)

        return (
            (self.Pixels[index] >> 16) & 0xFF,
            (self.Pixels[index] >> 8) & 0xFF,
            self.Pixels[index] & 0xFF
        )

    def setBreathCycle(self, index, rgb, cycle):
        pass

    def startSymphony(self):
        pass



class Gyro:
    def __init__(self, address=0x68):
        self.address = address
        self.sensor = mpu6050(self.address)

        self.gyro_z_offset = 0.0
        self.heading_deg = 0.0
        self.last_time = None

        print("MPU-6050 initialized")

    def get_gyro_data(self):
        return self.sensor.get_gyro_data()

    def get_accel_data(self):
        return self.sensor.get_accel_data()

    def get_sensor_data(self):
        return {
            "gyro": self.get_gyro_data(),
            "accel": self.get_accel_data(),
            "heading": self.get_heading()
        }

    def calibrate(self, samples=200):
        print("Calibrating gyro. Keep robot still...")

        total_z = 0.0

        for _ in range(samples):
            gyro = self.sensor.get_gyro_data()
            total_z += gyro["z"]
            time.sleep(0.01)

        self.gyro_z_offset = total_z / samples
        self.reset_heading()

        print(f"Gyro calibrated. Z offset: {self.gyro_z_offset:.3f}")

    def reset_heading(self):
        self.heading_deg = 0.0
        self.last_time = time.time()

    def update_heading(self):
        now = time.time()

        if self.last_time is None:
            self.last_time = now
            return self.heading_deg

        dt = now - self.last_time
        self.last_time = now

        gyro = self.sensor.get_gyro_data()
        gyro_z = gyro["z"] - self.gyro_z_offset

        self.heading_deg += gyro_z * dt

        return self.heading_deg

    def get_heading(self):
        return self.update_heading()




