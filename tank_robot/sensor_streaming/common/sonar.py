#!/usr/bin/python3
# coding=utf8

import sys
import time
from gpiozero import DistanceSensor

if sys.version_info.major == 2:
    print("Please run this program with python3!")
    sys.exit(0)


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


if __name__ == "__main__":
    s = Sonar()

    while True:
        print(f"{s.getDistance() / 10.0:.2f} cm")
        time.sleep(0.2)