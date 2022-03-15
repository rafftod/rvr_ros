#!/usr/bin/env python3


"""The purpose of this script is to make the robot
turn around, to test that UART can work properly for a longer
operating time when using the treads.
"""

import time
import rospy
import os
import sys


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver

rvr = SpheroRvrObserver()

params = {"left_velocity": -0.5, "right_velocity": 0.5}

rospy.init_node("rvr_driving_test")


def test_loop():
    while not rospy.is_shutdown():
        try:
            rvr.drive_tank_si_units(**params)
            time.sleep(1)
        except KeyboardInterrupt:
            print("Keyboard interrupted.")
            # rospy.signal_shutdown()
            time.sleep(0.5)
            rvr.close()
            exit()


def main():
    rvr.wake()

    time.sleep(2)

    rvr.reset_yaw()

    test_loop()


if __name__ == "__main__":
    main()
