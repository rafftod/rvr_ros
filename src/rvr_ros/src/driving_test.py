#!/usr/bin/env python3


"""The purpose of this script is to make the robot
turn around, to test that UART can work properly for a longer
operating time when using the treads.

To stop the script, use Ctrl+Z.
"""

from datetime import datetime
import time
import rospy
import os
import sys


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver

rvr = SpheroRvrObserver()

params = {"left_velocity": -0.5, "right_velocity": 0.5}

# time between 2 battery percentage measurements in seconds
BATTERY_MEASURE_TIMEOUT = 120

rospy.init_node("rvr_driving_test")

# battery percentage holder
battery_percentage: int


def battery_percentage_handler(bp):
    global battery_percentage
    battery_percentage = bp


def test_loop():
    last_measure_time = datetime.now().timestamp()
    rvr.get_battery_percentage(handler=battery_percentage_handler)
    # wait for battery response
    time.sleep(1)
    print(f"Current battery level : {battery_percentage}")
    while not rospy.is_shutdown():
        try:
            rvr.drive_tank_si_units(**params)
            if datetime.now().timestamp() - last_measure_time > BATTERY_MEASURE_TIMEOUT:
                rvr.get_battery_percentage(handler=battery_percentage_handler)
                # wait for battery response
                time.sleep(1)
                print(f"Current battery level : {battery_percentage}")
                last_measure_time = datetime.now().timestamp()
            else:
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
