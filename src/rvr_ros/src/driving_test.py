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
from typing import Dict
from driver_logger import DriverLogger

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver


class DrivingTest(DriverLogger):

    speed_params = {"left_velocity": -0.5, "right_velocity": 0.5}
    BATTERY_MEASURE_TIMEOUT = 120

    def __init__(self) -> None:
        # init ROS node
        rospy.init_node("rvr_driving_test")
        # init robot API connection
        self.log("Starting RVR API...")
        self.rvr = SpheroRvrObserver()
        # sensor values
        self.battery_percentage: float = 0
        self.setup_rvr()

    def setup_rvr(self) -> None:
        self.log("Waking up RVR...")
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()

    def battery_percentage_handler(self, bp: Dict[str, float]) -> None:
        self.battery_percentage = bp.get("percentage")

    def test_loop(self):
        last_measure_time = datetime.now().timestamp()
        self.rvr.get_battery_percentage(handler=self.battery_percentage_handler)
        # wait for battery response
        time.sleep(1)
        self.log(f"Current battery level : {self.battery_percentage}")
        while not rospy.is_shutdown():
            try:
                self.rvr.drive_tank_si_units(**self.speed_params)
                if (
                    datetime.now().timestamp() - last_measure_time
                    > self.BATTERY_MEASURE_TIMEOUT
                ):
                    self.rvr.get_battery_percentage(
                        handler=self.battery_percentage_handler
                    )
                    # wait for battery response
                    time.sleep(1)
                    self.log(f"Current battery level : {self.battery_percentage}")
                    last_measure_time = datetime.now().timestamp()
                else:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("Keyboard interrupted.")
                # rospy.signal_shutdown()
                time.sleep(0.5)
                self.rvr.close()
                exit()


if __name__ == "__main__":
    driving_test = DrivingTest()
    driving_test.test_loop()
