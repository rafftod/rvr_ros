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

    BATTERY_MEASURE_TIMEOUT: int = 120
    # main loop callback interval (seconds)
    CALLBACK_INTERVAL_DURATION: float = 0.250

    def __init__(self) -> None:
        # init ROS node
        rospy.init_node("rvr_driving_test")
        # init robot API connection
        self.log("Starting RVR API...")
        self.rvr = SpheroRvrObserver()
        # sensor values
        self.battery_percentage: float = 0
        self.latest_instruction: int = 0
        self.speed_params: Dict[str, float] = {
            "left_velocity": -0.5,
            "right_velocity": 0.5,
        }
        self.setup_rvr()

    def setup_rvr(self) -> None:
        self.log("Waking up RVR...")
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()
        # create timer for driving callback
        self.timer = rospy.Timer(
            rospy.Duration(self.CALLBACK_INTERVAL_DURATION), self.test_callback
        )

    def battery_percentage_handler(self, bp: Dict[str, float]) -> None:
        self.battery_percentage = bp.get("percentage")

    def test_callback(self, timer):
        current_time = rospy.Time.now().secs
        if current_time > 0:
            if self.latest_instruction == 0:
                self.latest_instruction = current_time
            elif current_time - self.latest_instruction > 2:
                self.speed_params = {k: -s for k, s in self.speed_params.items()}
                self.latest_instruction = current_time
        self.log(self.speed_params)
        self.rvr.drive_tank_si_units(
            **self.speed_params, timeout=self.CALLBACK_INTERVAL_DURATION
        )


if __name__ == "__main__":
    try:
        driving_test = DrivingTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Keyboard interrupted.")
        # rospy.signal_shutdown()
        time.sleep(0.5)
        driving_test.rvr.close()
        exit()
