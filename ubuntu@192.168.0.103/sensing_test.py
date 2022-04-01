#!/usr/bin/env python3


"""The purpose of this script is to make the robot
turn around, to test that UART can work properly for a longer
operating time when using the treads and sensors.

To stop the script, use Ctrl+Z.
"""

from datetime import datetime
import time
import rospy
import os
import sys
from typing import Dict, List, FrozenSet
from driver_logger import DriverLogger


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver
from sphero_sdk.common.rvr_streaming_services import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups


class SensingTest(DriverLogger):

    ### Loop settings

    # main loop callback interval (seconds)
    CALLBACK_INTERVAL_DURATION: float = 0.100
    # robot API sensor streaming interval (ms)
    SENSOR_STREAMING_INTERVAL: int = int(CALLBACK_INTERVAL_DURATION * 1000)
    ### Wheel settings

    # speed in m/s
    speed: float = 0.5

    ### LEDs settings

    ACTIVE_COLOR: Colors = Colors.red
    INACTIVE_COLOR: Colors = Colors.off
    BACK_COLOR: Colors = Colors.pink

    LEFT_LEDS: FrozenSet[RvrLedGroups] = frozenset(
        {
            RvrLedGroups.headlight_left,
            RvrLedGroups.battery_door_front,
            RvrLedGroups.battery_door_rear,
        }
    )

    RIGHT_LEDS: FrozenSet[RvrLedGroups] = frozenset(
        {
            RvrLedGroups.headlight_right,
            RvrLedGroups.power_button_front,
            RvrLedGroups.power_button_rear,
        }
    )
    BATTERY_MEASURE_TIMEOUT = 120

    def __init__(self) -> None:
        # init ROS node
        rospy.init_node("rvr_sensing_test")
        # init robot API connection
        self.log("Starting RVR API...")
        self.rvr = SpheroRvrObserver()
        # initial speed
        self.speed_params: Dict[str, float] = {
            "left_velocity": 0,
            "right_velocity": self.speed,
        }
        # initial LED settings
        self.led_settings: Dict[int, Colors] = {
            # left headlight
            RvrLedGroups.headlight_left: self.INACTIVE_COLOR,
            # right headlight
            RvrLedGroups.headlight_right: self.ACTIVE_COLOR,
            # left side LED, left half
            RvrLedGroups.battery_door_front: self.INACTIVE_COLOR,
            # left side LED, right half
            RvrLedGroups.battery_door_rear: self.INACTIVE_COLOR,
            # right side LED, left half
            RvrLedGroups.power_button_front: self.ACTIVE_COLOR,
            # right side LED, right half
            RvrLedGroups.power_button_rear: self.ACTIVE_COLOR,
            # back LED
            RvrLedGroups.brakelight_left: self.BACK_COLOR,
            RvrLedGroups.brakelight_right: self.BACK_COLOR,
        }
        # sensor values
        # battery
        self.battery_percentage: float = 0
        self.latest_instruction: int = 0
        # accelerometer
        self.accelerometer_reading: Dict[str, float] = {"X": 0, "Y": 0, "Z": 0}
        # ground color sensor
        self.ground_color: Dict[str, int] = {"R": 0, "G": 0, "B": 0}
        # gyroscope
        self.angular_velocity: Dict[str, float] = {"X": 0, "Y": 0, "Z": 0}
        # IMU
        self.imu_reading: Dict[str, float] = {"Pitch": 0, "Roll": 0, "Yaw": 0}
        # light sensor
        self.ambient_light: float = 0
        # locator
        self.location: Dict[str, float] = {"X": 0, "Y": 0}
        # quaternion
        self.quat_reading: Dict[str, float] = {"W": 0, "X": 0, "Y": 0, "Z": 0}
        # velocity
        self.velocity_reading: Dict[str, float] = {"X": 0, "Y": 0}
        self.setup_rvr()

    def setup_rvr(self) -> None:
        self.log("Waking up RVR...")
        self.rvr.wake()
        time.sleep(2)
        self.rvr.reset_yaw()
        self.rvr.led_control.turn_leds_off()
        self.enable_sensors()
        # create timer for driving callback
        self.timer = rospy.Timer(
            rospy.Duration(self.CALLBACK_INTERVAL_DURATION), self.test_callback
        )

    """ Robot Handlers """

    def battery_percentage_handler(self, bp: Dict[str, float]) -> None:
        self.battery_percentage = bp.get("percentage")

    def accelerometer_handler(self, data: Dict[str, float]) -> None:
        self.accelerometer_reading.update(data["Accelerometer"])

    def ground_sensor_handler(self, data) -> None:
        self.ground_color.update(data["ColorDetection"])

    def gyroscope_handler(self, data) -> None:
        self.angular_velocity.update(data["Gyroscope"])

    def imu_handler(self, data) -> None:
        self.imu_reading.update(data["IMU"])

    def light_handler(self, data) -> None:
        self.ambient_light = data["AmbientLight"]["Light"]

    def locator_handler(self, data) -> None:
        self.location.update(data["Locator"])

    def quaternion_handler(self, data) -> None:
        self.quat_reading.update(data["Quaternion"])

    def velocity_handler(self, data) -> None:
        self.velocity_reading.update(data["Velocity"])

    def enable_sensors(self) -> None:
        self.log("Enabling sensors...")
        self.rvr.enable_color_detection(is_enabled=True)
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.accelerometer,
            handler=self.accelerometer_handler,
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=self.ground_sensor_handler,
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.gyroscope, handler=self.gyroscope_handler
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu, handler=self.imu_handler
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.ambient_light, handler=self.light_handler
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.locator, handler=self.locator_handler
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.quaternion, handler=self.quaternion_handler
        )
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.velocity, handler=self.velocity_handler
        )
        self.rvr.sensor_control.start(interval=self.SENSOR_STREAMING_INTERVAL)

    def test_callback(self, timer):
        current_time = rospy.Time.now().secs
        # update wheel speed if needed
        if current_time > 0:
            if self.latest_instruction == 0:
                self.latest_instruction = current_time
            elif current_time - self.latest_instruction > 2:
                # change driving tread
                self.speed_params = {
                    k: abs(s - self.speed) for k, s in self.speed_params.items()
                }
                # update LEDs
                # left
                for led_id in self.LEFT_LEDS:
                    self.led_settings[led_id] = (
                        self.ACTIVE_COLOR
                        if self.speed_params["left_velocity"] > 0
                        else self.INACTIVE_COLOR
                    )
                # right
                for led_id in self.RIGHT_LEDS:
                    self.led_settings[led_id] = (
                        self.ACTIVE_COLOR
                        if self.speed_params["right_velocity"] > 0
                        else self.INACTIVE_COLOR
                    )
                self.latest_instruction = current_time
        self.log(self.speed_params)
        # send speeds to API
        self.rvr.drive_tank_si_units(
            **self.speed_params, timeout=self.CALLBACK_INTERVAL_DURATION
        )
        # update led values
        self.rvr.led_control.set_multiple_leds_with_enums(
            leds=list(self.led_settings.keys()), colors=list(self.led_settings.values())
        )


if __name__ == "__main__":
    try:
        sensing_test = SensingTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        time.sleep(0.5)
        sensing_test.rvr.led_control.turn_leds_off()
        sensing_test.rvr.close()
        exit()
