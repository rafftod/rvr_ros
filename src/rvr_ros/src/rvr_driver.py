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

from std_msgs.msg import Float32MultiArray, ColorRGBA, MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Pose, Vector3
from sensor_msgs.msg import Imu, Illuminance
from rvr_ros.msg import Leds
import tf
import tf_conversions
from math import pi


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver
from sphero_sdk.common.rvr_streaming_services import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups


class RobotDriver(DriverLogger):

    ### Loop settings

    # main loop callback interval (seconds)
    CALLBACK_INTERVAL_DURATION: float = 0.100
    # robot API sensor streaming interval (ms)
    SENSOR_STREAMING_INTERVAL: int = int(CALLBACK_INTERVAL_DURATION * 1000)
    ### Wheel settings

    # speed in m/s
    speed: float = 0

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
            RvrLedGroups.headlight_right: self.INACTIVE_COLOR,
            # left side LED, left half
            RvrLedGroups.battery_door_front: self.INACTIVE_COLOR,
            # left side LED, right half
            RvrLedGroups.battery_door_rear: self.INACTIVE_COLOR,
            # right side LED, left half
            RvrLedGroups.power_button_front: self.INACTIVE_COLOR,
            # right side LED, right half
            RvrLedGroups.power_button_rear: self.INACTIVE_COLOR,
            # back LED
            RvrLedGroups.brakelight_left: self.INACTIVE_COLOR,
            RvrLedGroups.brakelight_right: self.INACTIVE_COLOR,
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
        # IMU angles in degrees
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
        self.create_ros_publishers()
        # create timer for driving callback
        # self.timer = rospy.Timer(
        #     rospy.Duration(self.CALLBACK_INTERVAL_DURATION), self.test_callback
        # )
        # create timer for sensor send callback
        # self.sensor_pub_timer = rospy.Timer(
        #     rospy.Duration(self.CALLBACK_INTERVAL_DURATION), self.sensor_pub_callback
        # )
        # create timer for driving callback
        self.timer = rospy.Timer(
            rospy.Duration(self.CALLBACK_INTERVAL_DURATION), self.driving_callback
        )

    def create_ros_subscribers(self) -> None:
        # wheels speed subscriber
        rospy.Subscriber(
            "/rvr/wheels_speed",
            Float32MultiArray,
            self.wheels_speed_callback,
            queue_size=10,
        )
        # rgb leds subscriber
        rospy.Subscriber(
            "/rvr/rgb_leds",
            Leds,
            self.rgb_leds_callback,
            queue_size=10,
        )

    def create_ros_publishers(self) -> None:
        # Ground color as RGB
        self.ground_color_pub = rospy.Publisher(
            "/rvr/ground_color", ColorRGBA, queue_size=10
        )
        # IMU message includes :
        # - imu orientation
        # - gyroscope velocities
        # - linear acceleration
        self.imu_pub = rospy.Publisher("/rvr/imu", Imu, queue_size=10)
        # Ambient light
        self.light_pub = rospy.Publisher(
            "/rvr/ambient_light", Illuminance, queue_size=10
        )
        # Odometry message includes :
        # - Pose : position (locator) and orientation (quaternion)
        # - Twist : angular (gyro) and linear (velocity) velocities
        self.odom_pub = rospy.Publisher("/rvr/odom", Odometry, queue_size=10)

    def wheels_speed_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback for wheels speed subscriber.
        """
        self.speed_params["left_velocity"] = msg.data[0]
        self.speed_params["right_velocity"] = msg.data[1]

    def rgb_leds_callback(self, msg: Leds) -> None:
        """
        Callback for rgb leds subscriber.
        """
        # left headlight
        self.led_settings[RvrLedGroups.headlight_left] = [
            msg.front_left_color.r,
            msg.front_left_color.g,
            msg.front_left_color.b,
        ]
        # right headlight
        self.led_settings[RvrLedGroups.headlight_right] = [
            msg.front_right_color.r,
            msg.front_right_color.g,
            msg.front_right_color.b,
        ]
        # left side LED, both battery doors
        self.led_settings[RvrLedGroups.battery_door_front] = [
            msg.left_color.r,
            msg.left_color.g,
            msg.left_color.b,
        ]
        self.led_settings[RvrLedGroups.battery_door_rear] = [
            msg.left_color.r,
            msg.left_color.g,
            msg.left_color.b,
        ]
        # right side LED, both power buttons
        self.led_settings[RvrLedGroups.power_button_front] = [
            msg.right_color.r,
            msg.right_color.g,
            msg.right_color.b,
        ]
        self.led_settings[RvrLedGroups.power_button_rear] = [
            msg.right_color.r,
            msg.right_color.g,
            msg.right_color.b,
        ]
        # back LED
        self.led_settings[RvrLedGroups.brakelight_left] = [
            msg.back_color.r,
            msg.back_color.g,
            msg.back_color.b,
        ]
        self.led_settings[RvrLedGroups.brakelight_right] = [
            msg.back_color.r,
            msg.back_color.g,
            msg.back_color.b,
        ]

    """ Robot Handlers """

    def battery_percentage_handler(self, bp: Dict[str, float]) -> None:
        self.battery_percentage = bp.get("percentage")

    def accelerometer_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.accelerometer_reading.update(
            (k, data["Accelerometer"][k])
            for k in self.accelerometer_reading.keys() & data["Accelerometer"].keys()
        )

    def ground_sensor_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.ground_color.update(
            (k, data["ColorDetection"][k])
            for k in self.ground_color.keys() & data["ColorDetection"].keys()
        )

    def gyroscope_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.angular_velocity.update(
            (k, data["Gyroscope"][k])
            for k in self.angular_velocity.keys() & data["Gyroscope"].keys()
        )

    def imu_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.imu_reading.update(
            (k, data["IMU"][k]) for k in self.imu_reading.keys() & data["IMU"].keys()
        )

    def light_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.ambient_light = data["AmbientLight"]["Light"]

    def locator_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.location.update(
            (k, data["Locator"][k])
            for k in self.location.keys() & data["Locator"].keys()
        )

    def quaternion_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.quat_reading.update(
            (k, data["Quaternion"][k])
            for k in self.quat_reading.keys() & data["Quaternion"].keys()
        )

    def velocity_handler(self, data: Dict[str, Dict[str, float]]) -> None:
        self.velocity_reading.update(
            (k, data["Velocity"][k])
            for k in self.velocity_reading.keys() & data["Velocity"].keys()
        )

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
        # send speeds to API
        self.rvr.drive_tank_si_units(
            **self.speed_params, timeout=self.CALLBACK_INTERVAL_DURATION
        )
        # update led values
        self.rvr.led_control.set_multiple_leds_with_enums(
            leds=list(self.led_settings.keys()), colors=list(self.led_settings.values())
        )

    def publish_color(self) -> None:
        """Sends the stored ground color as an RGBA ROS message."""
        color_msg = ColorRGBA()
        color_msg.r = self.ground_color["R"]
        color_msg.g = self.ground_color["G"]
        color_msg.b = self.ground_color["B"]
        color_msg.a = 255
        self.ground_color_pub.publish(color_msg)

    def publish_imu(self):
        """Sends the stored IMU data as an Imu ROS message.
        This message includes :
        - orientation (pitch, roll, yaw) from the IMU sensor
        - angular velocities (x, y, z) from the gyroscope
        - linear acceleration (x, y, z) from the accelerometer"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        # build quaterion by converting degrees to radians
        imu_msg.orientation = Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(
                self.imu_reading.get("Roll") * pi / 180,
                self.imu_reading.get("Pitch") * pi / 180,
                self.imu_reading.get("Yaw") * pi / 180,
            )
        )
        # convert degrees/sec to radians/sec
        imu_msg.angular_velocity = Vector3(
            **{k.lower(): v * pi / 180 for k, v in self.angular_velocity.items()}
        )
        # convert values from g to m/s^2
        imu_msg.linear_acceleration = Vector3(
            **{k.lower(): v * 9.81 for k, v in self.accelerometer_reading.items()}
        )
        self.imu_pub.publish(imu_msg)

    def publish_light(self):
        """Publishes the light illuminance in Lux."""
        light_msg = Illuminance()
        light_msg.header.stamp = rospy.Time.now()
        light_msg.illuminance = self.ambient_light
        self.light_pub.publish(light_msg)

    def publish_odom(self):
        """Publishes the odometry data as a ROS message.
        This message includes :
        - the current location of the robot in the map frame from the locator
        - the current orientation of the robot in the map frame from the quaternion
        - the current velocity of the robot in the map frame from the velocity sensor
        - the current angular velocity of the robot in the map frame from the gyroscope"""
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = self.location.get("X")
        odom_msg.pose.pose.position.y = self.location.get("Y")
        odom_msg.pose.pose.position.z = 0
        odom_quat = Quaternion(**{k.lower(): v for k, v in self.quat_reading.items()})
        odom_msg.pose.pose.orientation = odom_quat
        # convert degrees/sec to radians/sec
        odom_msg.twist.twist.angular = Vector3(
            **{k.lower(): v * pi / 180 for k, v in self.angular_velocity.items()}
        )
        odom_msg.twist.twist.linear = Vector3(
            **{k.lower(): v for k, v in self.velocity_reading.items()}
        )
        self.odom_pub.publish(odom_msg)

    def sensor_pub_callback(self, timer):
        self.publish_info()

    def publish_info(self):
        self.publish_color()
        self.publish_imu()
        self.publish_light()
        self.publish_odom()

    def driving_callback(self, timer):
        self.publish_info()
        self.apply_actuators()

    def apply_actuators(self):
        """Applies the stored actuator values to the robot."""
        self.rvr.led_control.set_multiple_leds_with_enums(
            leds=list(self.led_settings.keys()), colors=list(self.led_settings.values())
        )
        self.rvr.drive_tank_si_units(
            **self.speed_params, timeout=self.CALLBACK_INTERVAL_DURATION
        )


if __name__ == "__main__":
    try:
        sensing_test = RobotDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        time.sleep(0.5)
        sensing_test.rvr.led_control.turn_leds_off()
        sensing_test.rvr.close()
        exit()
