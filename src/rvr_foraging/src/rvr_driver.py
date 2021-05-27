#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, ColorRGBA, MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

import os
import sys
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import RvrStreamingServices
import time

rvr = SpheroRvrObserver()

# received data storage
led_colors = [[0, 0, 0]] * 5
led_matchings = (
    RvrLedGroups.headlight_left.value,
    RvrLedGroups.headlight_right.value,
    RvrLedGroups.battery_door_front.value,
    RvrLedGroups.battery_door_rear.value,
    RvrLedGroups.undercarriage_white.value,
)
velocity = [0.0, 0.0]
# odometry variables
x, y, theta = 0.0, 0.0, 0.0


# Utility functions
def clamp(speed: float):
    """
    Clamps the speed to [-1.555; 1.555]
    """
    return float(sorted((-1.555, speed, 1.555))[1])


# ROS handlers
def WheelsHandler(speeds: Float32MultiArray):
    velocity[0] = clamp(speeds.data[0])
    velocity[1] = clamp(speeds.data[1])


def LedHandler(led_color: ColorRGBA, led_number: int):
    led_colors[led_number] = [int(led_color.r), int(led_color.g), int(led_color.b)]


# could be improved with 1 subscriber/publisher and better data structure for leds
def LedHandler0(led_color: ColorRGBA):
    return LedHandler(led_color, 0)


def LedHandler1(led_color: ColorRGBA):
    return LedHandler(led_color, 1)


def LedHandler2(led_color: ColorRGBA):
    return LedHandler(led_color, 2)


def LedHandler3(led_color: ColorRGBA):
    return LedHandler(led_color, 3)


def LedHandler4(led_color: ColorRGBA):
    return LedHandler(led_color, 4)


led_handlers = [LedHandler0, LedHandler1, LedHandler2, LedHandler3, LedHandler4]

rospy.init_node("rvr_driver")

# setup publisher for color sensor
color_pub = rospy.Publisher("sensor_color", ColorRGBA, queue_size=10)
color_msg = ColorRGBA()

# setup publisher for odometry
odom_pub = rospy.Publisher("odom", Odometry(), queue_size=10)
odom_msg = Odometry()

# setup sub for treads
vel_sub = rospy.Subscriber(
    "wheels_velocity", Float32MultiArray, callback=WheelsHandler, queue_size=10
)
vel_msg = Float32MultiArray()
vel_msg.layout.dim.append(MultiArrayDimension())
vel_msg.layout.dim[0].size = 2
vel_msg.layout.dim[0].stride = 1
vel_msg.layout.dim[0].label = "wheel_vel"
vel_msg.data.clear()

# setup subs for LEDs
led_sub = [
    rospy.Subscriber(
        f"led_color_{i}", ColorRGBA, callback=led_handlers[i], queue_size=10
    )
    for i in range(5)
]
led_msg = [ColorRGBA() for _ in range(5)]


def color_detected_handler(color_detected_data):
    """
    Handle triggered by RVR API when detecting color
    """
    color_msg.r = color_detected_data["ColorDetection"]["R"]
    color_msg.g = color_detected_data["ColorDetection"]["G"]
    color_msg.b = color_detected_data["ColorDetection"]["B"]
    color_msg.a = 255
    color_pub.publish(color_msg)


def locator_handler(locator_data):
    global x, y
    x, y = locator_data["Locator"]["X"], locator_data["Locator"]["Y"]
    # we only publish in the IMU handler


def imu_handler(imu_data):
    global x, y, theta, odom_msg
    theta = imu_data["IMU"]["Yaw"]
    # create odometry message to publish
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "rvr/odom"
    odom_msg.child_frame_id = "rvr/base_link"
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation = odom_quat
    odom_pub.publish(odom_msg)


def main():
    while not rospy.is_shutdown():
        try:
            rvr.drive_tank_si_units(
                left_velocity=velocity[0], right_velocity=velocity[1]
            )
            for i in range(len(led_colors)):
                rvr.set_all_leds(
                    led_group=led_matchings[i],
                    led_brightness_values=led_colors[i],
                )
            time.sleep(0.1)
            # rospy.spin()
        except KeyboardInterrupt:
            print("Keyboard interrupted.")
            rvr.sensor_control.clear()
            # rospy.signal_shutdown()
            time.sleep(0.5)
            rvr.close()
            break


if __name__ == "__main__":

    rvr.wake()

    time.sleep(2)

    rvr.reset_yaw()

    rvr.enable_color_detection(is_enabled=True)

    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection, handler=color_detected_handler
    )
    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu, handler=imu_handler
    )
    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator, handler=locator_handler
    )
    rvr.sensor_control.start(interval=100)  # match argos refresh interval
    main()