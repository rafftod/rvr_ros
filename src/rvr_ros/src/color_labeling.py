import rospy
from std_msgs.msg import ColorRGBA


class ColorLabeler:

    color_labels = {
        "red": [255, 0, 0],
        "green": [0, 255, 0],
        "yellow": [255, 255, 0],
        "gray": [127, 127, 127],
    }

    def __init__(self) -> None:
        self.rvr_color_sub = rospy.Subscriber(
            "/rvr/ground_color", ColorRGBA, self.rvr_color_callback, queue_size=10
        )
        self.labeled_color_pub = rospy.Publisher(
            "/rvr/fixed_color", ColorRGBA, queue_size=10, latch=True
        )

    def rvr_color_callback(self, msg: ColorRGBA) -> None:
        dist_arr: list = []
        for color in self.color_labels.values():
            dist_arr.append(
                (msg.r - color[0]) ** 2
                + (msg.g - color[1]) ** 2
                + (msg.b - color[2]) ** 2
            )
        best_dist = min(dist_arr)
        for i, d in enumerate(dist_arr):
            if d == best_dist:
                rospy.loginfo(f"Closest color is {list(self.color_labels.keys())[i]}")
                color_msg = ColorRGBA()
                color_msg.r = list(self.color_labels.values())[i][0]
                color_msg.g = list(self.color_labels.values())[i][1]
                color_msg.b = list(self.color_labels.values())[i][2]
                color_msg.a = 255
                self.labeled_color_pub.publish(color_msg)


def main():
    rospy.init_node("color_labeler_node")
    labeler = ColorLabeler()
    rospy.spin()


if __name__ == "__main__":
    main()
