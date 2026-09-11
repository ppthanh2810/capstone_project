#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np


class MtoM(Node):
    def __init__(self):
        super().__init__("m_to_m")

        self.R_wheel = 0.0535      # meter
        self.wheel_distance = 0.34 # meter

        # cmd_vel cache
        self.x = 0.0
        self.z = 0.0

        # percent
        self.percent_ = None
        self.stop_threshold = 50.0

        # pub msg
        self.msg_rpm = Float64MultiArray()
        self.msg_rpm.data = [0.0, 0.0]

        self.motion_sub = self.create_subscription(Twist, "/cmd_vel", self.motion_sub_callback, 10)
        self.percent_sub = self.create_subscription(Float32, "/PT/percent", self.percent_sub_callback, 10)
        self.move_pub = self.create_publisher(Float64MultiArray, "/PT/move", 10)

        # publish at 10 Hz
        self.timer = self.create_timer(0.1, self.move_pub_callback)

        self.get_logger().info("m_to_m started")

    def motion_sub_callback(self, msg: Twist):
        self.x = float(msg.linear.x)
        self.z = float(msg.angular.z)

    def percent_sub_callback(self, msg: Float32):
        self.percent_ = float(msg.data)

    def move_pub_callback(self):
        # percent default nếu chưa nhận
        percent = self.percent_ if self.percent_ is not None else 0.0

        # nếu vượt ngưỡng thì stop linear, vẫn giữ angular (nếu bạn muốn quay tại chỗ)
        if percent >= self.stop_threshold:
            x = 0.0
            z = self.z
        else:
            x = self.x
            z = self.z

        if x >= 0.0:
            l_rpm = (x - (self.wheel_distance / 2) * (z)) / self.R_wheel
            r_rpm = ((x + (self.wheel_distance / 2) * (z)) / self.R_wheel)
            l_rpm = l_rpm *(60/(2*np.pi))
            r_rpm = -r_rpm *(60/(2*np.pi))
        else:
            l_rpm = (x + (self.wheel_distance / 2) * (z)) / self.R_wheel
            r_rpm = ((x - (self.wheel_distance / 2) * (z)) / self.R_wheel)
            l_rpm = l_rpm *(60/(2*np.pi))
            r_rpm = -r_rpm *(60/(2*np.pi))

        # publish
        self.msg_rpm.data = [float(l_rpm), float(r_rpm)]
        self.move_pub.publish(self.msg_rpm)


def main(args=None):
    rclpy.init(args=args)
    node = MtoM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
