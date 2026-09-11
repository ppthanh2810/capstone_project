#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class Directrion (Node):

    def __init__ (self):
        super().__init__("direction")
        self.motion_pub = self.create_publisher(Twist, "/PT/motion", 10)
        self.timers_ = self.create_timer(0.1, self.motion_callback)
        self.get_logger().info("directrion coming")

    def motion_callback (self):

        msg = Twist()
        direction = str(input("Direction (f/fl/fr/b/bl/br):"))

        flag = True
        while flag:

            if direction == "f":
                msg.linear.x = 0.015
                msg.angular.z = 0.0
                flag = False

            elif direction == "b":
                msg.linear.x = -0.015
                msg.angular.z = 0.0
                flag = False

            elif direction == "fr":
                msg.linear.x = 0.012
                msg.angular.z = np.pi/40
                flag = False

            elif direction == "br":
                msg.linear.x = -0.012
                msg.angular.z = -np.pi/40
                flag = False

            elif direction == "bl":
                msg.linear.x = -0.012
                msg.angular.z = np.pi/40
                flag = False

            elif direction == "fl":
                msg.linear.x = 0.012
                msg.angular.z = -np.pi/40
                flag = False
            else:
                direction = str(input("input Direction again (f/fl/fr/b/bl/br):"))

        self.motion_pub.publish(msg)


def main (args=None):
    rclpy.init()
    node = Directrion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()