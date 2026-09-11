#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from da1.zlac8015d import Controller
from std_msgs.msg import Float64MultiArray

# import numpy as np

class Move (Node):
    def __init__ (self):
        super().__init__("move")

        self.motors = Controller (port='/dev/ttyUSB1')
        self.motors.disable_motor()
        self.motors.set_accel_time(500, 500)
        self.motors.set_decel_time(500, 500)

        self.motors.set_mode(3)
        self.motors.enable_motor()

        self.move_sub = self.create_subscription (Float64MultiArray, "/PT/move", self.move_sub_callback, 10)
        self.get_logger().info("move soon")

    def move_sub_callback (self, msg: Float64MultiArray):

        self.motors.set_rpm (int(msg.data[0]), int(msg.data[1]))
        
        # print(f"rpmL-rpmL ouput: {l_rpm: .4f}, {-r_rpm: .4f}")

def main (args=None):
    rclpy.init(args=args)
    node = Move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motors.disable_motor()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
