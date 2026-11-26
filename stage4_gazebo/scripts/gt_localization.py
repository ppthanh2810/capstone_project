#!/usr/bin/env python3
"""Định vị GROUND TRUTH cho Gazebo Planner Benchmark (CHỈ SIM, Master Plan V4 G3) — thay AMCL.

Phát TF map -> odom = GT(map -> base_footprint) · EKF(odom -> base_footprint)^-1, để planner_server, costmap,
CustomDWB và goal_path_bridge thấy pose thật (như Fast 2D "định vị hoàn hảo"). Không đụng EKF / odom -> base_footprint
(vẫn từ EKF), nên chuỗi điều khiển (odometry, độ trễ, ramp) giữ nguyên; chỉ bỏ sai số AMCL.
World Gazebo env:=map = frame map (map_to_world.py) nên /sim/ground_truth/odom dùng thẳng.
Cũng phát /amcl_pose (= GT) để công cụ đo cũ đọc được sai số định vị = 0.

Dùng qua s2_slam / s3_navigation `localization_source:=ground_truth` (KHÔNG bao giờ trên xe thật).
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GtLocalization(Node):
    def __init__(self):
        super().__init__('gt_localization')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 20.0)
        # Đóng dấu tương lai như AMCL transform_tolerance (nhỏ hơn: GT không trễ) để TF không phải ngoại suy.
        self.declare_parameter('future_dating', 0.1)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.future = self.get_parameter('future_dating').value
        self.gt = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', latched)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 50)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.warned = False

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, yaw_of(q), m.header.stamp)

    def tick(self):
        if self.gt is None:
            return
        gx, gy, gyaw, stamp = self.gt
        try:
            # odom -> base CÙNG thời điểm với mẫu GT (nội suy trong buffer TF); chưa có thì lấy mới nhất
            try:
                t = self.buf.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time.from_msg(stamp))
            except Exception:  # noqa: BLE001
                t = self.buf.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            if not self.warned:
                self.get_logger().warn(f'chưa có TF {self.odom_frame}->{self.base_frame}: {e}')
                self.warned = True
            return
        ox, oy = t.transform.translation.x, t.transform.translation.y
        oyaw = yaw_of(t.transform.rotation)
        # map->odom = G · O^-1
        dyaw = gyaw - oyaw
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = gx - (c * ox - s * oy)
        ty = gy - (s * ox + c * oy)
        msg = TransformStamped()
        msg.header.stamp = (self.get_clock().now() + Duration(seconds=self.future)).to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x, msg.transform.translation.y = tx, ty
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(dyaw / 2), math.cos(dyaw / 2)
        self.br.sendTransform(msg)
        pm = PoseWithCovarianceStamped()
        pm.header.stamp = self.get_clock().now().to_msg()
        pm.header.frame_id = self.map_frame
        pm.pose.pose.position.x, pm.pose.pose.position.y = gx, gy
        pm.pose.pose.orientation.z, pm.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        self.pose_pub.publish(pm)


def main():
    rclpy.init()
    n = GtLocalization()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
