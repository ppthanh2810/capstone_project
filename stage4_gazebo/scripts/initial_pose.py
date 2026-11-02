#!/usr/bin/env python3
"""Gửi pose ban đầu một lần (thay thao tác 2D Pose Estimate trong RViz) — chỉ khi s2_slam init_pose:=true (chỉ sim).

Chờ AMCL `active` (/amcl/get_state) và có subscriber trên topic đích, rồi phát PoseWithCovarianceStamped
(frame map, covariance giống RViz) lên /initialpose_guess (csm:=true -> CSM tinh chỉnh) hoặc /initialpose.
World Gazebo = frame map nên vị trí spawn là pose đúng (sai số chỉ bằng sai số của map).

  initial_pose.py --x 5.6 --y 2.18 --yaw -3.136 [--topic /initialpose_guess]
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.parameter import Parameter


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--yaw', type=float, required=True)
    ap.add_argument('--topic', default='/initialpose_guess')
    ap.add_argument('--timeout', type=float, default=120.0, help='giây (wall) chờ AMCL active')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('initial_pose_sender', parameter_overrides=[Parameter('use_sim_time', value=True)])
    log = node.get_logger()
    pub = node.create_publisher(PoseWithCovarianceStamped, args.topic, 10)
    client = node.create_client(GetState, '/amcl/get_state')

    t0 = time.monotonic()
    active = False
    while rclpy.ok() and time.monotonic() - t0 < args.timeout:
        if client.wait_for_service(timeout_sec=1.0):
            fut = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            if fut.done() and fut.result() and fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE \
                    and pub.get_subscription_count() > 0 and node.get_clock().now().nanoseconds > 0:
                active = True
                break
        time.sleep(1.0)
    if not active:
        log.error(f'AMCL chưa active / chưa có subscriber {args.topic} sau {args.timeout:.0f} s: không gửi pose')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # đợi thêm cho scan / TF odom ổn định (CSM cần scan mới)
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25      # giống RViz 2D Pose Estimate
    msg.pose.covariance[35] = 0.06853891945200942
    pub.publish(msg)
    log.info(f'Đã gửi pose ban đầu ({args.x:+.3f}, {args.y:+.3f}, {math.degrees(args.yaw):+.1f}°) -> {args.topic}')
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
