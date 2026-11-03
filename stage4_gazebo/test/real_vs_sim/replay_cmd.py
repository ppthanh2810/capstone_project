"""Phát lại chuỗi /cmd_vel_out của bag thật lên /cmd_vel_nav trong sim (theo sim time), ghi lại phản hồi."""
import json, math, sys, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
real = json.load(open(sys.argv[1])); outp = sys.argv[2]
cmds = real['/cmd_vel_out']; c0 = cmds[0][0]
seq = [(c[0]-c0, c[1], c[2]) for c in cmds]
rclpy.init()
n = Node('replay', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
out = {k: [] for k in ('/cmd_vel_out','/wheel_feedback','/odometry/filtered','/imu_odom','/wheel_odom','/sim/ground_truth/odom')}
now = lambda: n.get_clock().now().nanoseconds*1e-9
def odo(topic):
    def cb(m):
        p=m.pose.pose; out[topic].append([now(),p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z])
    return cb
for t in ('/odometry/filtered','/imu_odom','/wheel_odom','/sim/ground_truth/odom'): n.create_subscription(Odometry, t, odo(t), 100)
n.create_subscription(Float64MultiArray, '/wheel_feedback', lambda m: out['/wheel_feedback'].append([now()]+list(m.data)), 100)
n.create_subscription(Twist, '/cmd_vel_out', lambda m: out['/cmd_vel_out'].append([now(), m.linear.x, m.angular.z]), 100)
pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
while rclpy.ok() and (now() < 1.0 or not out['/odometry/filtered']): rclpy.spin_once(n, timeout_sec=0.1)
t_start = now() + 3.0; i = 0
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.005)
    t = now() - t_start
    while i < len(seq) and t >= seq[i][0]:
        m = Twist(); m.linear.x, m.angular.z = seq[i][1], seq[i][2]; pub.publish(m); i += 1
    if i >= len(seq) and t > seq[-1][0] + 4.0: break
json.dump(out, open(outp,'w')); print('done', {k: len(v) for k,v in out.items()})
