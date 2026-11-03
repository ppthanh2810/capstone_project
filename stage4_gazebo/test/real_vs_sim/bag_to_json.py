import sys, json, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
out = {k: [] for k in types}
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, get_message(types[topic])); t = t*1e-9
    if topic in ('/cmd_vel_out','/cmd_vel_nav'): out[topic].append([t, m.linear.x, m.angular.z])
    elif topic == '/wheel_feedback': out[topic].append([t]+list(m.data))
    elif topic in ('/odometry/filtered','/imu_odom'):
        p=m.pose.pose; out[topic].append([t,p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z])
    elif topic == '/amcl_pose':
        p=m.pose.pose; out[topic].append([t,p.position.x,p.position.y,yaw(p.orientation)])
    elif topic == '/CustomDWB/state':
        out[topic].append([t, {kv.key: kv.value for kv in m.values}])
json.dump(out, open(sys.argv[2],'w'))
