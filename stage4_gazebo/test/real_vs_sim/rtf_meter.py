"""Đo RTF = Δ sim time (/clock) / Δ wall time, cửa sổ 1 s, trong N giây wall. In JSON."""
import json, sys, time, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy
dur = float(sys.argv[1]); outp = sys.argv[2]
rclpy.init(); n = Node('rtf_meter'); last = [None]
n.create_subscription(Clock, '/clock', lambda m: last.__setitem__(0, m.clock.sec + m.clock.nanosec*1e-9),
                      QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
def cpu():
    f = open('/proc/stat').readline().split()[1:]; v = list(map(int, f)); return sum(v), v[3]+v[4]
while last[0] is None: rclpy.spin_once(n, timeout_sec=0.1)
samples = []; w0 = time.monotonic(); s0 = last[0]; c0 = cpu(); start = w0
while time.monotonic() - start < dur:
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end: rclpy.spin_once(n, timeout_sec=0.05)
    w1 = time.monotonic(); s1 = last[0]; c1 = cpu()
    busy = 1 - (c1[1]-c0[1])/max(1, c1[0]-c0[0])
    samples.append([w1-start, (s1-s0)/(w1-w0), busy]); w0, s0, c0 = w1, s1, c1
r = sorted(x[1] for x in samples)
res = dict(n=len(r), mean=sum(r)/len(r), min=r[0], p5=r[int(0.05*len(r))], frac_below_0_9=sum(x<0.9 for x in r)/len(r),
           cpu_mean=sum(x[2] for x in samples)/len(samples), samples=samples)
json.dump(res, open(outp,'w')); print({k:v for k,v in res.items() if k!='samples'})
