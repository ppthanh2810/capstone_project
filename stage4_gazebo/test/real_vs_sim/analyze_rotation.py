import json, math, sys
TRACK = 0.34
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def unwrap(ys):
    out=[ys[0]]
    for y in ys[1:]: out.append(out[-1]+wrap(y-out[-1]))
    return out
def analyze(d):
    cmd = d['/cmd_vel_out']
    t_on = next(c[0] for c in cmd if abs(c[2])>1e-6 or abs(c[1])>1e-6)
    t_off = next(c[0] for c in cmd if c[0]>t_on and abs(c[2])<1e-6 and abs(c[1])<1e-6)
    wf = d['/wheel_feedback']
    t_rpm = next(w[0] for w in wf if w[0]>=t_on-0.05 and (abs(w[1])>=1 or abs(w[2])>=1))
    hold = [w for w in wf if t_on+1.0 < w[0] < t_off-0.05]
    rpmL = sum(w[1] for w in hold)/len(hold); rpmR = sum(w[2] for w in hold)/len(hold)
    t_stop = next(w[0] for w in wf if w[0]>t_off and abs(w[1])<1 and abs(w[2])<1)
    t_end = t_stop + 1.5
    def at(series, t, i): return min(series, key=lambda x: abs(x[0]-t))[i]
    w0 = min(wf, key=lambda x: abs(x[0]-(t_on-0.3))); w1 = min(wf, key=lambda x: abs(x[0]-t_end))
    wheel_yaw = ((w1[4]-w0[4])-(w1[3]-w0[3]))/TRACK
    wheel_yaw_off = None
    res = dict(delay=t_rpm-t_on, rpmL=rpmL, rpmR=rpmR, stop=t_stop-t_off, wheel_yaw=math.degrees(wheel_yaw))
    for k,topic,i in (('imu_yaw','/imu_odom',3),('ekf_yaw','/odometry/filtered',3),('gt_yaw','/sim/ground_truth/odom',3)):
        s = d.get(topic)
        if not s: continue
        s = [x for x in s if t_on-0.5 <= x[0] <= t_end]
        ys = unwrap([x[i] for x in s]); ts=[x[0] for x in s]
        y_on = ys[min(range(len(ts)), key=lambda j: abs(ts[j]-t_on))]
        y_off = ys[min(range(len(ts)), key=lambda j: abs(ts[j]-t_off))]
        res[k] = math.degrees(ys[-1]-y_on)
        res[k+'_after_stop'] = math.degrees(ys[-1]-y_off)
    o = [x for x in d['/odometry/filtered'] if t_on-0.5 <= x[0] <= t_end]
    res['ekf_xy_drift_mm'] = 1000*math.hypot(o[-1][1]-o[0][1], o[-1][2]-o[0][2])
    vz = [abs(x[5]) for x in d['/odometry/filtered'] if t_on+1.0 < x[0] < t_off]
    res['ekf_wz'] = sum(vz)/len(vz)
    return res
if __name__ == '__main__':
    r = analyze(json.load(open(sys.argv[1])))
    for k,v in r.items(): print(f"{k:22s} {v:9.3f}")
