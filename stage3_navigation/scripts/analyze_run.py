#!/usr/bin/env python3
"""Phân tích một lần chạy do record_run.sh ghi -> <run_dir>/report.md (+ report.png nếu có matplotlib).

    python3 stage3_navigation/scripts/analyze_run.py ~/robot_runs/<run> [--cloud-hz 2]

Chỉ đọc bag (offline, không cần robot). Các mục:
  1. Tần số / khoảng trống của topic chính (cảm biến bị rớt?)
  2. Đích + trạng thái bridge (SUCCEEDED / CANCELED / ABORTED, lý do)
  3. Định vị (/localization/status: thời gian OK/WARN/ERROR, lý do, consistency thấp nhất)
  4. CustomDWB (pha, chu kỳ không có quỹ đạo tiến, loại bỏ, thời gian tính) + cảnh báo /rosout
  5. Chuyển động (quãng đường, v lệnh vs đo, trễ lệnh -> odom)
  6. Camera: tần số, số điểm, MẶT SÀN theo TF URDF (độ nghiêng/độ cao sai), nhiễu sàn theo cự ly,
     điểm sàn lọt vào dải vật cản (vật cản ma), ô chỉ camera thấy so với LiDAR
  7. FusionLayer status (ô xác nhận / vẽ, tf_drops, cloud_age)
"""
import argparse
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit('Cần source /opt/ros/humble/setup.bash (và install/setup.bash)')

CLOUD = '/camera/camera/depth/color/points'
BASE = 'base_footprint'
# Phải khớp config/camera_fusion.yaml
BAND = (0.05, 0.70)
RANGE = (0.30, 2.5)
SELF_R = 0.25


# ----------------------------------------------------------------------------- tiện ích
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def kvs(msg):
    return {kv.key: kv.value for kv in msg.values}


def fnum(s, default=float('nan')):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def open_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}


class StaticTf:
    """Cây TF tĩnh (/tf_static) -> ma trận 4x4 child->BASE."""

    def __init__(self):
        self.parent = {}

    def add(self, msg):
        for t in msg.transforms:
            T = np.eye(4)
            T[:3, :3] = quat_mat(t.transform.rotation)
            tr = t.transform.translation
            T[:3, 3] = [tr.x, tr.y, tr.z]
            self.parent[t.child_frame_id.lstrip('/')] = (t.header.frame_id.lstrip('/'), T)

    def to_base(self, frame):
        T = np.eye(4)
        f = frame.lstrip('/')
        for _ in range(20):
            if f == BASE:
                return T
            if f not in self.parent:
                return None
            p, Tp = self.parent[f]
            T = Tp @ T
            f = p
        return None


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in 'xyz'):
        return np.zeros((0, 3))
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * msg.point_step]
    raw = raw.reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for i, k in enumerate('xyz'):
        out[:, i] = raw[:, off[k]: off[k] + 4].copy().view('<f4').ravel()
    return out


def fit_plane(p, iters=4, thr=0.03):
    """z = a x + b y + c, lặp bỏ ngoại lai. Trả (a, b, c, residual, mask) hoặc None."""
    m = np.ones(len(p), bool)
    for _ in range(iters):
        if m.sum() < 50:
            return None
        A = np.c_[p[m, 0], p[m, 1], np.ones(m.sum())]
        coef, *_ = np.linalg.lstsq(A, p[m, 2], rcond=None)
        res = p[:, 2] - (coef[0] * p[:, 0] + coef[1] * p[:, 1] + coef[2])
        m = np.abs(res) < thr
    return coef[0], coef[1], coef[2], res, m


def rate_gap(ts):
    if len(ts) < 2:
        return float('nan'), float('nan')
    d = np.diff(ts)
    return (len(ts) - 1) / (ts[-1] - ts[0]), d.max()


# ----------------------------------------------------------------------------- đọc bag
def read(bag, cloud_hz):
    r, types = open_bag(bag)
    D = defaultdict(list)
    tf = StaticTf()
    clouds = []
    last_cloud_kept = -1e9
    cloud_times = []
    cloud_npts = []
    scans = []  # (t, ranges, angle_min, inc, frame)
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        D['_t'].append(t)
        if topic == CLOUD:
            cloud_times.append(t)
            if t - last_cloud_kept < 1.0 / cloud_hz:
                cloud_npts.append(None)
                continue
            m = deserialize_message(data, get_message(types[topic]))
            xyz = cloud_xyz(m)
            fin = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 0)
            cloud_npts.append((len(xyz), int(fin.sum())))
            clouds.append((t, m.header.frame_id, xyz[fin], m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
            last_cloud_kept = t
            continue
        if topic == '/tf':
            m = deserialize_message(data, get_message(types[topic]))
            for tr in m.transforms:
                if tr.header.frame_id.lstrip('/') == 'odom' and tr.child_frame_id.lstrip('/') == BASE:
                    D['_odom_base'].append((t, tr.transform.translation.x, tr.transform.translation.y,
                                            yaw_of(tr.transform.rotation)))
            D[topic].append((t, None))
            continue
        if topic in ('/local_costmap/costmap_updates', '/map',
                     '/CustomDWB/trajectories', '/particle_cloud'):
            D[topic].append((t, None))  # chỉ đếm tần số
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf_static':
            tf.add(m)
        elif topic == '/scan_filtered':
            scans.append((t, np.array(m.ranges, dtype=np.float32), m.angle_min, m.angle_increment,
                          m.header.frame_id))
            D[topic].append((t, None))
        else:
            D[topic].append((t, m))
    D['_cloud_times'] = cloud_times
    D['_cloud_npts'] = cloud_npts
    return D, tf, clouds, scans, types


# ----------------------------------------------------------------------------- các mục báo cáo
def sec_topics(D, scans, out):
    out.append('## 1. Tần số topic\n')
    out.append('| Topic | Hz · gap max (s) |\n|---|---|')
    want = ['/scan_filtered', CLOUD, '/odometry/filtered', '/wheel_feedback', '/imu_odom',
            '/amcl_pose', '/localization/status', '/CustomDWB/state', '/cmd_vel_out',
            '/local_costmap/costmap', '/local_costmap/fusion_layer/status', '/tf']
    warn = []
    for tp in want:
        if tp == CLOUD:
            ts = np.array(D['_cloud_times'])
        else:
            ts = np.array([x[0] for x in D.get(tp, [])])
        if len(ts) == 0:
            out.append(f'| `{tp}` | — (không có) |')
            continue
        hz, gap = rate_gap(ts)
        out.append(f'| `{tp}` | {hz:.1f} · {gap:.2f} |')
        if tp in ('/scan_filtered', CLOUD, '/odometry/filtered') and gap > 0.5:
            warn.append(f'`{tp}` có khoảng trống {gap:.2f} s (> 0.5 s: costmap/EKF có thể dừng)')
    out.append('')
    for w in warn:
        out.append(f'- ⚠ {w}')
    out.append('')


def sec_goals(D, t0, out):
    out.append('## 2. Đích và trạng thái bridge\n')
    for t, m in D.get('/goal_pose', []):
        p = m.pose.position
        out.append(f'- {t - t0:7.1f} s  goal ({p.x:.2f}, {p.y:.2f}) yaw {math.degrees(yaw_of(m.pose.orientation)):.0f}°')
    states = [(t, m.data) for t, m in D.get('/goal_path_bridge/follow_state', [])]
    for t, s in states:
        out.append(f'- {t - t0:7.1f} s  `{s}`')
    if not states and not D.get('/goal_pose'):
        out.append('- (không có goal/state)')
    res = Counter(s.split(':')[0].split(' (')[0] for _, s in states)
    if res:
        out.append('\nTổng: ' + ', '.join(f'{k} {v}' for k, v in sorted(res.items())))
    out.append('')


def sec_loc(D, t0, out):
    out.append('## 3. Định vị (`/localization/status`)\n')
    st = D.get('/localization/status', [])
    if not st:
        out.append('- không có dữ liệu (monitor:=false?)\n')
        return
    lvl = Counter()
    prev = None
    cons = []
    events = []
    for i, (t, m) in enumerate(st):
        dt = (st[i + 1][0] - t) if i + 1 < len(st) else 0.0
        name = {0: 'OK', 1: 'WARN', 2: 'ERROR'}.get(int.from_bytes(m.level, 'little') if isinstance(m.level, bytes) else int(m.level), 'STALE')
        lvl[name] += dt
        v = kvs(m)
        c = fnum(v.get('scan_consistency'))
        if math.isfinite(c):
            cons.append((t, c))
        if name != prev:
            events.append(f'- {t - t0:7.1f} s  {name}: {m.message} {v.get("reasons", "")}'
                          f' (consistency {c:.2f}, blocked {fnum(v.get("blocked_ratio")):.2f})')
            prev = name
    tot = sum(lvl.values()) or 1
    out.append('Thời gian: ' + ', '.join(f'{k} {100 * v / tot:.0f} %' for k, v in lvl.items()))
    if cons:
        tm, cm = min(cons, key=lambda x: x[1])
        out.append(f'\nconsistency thấp nhất {cm:.2f} lúc {tm - t0:.1f} s; trung vị {np.median([c for _, c in cons]):.2f}')
    out.append('\nChuyển trạng thái:')
    out.extend(events[:40])
    out.append('')


def sec_dwb(D, t0, out):
    out.append('## 4. CustomDWB + cảnh báo\n')
    st = D.get('/CustomDWB/state', [])
    if st:
        phase = Counter()
        nofwd = 0
        rej = Counter()
        cmax = 0.0
        blocked = []
        dev = []
        rot_entries = 0
        prev_ph = None
        for t, m in st:
            v = kvs(m)
            phase[v.get('phase', m.message or '?')] += 1
            if v.get('phase') == 'FOLLOW_PATH' and v.get('n_forward_valid') == '0':
                nofwd += 1
            for k in ('rej_collision', 'rej_unknown', 'rej_off_costmap', 'rej_braking'):
                rej[k] += int(fnum(v.get(k), 0))
            cmax = max(cmax, fnum(v.get('compute_ms'), 0))
            if v.get('phase') == 'BLOCKED' or 'BLOCKED' in (m.message or ''):
                blocked.append(t - t0)
            dev.append(fnum(v.get('deviation'), 0))
            if v.get('phase') == 'ROTATE_TO_PATH' and prev_ph != 'ROTATE_TO_PATH':
                rot_entries += 1
            prev_ph = v.get('phase')
        out.append('Pha (số bản tin 5 Hz): ' + ', '.join(f'{k} {n}' for k, n in phase.most_common()))
        out.append(f'\nFOLLOW_PATH mà 0 quỹ đạo tiến: {nofwd}; loại bỏ (tổng): '
                   + ', '.join(f'{k[4:]} {n}' for k, n in rej.items()) + f'; compute max {cmax:.1f} ms')
        out.append(f'\nLệch đường max {max(dev):.2f} m (giới hạn max_path_deviation 1.0); vào ROTATE_TO_PATH {rot_entries} lần')
        if rot_entries >= 3:
            out.append('\n- ⚠ nhiều lần ROTATE_TO_PATH (> 2.5 rad lệch hướng): robot quay ngược hướng đường — do né vật cản (thật/ma) '
                       'hoặc quay vượt vì motor trễ (mục 5)')
        if blocked:
            out.append(f'\nBLOCKED lúc: {", ".join(f"{x:.1f}" for x in blocked[:20])} s')
    else:
        out.append('- không có /CustomDWB/state')
    logs = Counter()
    first = {}
    for t, m in D.get('/rosout', []):
        if m.level >= 30:  # WARN+
            key = f'[{m.name}] {m.msg[:90]}'
            logs[key] += 1
            first.setdefault(key, t - t0)
    if logs:
        out.append('\nCảnh báo/lỗi /rosout (số lần · lần đầu):')
        for k, n in logs.most_common(25):
            out.append(f'- {n}× · {first[k]:.1f} s · {k}')
    out.append('')


def _xcorr_lag(tc, a, tb, b, t_lo, t_hi, max_lag=2.0):
    grid = np.arange(max(t_lo, tc[0], tb[0]), min(t_hi, tc[-1], tb[-1]), 0.02)
    if len(grid) < 200:
        return float('nan')
    x = np.interp(grid, tc, a)
    y = np.interp(grid, tb, b)
    x, y = x - x.mean(), y - y.mean()
    if np.std(x) < 1e-3 or np.std(y) < 1e-3:
        return float('nan')
    n = int(max_lag / 0.02)
    cc = [np.dot(x[:len(x) - k], y[k:]) for k in range(n)]
    return int(np.argmax(cc)) * 0.02


def sec_motion(D, t0, out, fig):
    out.append('## 5. Chuyển động và đáp ứng motor\n')
    od = D.get('/odometry/filtered', [])
    if len(od) < 2:
        out.append('- không có /odometry/filtered\n')
        return
    T = np.array([t for t, _ in od])
    XY = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in od])
    V = np.array([m.twist.twist.linear.x for _, m in od])
    W = np.array([m.twist.twist.angular.z for _, m in od])
    dist = np.linalg.norm(np.diff(XY, axis=0), axis=1).sum()
    yaw = np.unwrap([yaw_of(m.pose.pose.orientation) for _, m in od])
    turns = np.abs(np.diff(yaw)).sum() / (2 * math.pi)
    out.append(f'- quãng đường (odom) {dist:.2f} m; tổng góc quay {turns:.1f} vòng; '
               f'v max {V.max():.2f} m/s; |ω| max {np.abs(W).max():.2f} rad/s')
    if dist > 0.5 and turns / dist > 0.5:
        out.append('- ⚠ quay rất nhiều so với quãng đường (> 0.5 vòng/m): robot xoay vòng / lắc, xem mục 4 (ROTATE_TO_PATH) và 8')
    cmd = D.get('/cmd_vel_out', [])
    nav = D.get('/cmd_vel_nav', [])
    if len(cmd) > 10:
        Tc = np.array([t for t, _ in cmd])
        Vc = np.array([m.linear.x for _, m in cmd])
        Wc = np.array([m.angular.z for _, m in cmd])
        lo, hi = (nav[0][0], nav[-1][0]) if nav else (Tc[0], Tc[-1])
        rows = [('odom', T, V, W)]
        wf = D.get('/wheel_feedback', [])
        if len(wf) > 10:
            k = math.pi / 30 * 0.0535
            Tw = np.array([t for t, _ in wf])
            L = np.array([m.data[0] for _, m in wf]) * k
            R = np.array([m.data[1] for _, m in wf]) * k
            rows.insert(0, ('wheel_feedback', Tw, (L + R) / 2, (R - L) / 0.34))
            acc = []
            for x in (L, R):
                dv = np.convolve(np.diff(x) / np.maximum(np.diff(Tw), 1e-3), np.ones(5) / 5, 'same')
                acc.append((np.percentile(dv, 95), np.percentile(dv, 5)))
            out.append(f'- gia tốc bánh đo (p95 tăng / p5 giảm): trái {acc[0][0]:.2f}/{acc[0][1]:.2f}, '
                       f'phải {acc[1][0]:.2f}/{acc[1][1]:.2f} m/s² (CustomDWB giả định acc/decel_lim_x 0.5)')
        out.append('\n| Đáp ứng | trễ v · trễ ω (s) |\n|---|---|')
        worst = 0.0
        for name, tb, vb, wb in rows:
            lv = _xcorr_lag(Tc, Vc, tb, vb, lo, hi)
            lw = _xcorr_lag(Tc, Wc, tb, wb, lo, hi)
            worst = max(worst, np.nanmax([lv, lw, 0]))
            out.append(f'| `/cmd_vel_out` → {name} | {lv:.2f} · {lw:.2f} |')
        if worst > 0.4:
            out.append(f'\n- ⚠ **motor đáp ứng chậm** (trễ tương quan tới {worst:.2f} s, gồm cả dốc tăng tốc) so với '
                       '`control_latency` 0.2 s: DWB dự đoán sai -> vượt góc, lệch đường, quãng phanh dài hơn tính.')
        fig['v'] = (Tc - t0, Vc, T - t0, V)
    fig['xy'] = XY
    out.append('')


def sec_costmap(D, scans, tf, t0, out):
    out.append('## 8. Local costmap so với LiDAR\n')
    cm = [(t, m) for t, m in D.get('/local_costmap/costmap', []) if m is not None]
    if not cm:
        out.append('- không có /local_costmap/costmap\n')
        return
    lidar_T = tf.to_base(scans[0][4]) if scans else None
    tfo = D.get('_odom_base', [])
    to = np.array([x[0] for x in tfo]) if tfo else None
    st = np.array([s[0] for s in scans]) if scans else None
    rows = []
    for t, m in cm:
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        known = a >= 0
        leth = a >= 99
        frac = leth.sum() / max(known.sum(), 1)
        far = float('nan')
        if lidar_T is not None and to is not None and len(to):
            i = int(np.argmin(np.abs(to - t)))
            j = int(np.argmin(np.abs(st - t)))
            if abs(to[i] - t) < 0.3 and abs(st[j] - t) < 0.3:
                x, y, yw = tfo[i][1:]
                _, rr, amin, inc, _ = scans[j]
                ang = amin + inc * np.arange(len(rr))
                ok = np.isfinite(rr) & (rr > 0.05)
                L = np.c_[rr[ok] * np.cos(ang[ok]), rr[ok] * np.sin(ang[ok]), np.zeros(ok.sum())]
                L = L @ lidar_T[:3, :3].T + lidar_T[:3, 3]
                gx = x + L[:, 0] * math.cos(yw) - L[:, 1] * math.sin(yw)
                gy = y + L[:, 0] * math.sin(yw) + L[:, 1] * math.cos(yw)
                res = m.info.resolution
                ci = np.floor((gx - m.info.origin.position.x) / res).astype(int)
                cj = np.floor((gy - m.info.origin.position.y) / res).astype(int)
                near = np.zeros_like(leth)
                r = int(math.ceil(0.45 / res))  # inflation 0.30 + robot 0.20 - lề
                inb = (ci >= 0) & (ci < m.info.width) & (cj >= 0) & (cj < m.info.height)
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        if di * di + dj * dj <= r * r:
                            u, v = ci[inb] + di, cj[inb] + dj
                            k = (u >= 0) & (u < m.info.width) & (v >= 0) & (v < m.info.height)
                            near[v[k], u[k]] = True
                far = (leth & ~near).sum() / max(leth.sum(), 1)
        rows.append((t, frac, far))
    R = np.array(rows, float)
    out.append(f'- ô ≥ INSCRIBED (99): trung vị {100 * np.median(R[:, 1]):.0f} %, max {100 * R[:, 1].max():.0f} % diện tích costmap')
    fin = R[np.isfinite(R[:, 2]), 2]
    if len(fin):
        out.append(f'- trong đó KHÔNG gần điểm LiDAR nào (≥ 0.45 m): trung vị {100 * np.median(fin):.0f} %, max {100 * fin.max():.0f} %'
                   ' -> do camera (hoặc LiDAR cũ chưa xóa)')
        if np.median(fin) > 0.3:
            out.append('- ⚠ **phần lớn vật cản trong costmap không có LiDAR xác nhận** — với camera:=true nghi vật cản ma từ camera (mục 6)')
    worst = R[int(np.argmax(R[:, 1]))]
    out.append(f'- lúc nặng nhất {worst[0] - t0:.1f} s: {100 * worst[1]:.0f} % bị chiếm')
    out.append('')


def sec_camera(D, tf, clouds, scans, t0, out, fig):
    out.append('## 6. Camera (D435 depth cloud)\n')
    ct = D['_cloud_times']
    if not ct:
        out.append('- **không có cloud** (`--no-cloud`, camera:=false, hoặc driver không chạy)\n')
        return
    npts = [x for x in D['_cloud_npts'] if x]
    hz, gap = rate_gap(np.array(ct))
    out.append(f'- {len(ct)} frame, {hz:.1f} Hz, gap max {gap:.2f} s; phân tích {len(clouds)} frame (mẫu)')
    if npts:
        tot = np.array([a for a, _ in npts])
        fin = np.array([b for _, b in npts])
        out.append(f'- điểm/frame: trung vị {np.median(tot):.0f}, hợp lệ {100 * np.median(fin / np.maximum(tot, 1)):.0f} %'
                   f' (min {100 * (fin / np.maximum(tot, 1)).min():.0f} %: lỗ độ sâu/ánh sáng)')
    if not clouds:
        return
    T = tf.to_base(clouds[0][1])
    if T is None:
        out.append(f'- ⚠ không có TF tĩnh `{clouds[0][1]}` -> `{BASE}` trong bag: bỏ qua phân tích hình học\n')
        return
    lidar_T = tf.to_base(scans[0][4]) if scans else None
    scan_t = np.array([s[0] for s in scans]) if scans else None
    stamp_lag = [t - st for t, _, _, st in clouds]
    out.append(f'- trễ header.stamp -> lúc nhận: trung vị {np.median(stamp_lag) * 1000:.0f} ms, max {max(stamp_lag) * 1000:.0f} ms')

    od = D.get('/odometry/filtered', [])
    od_t = np.array([x[0] for x in od]) if od else None
    od_v = np.array([abs(m.twist.twist.linear.x) + abs(m.twist.twist.angular.z) for _, m in od]) if od else None
    planes = []
    still = []
    bins = np.array([0.3, 0.6, 0.9, 1.2, 1.5, 2.0, 2.5])
    res_by_bin = defaultdict(list)
    ghost_by_bin = defaultdict(lambda: [0, 0])
    cam_only_low = cam_only_high = cam_cells_total = 0
    band_pts = []
    for t, frame, xyz, _ in clouds:
        P = xyz @ T[:3, :3].T + T[:3, 3]
        rng = np.hypot(P[:, 0], P[:, 1])
        near_floor = (np.abs(P[:, 2]) < 0.12) & (rng > 0.3) & (rng < 2.5)
        fp = fit_plane(P[near_floor])
        if fp:
            a, b, c, res, m = fp
            # độ cao sàn ngay dưới camera (tách khỏi ảnh hưởng của pitch/roll quanh gốc base)
            c_cam = c + a * T[0, 3] + b * T[1, 3]
            planes.append((t, math.degrees(math.atan(a)), math.degrees(math.atan(b)), c_cam, m.sum()))
            if od_t is not None and len(od_t) and od_v[int(np.argmin(np.abs(od_t - t)))] < 0.01:
                still.append(planes[-1][1:4])
            fpts = P[near_floor][m]
            r_f = np.hypot(fpts[:, 0], fpts[:, 1])
            idx = np.digitize(r_f, bins) - 1
            for i in range(len(bins) - 1):
                sel = idx == i
                if sel.sum() > 20:
                    res_by_bin[i].append(np.std(res[m][sel]))
            # Điểm "sàn" (gần mặt phẳng fit, trong 0.15 m) nhưng z URDF >= BAND[0] -> fusion coi là vật cản
            close = np.abs(res) < 0.15
            fl = P[near_floor][close]
            r_fl = np.hypot(fl[:, 0], fl[:, 1])
            idx2 = np.digitize(r_fl, bins) - 1
            for i in range(len(bins) - 1):
                s = idx2 == i
                ghost_by_bin[i][0] += int((fl[s, 2] >= BAND[0]).sum())
                ghost_by_bin[i][1] += int(s.sum())
        inband = (P[:, 2] >= BAND[0]) & (P[:, 2] <= BAND[1]) & (rng >= RANGE[0]) & (rng <= RANGE[1]) & (rng > SELF_R)
        band_pts.append(int(inband.sum()))
        # Ô 0.05 m camera thấy trong dải vật cản vs điểm LiDAR gần nhất về thời gian
        if inband.sum() and scans and lidar_T is not None:
            j = int(np.argmin(np.abs(scan_t - t)))
            if abs(scan_t[j] - t) < 0.2:
                _, rr, amin, inc, _ = scans[j]
                ang = amin + inc * np.arange(len(rr))
                ok = np.isfinite(rr) & (rr > 0.05) & (rr < RANGE[1] + 0.5)
                L = np.c_[rr[ok] * np.cos(ang[ok]), rr[ok] * np.sin(ang[ok]), np.zeros(ok.sum())]
                L = L @ lidar_T[:3, :3].T + lidar_T[:3, 3]
                lid = set(map(tuple, np.floor(L[:, :2] / 0.05).astype(int)))
                lid_n = set((i + di, k + dk) for i, k in lid for di in (-2, -1, 0, 1, 2) for dk in (-2, -1, 0, 1, 2))
                cells = defaultdict(float)
                cnt = Counter()
                for (cx, cy), z in zip(map(tuple, np.floor(P[inband, :2] / 0.05).astype(int)), P[inband, 2]):
                    cells[(cx, cy)] = max(cells[(cx, cy)], z)
                    cnt[(cx, cy)] += 1
                for c, zmax in cells.items():
                    if cnt[c] < 3:
                        continue
                    cam_cells_total += 1
                    if c not in lid_n:
                        if zmax < 0.15:
                            cam_only_low += 1
                        else:
                            cam_only_high += 1

    if planes:
        pa = np.array([p[1:4] for p in planes])
        med = np.median(pa, axis=0)
        sd = pa.std(axis=0)
        out.append('\n**Mặt sàn đo được (khung `base_footprint` theo URDF; đúng = 0°, 0°, 0 m):**\n')
        out.append('| Đại lượng | trung vị · độ lệch chuẩn |\n|---|---|')
        out.append(f'| pitch (dốc theo x) | {med[0]:+.2f}° · {sd[0]:.2f}° |')
        out.append(f'| roll (dốc theo y) | {med[1]:+.2f}° · {sd[1]:.2f}° |')
        out.append(f'| độ cao sàn dưới camera | {med[2] * 100:+.1f} cm · {sd[2] * 100:.1f} cm |')
        if still:
            sa = np.array(still)
            sm = np.median(sa, axis=0)
            out.append(f'| khi robot đứng yên ({len(sa)} frame) | pitch {sm[0]:+.2f}°, roll {sm[1]:+.2f}°, z {sm[2] * 100:+.1f} cm |')
        out.append('')
        z2 = abs(med[0]) * math.pi / 180 * 2.0 + abs(med[2])
        if abs(med[0]) > 1.0 or abs(med[1]) > 1.0 or abs(med[2]) > 0.02:
            out.append(f'- ⚠ **camera lệch so với URDF**: ở 2 m sàn bị nâng/hạ ~{z2 * 100:.0f} cm '
                       f'(ngưỡng vật cản {BAND[0] * 100:.0f} cm). Hiệu chỉnh `camera_joint` rpy/z hoặc nâng `min_obstacle_height`.')
            out.append(f'  Gợi ý cộng thêm vào `camera_joint`: pitch {med[0]:+.2f}° ({math.radians(med[0]):+.4f} rad), '
                       f'roll {-med[1]:+.2f}° ({math.radians(-med[1]):+.4f} rad), z {-med[2] * 100:+.1f} cm '
                       '(ước lượng; kiểm tra lại trên RViz: sàn trong cloud phải nằm ở z = 0).')
        else:
            out.append('- Mặt sàn khớp URDF (|pitch|,|roll| ≤ 1°, |z| ≤ 2 cm).')
        if sd[0] > 0.5 or sd[2] > 0.01:
            out.append('- ⚠ mặt sàn dao động giữa các frame (rung khi chạy / sàn không phẳng / camera lỏng).')
        fig['plane'] = (np.array([p[0] for p in planes]) - t0, pa)
    else:
        out.append('- không fit được mặt sàn (camera không nhìn thấy sàn? kiểm tra hướng camera)')

    out.append('\n**Nhiễu sàn theo cự ly** (std mặt sàn · % điểm sàn có z ≥ 5 cm = nguy cơ vật cản ma):\n')
    out.append('| Cự ly | std · % ≥ 5 cm |\n|---|---|')
    rows = []
    for i in range(len(bins) - 1):
        g, n = ghost_by_bin[i]
        s = np.median(res_by_bin[i]) if res_by_bin[i] else float('nan')
        pct = 100 * g / n if n else float('nan')
        rows.append((bins[i], s, pct))
        out.append(f'| {bins[i]:.1f}–{bins[i + 1]:.1f} m | {s * 100:.1f} cm · {pct:.1f} % |')
    fig['noise'] = rows
    bad = [r for r in rows if r[2] > 1.0]
    if bad:
        out.append(f'\n- ⚠ từ {bad[0][0]:.1f} m sàn lọt vào dải vật cản > 1 % điểm -> FusionLayer có thể vẽ vật cản ma'
                   f' (xem mục 7 `drawn_cells` khi đường trống). Cân nhắc `max_range` nhỏ hơn hoặc `min_obstacle_height` lớn hơn.')
    out.append(f'\n- điểm trong dải vật cản/frame: trung vị {np.median(band_pts):.0f}, max {max(band_pts)}')
    if cam_cells_total:
        out.append(f'- ô 5 cm camera thấy (≥ 3 điểm): {cam_cells_total}; không có LiDAR trong 10 cm: '
                   f'**thấp < 15 cm {cam_only_low}** (nghi sàn/nhiễu, hoặc vật thấp thật), cao ≥ 15 cm {cam_only_high} '
                   '(vật dưới/ trên mặt quét LiDAR — bàn, chân ghế — hoặc nhiễu).')
    out.append('')


def sec_fusion(D, t0, out):
    out.append('## 7. FusionLayer status\n')
    st = D.get('/local_costmap/fusion_layer/status', [])
    if not st:
        out.append('- không có (camera_layer không phải fusion)\n')
        return
    keys = ('confirmed_cells', 'drawn_cells', 'pending_cells', 'lidar_suppressed_cells', 'cloud_age', 'proc_ms_max')
    vals = defaultdict(list)
    lvls = Counter()
    for t, m in st:
        v = kvs(m)
        for k in keys:
            vals[k].append(fnum(v.get(k)))
        lvls[m.message] += 1
    last = kvs(st[-1][1])
    for k in keys:
        a = np.array(vals[k], float)
        a = a[np.isfinite(a)]
        if len(a):
            out.append(f'- {k}: trung vị {np.median(a):.2f}, max {a.max():.2f}')
    out.append(f'- tf_drops {last.get("tf_drops")}, queue_drops {last.get("queue_drops")}, frames {last.get("frames")}')
    out.append('- message: ' + ', '.join(f'"{k}" {n}' for k, n in lvls.most_common(5)))
    out.append('')


def draw(fig, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    f, ax = plt.subplots(2, 2, figsize=(11, 8))
    if 'xy' in fig:
        ax[0, 0].plot(fig['xy'][:, 0], fig['xy'][:, 1])
        ax[0, 0].set_title('Quỹ đạo odom')
        ax[0, 0].axis('equal')
    if 'v' in fig:
        tc, vc, to, vo = fig['v']
        ax[0, 1].plot(tc, vc, label='cmd_vel_out')
        ax[0, 1].plot(to, vo, label='odom v')
        ax[0, 1].legend()
        ax[0, 1].set_title('v (m/s)')
    if 'plane' in fig:
        t, pa = fig['plane']
        ax[1, 0].plot(t, pa[:, 0], label='pitch °')
        ax[1, 0].plot(t, pa[:, 1], label='roll °')
        ax[1, 0].plot(t, pa[:, 2] * 100, label='z cm')
        ax[1, 0].legend()
        ax[1, 0].set_title('Mặt sàn camera theo thời gian')
    if 'noise' in fig:
        r = np.array(fig['noise'], float)
        ax[1, 1].bar(r[:, 0] + 0.15, r[:, 2], width=0.25)
        ax[1, 1].set_title('% điểm sàn ≥ 5 cm theo cự ly (m)')
    f.tight_layout()
    f.savefig(path, dpi=90)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--cloud-hz', type=float, default=2.0, help='số frame cloud/giây đem phân tích')
    a = ap.parse_args()
    run = Path(a.run_dir).expanduser()
    bag = run / 'bag' if (run / 'bag' / 'metadata.yaml').exists() else run
    D, tf, clouds, scans, _ = read(bag, a.cloud_hz)
    t0 = D['_t'][0] if D['_t'] else 0.0
    out = [f'# Báo cáo `{run.name}`\n', f'Thời lượng bag {D["_t"][-1] - t0:.1f} s.\n']
    fig = {}
    sec_topics(D, scans, out)
    sec_goals(D, t0, out)
    sec_loc(D, t0, out)
    sec_dwb(D, t0, out)
    sec_motion(D, t0, out, fig)
    sec_camera(D, tf, clouds, scans, t0, out, fig)
    sec_fusion(D, t0, out)
    sec_costmap(D, scans, tf, t0, out)
    if draw(fig, run / 'report.png'):
        out.append('![](report.png)')
    (run / 'report.md').write_text('\n'.join(out) + '\n')
    print('\n'.join(out))
    print(f'\n-> {run / "report.md"}')


if __name__ == '__main__':
    main()
