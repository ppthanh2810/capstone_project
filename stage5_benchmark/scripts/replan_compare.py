#!/usr/bin/env python3
"""So sánh cơ chế Replanning (Master Plan V5 §3.6): R0 V3 stall, R1 V4 early_distance, R2 Local Avoidance First,
R3 chỉ DWA (chẩn đoán, CHỈ mô phỏng) — CÙNG scenario, seed, bộ tham số DWA, cấu hình robot và evaluator.

Mỗi episode ghi thêm (tính trong tiến trình con từ trace, không gửi trace qua tiến trình):
  - đoạn tương tác: mỗi vật cản MỚI Δ chặn đường A* ban đầu (điểm đường cách Δ < r_block = inscribed 0.25 m)
    -> [s_in, s_out] dọc đường ban đầu; "vượt qua" khi hình chiếu robot lên đường ban đầu >= s_out + 0.3 m
    (hoặc episode SUCCESS); Local Avoidance = vượt qua mà KHÔNG có replan nào từ lúc tới s_in - 2.5 m (tầm LiDAR
    đánh dấu) tới lúc vượt qua.
  - mỗi lần replan: thời điểm, nguyên nhân (phân loại từ thông điệp ReplanSupervisor), trạng thái DWA tại lúc đó
    (fs_clearance, tỷ lệ cung tự do, n_forward/n_candidates, v, khoảng cách tới ô bị chặn, pha, hở GT).
  - --signals: chuỗi tín hiệu 5 Hz (để chọn ngưỡng R2 từ dữ liệu R3), lưu <out>_signals.npz.

  replan_compare.py --data ~/capstone_ws/bench_data/v3 --splits general_train --out res.jsonl \
      --mechs R0,R1,R2,R3 [--sets default,timid,...] [--seeds 0] [--signals] [--workers 10]
Test split bị khóa.
"""
import argparse
import json
import math
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402

# ---- cơ chế (ghi đè goal_path_bridge.replan.*). Giá trị R2 = cấu hình đã chọn (nav2_params.yaml). ----
MECHS = {
    'R0': {'trigger': 'stall', 'early_distance': 0.0, 'early_valid_ratio': 0.0, 'approach_speed': 0.0},
    'R1': {'trigger': 'stall', 'early_distance': 1.0, 'early_max_deviation': 0.15, 'early_valid_ratio': 0.0,
           'approach_speed': 0.0},
    # R2 = Local Avoidance First, ngưỡng chọn trên general_train (lưới 2026-11-27, xem ACCEPTANCE_2026-11-27):
    #   cung tự do < 0.15 liên tục 0.4 s, hoặc DWA BLOCKED / không còn quỹ đạo tiến; dự phòng không tiến hữu ích
    #   4 s (dịch chuyển 0.3 m cũng tính); R2s = R2 + tiếp cận chậm 0.15 m/s khi ô bị chặn còn <= 1.2 m.
    'R2': {'trigger': 'conditional', 'early_distance': 0.0, 'early_valid_ratio': 0.0, 'local_free_ratio_min': 0.15,
           'local_hold_time': 0.4, 'useful_displacement': 0.3, 'approach_speed': 0.0},
    'R2s': {'trigger': 'conditional', 'early_distance': 0.0, 'early_valid_ratio': 0.0, 'local_free_ratio_min': 0.15,
            'local_hold_time': 0.4, 'useful_displacement': 0.3, 'approach_speed': 0.15, 'approach_distance': 1.2},
    'R3': {'trigger': 'conditional', 'early_distance': 0.0, 'early_valid_ratio': 0.0, 'obstacle_replan': 0.0,
           'max_abort_replans': 0, 'approach_speed': 0.0},
}

# ---- bộ tham số DWA [wP, wG, wO, wH, wS, T_sim] (miền action SAC §9), cố định trước khi chạy ----
SETS = {
    'default': {},
    'timid': {'obstacle_scale': 8.0, 'speed_scale': 0.5, 'sim_time': 2.5},
    'bold': {'obstacle_scale': 1.0, 'speed_scale': 3.0, 'sim_time': 1.5},
    'pathy': {'path_distance_scale': 2.0, 'carrot_distance_scale': 1.0, 'heading_scale': 1.0},
    'carrot': {'path_distance_scale': 0.0, 'carrot_distance_scale': 4.0, 'heading_scale': 0.0},
    'long': {'sim_time': 3.0},
    'short': {'sim_time': 1.5},
    'wO0': {'obstacle_scale': 0.0},
}

R_BLOCK = 0.25          # m = inscribed local/global (robot_radius 0.20 + padding 0.05)
DETECT = 2.5            # m = obstacle_max_range
PASS_MARGIN = 0.3       # m sau s_out


def shape_dist(sh, x, y):
    if sh['type'] == 'circle':
        return math.hypot(x - sh['x'], y - sh['y']) - sh['r']
    c, s = math.cos(sh['yaw']), math.sin(sh['yaw'])
    dx, dy = x - sh['x'], y - sh['y']
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    qx, qy = abs(lx) - sh['sx'] / 2, abs(ly) - sh['sy'] / 2
    out = math.hypot(max(qx, 0.0), max(qy, 0.0))
    return out + min(max(qx, qy), 0.0)


def classify(text):
    t = text
    if 'early_distance' in t:
        return 'distance_1m'
    if 'forward-valid' in t:
        return 'valid_ratio'
    if 'LOCAL_BLOCKED (no useful progress)' in t:
        return 'no_useful_progress'
    if 'LOCAL_BLOCKED (' in t:
        return 'local_' + t.split('LOCAL_BLOCKED (')[1].split(')')[0]
    if 'no progress for' in t:
        return 'stall'
    if 'FollowPath aborted' in t:
        return 'abort'
    if t.startswith('retry'):
        return 'retry'
    return 'other'


def features(rec, sc):
    cols = rec.get('trace_columns') or []
    tr = np.asarray(rec.get('trace'))
    ix = {c: i for i, c in enumerate(cols)}
    out = {'id': rec['scenario_id'], 'seed': rec['seed'], 'type': sc['type'], 'level': sc['difficulty']['level'],
           'feas': sc['feasibility'], 'knowledge': sc.get('knowledge'), 'label': rec['eval']['label'],
           'sub': rec['eval']['sub_reason'], 'time': rec['eval']['time'], 'path_length': rec['eval']['path_length'],
           'clr': rec.get('min_clearance', math.inf), 'replans': rec.get('replans', 0),
           'orep': rec.get('obstacle_replans', 0), 'arep': rec.get('abort_replans', 0),
           'term': (rec.get('terminal_state') or '')[:100], 'l_init': rec.get('initial_path_length', 0.0),
           'traps': sum(1 for _, s in rec.get('events', []) if 'robot pose is in collision' in s),
           'infra': rec.get('infra_error', '')}
    if len(tr) == 0 or not cols:
        out['replan_events'] = []
        out['segments'] = []
        return out, None

    def row_at(t):
        k = int(np.argmin(np.abs(tr[:, 0] - t)))
        r = tr[k]
        return {'fs_clr': round(float(r[ix['fs_clearance']]), 3), 'fs_free': round(float(r[ix['fs_free_ratio']]), 3),
                'nf': int(r[ix['n_forward']]), 'nc': int(r[ix['n_candidates']]), 'v': round(float(r[ix['v']]), 3),
                'block_dist': round(float(r[ix['block_dist']]), 2), 'phase': int(r[ix['phase']]),
                'gt_clr': round(float(r[ix['clearance']]), 3), 'x': round(float(r[ix['x']]), 2),
                'y': round(float(r[ix['y']]), 2)}

    ev = []
    for t, s in rec.get('events', []):
        if s.startswith('REPLANNING: '):
            why = s[len('REPLANNING: '):]
            e = {'t': t, 'cause': classify(why), 'text': why[:140]}
            e.update(row_at(t))
            ev.append(e)
    out['replan_events'] = ev
    rt = [e['t'] for e in ev]

    # ---- đoạn tương tác trên đường A* ban đầu ----
    segs = []
    plans = rec.get('plans') or []
    obs = sc.get('obstacles') or []
    if plans and obs:
        P = np.asarray(plans[0][1])
        s_arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
        hit = []
        for sh in obs:
            d = np.array([shape_dist(sh, px, py) for px, py in P[::2]])
            idx = np.nonzero(d < R_BLOCK)[0] * 2
            if len(idx):
                hit.append([s_arc[idx[0]], s_arc[idx[-1]]])
        hit.sort()
        merged = []
        for a, b in hit:
            if merged and a <= merged[-1][1] + 0.5:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        if merged:
            # hình chiếu robot lên đường ban đầu (điểm gần nhất, không giảm)
            xy = tr[:, 1:3]
            step = max(1, len(P) // 400)
            Pd = P[::step]
            sd = s_arc[::step]
            d2 = (xy[:, None, 0] - Pd[None, :, 0]) ** 2 + (xy[:, None, 1] - Pd[None, :, 1]) ** 2
            s_rob = np.maximum.accumulate(sd[np.argmin(d2, axis=1)])
            t = tr[:, 0]
            for a, b in merged:
                k_in = np.nonzero(s_rob >= a - DETECT)[0]
                k_pass = np.nonzero(s_rob >= b + PASS_MARGIN)[0]
                t_in = float(t[k_in[0]]) if len(k_in) else None
                t_pass = float(t[k_pass[0]]) if len(k_pass) else None
                if t_pass is None and out['label'] == 'SUCCESS':
                    t_pass = float(out['time'])
                reached = t_in is not None
                rp = [x for x in rt if t_in is not None and x >= t_in - 1e-6 and
                      (t_pass is None or x <= t_pass + 1e-6)]
                status = ('not_reached' if not reached else
                          'local' if (t_pass is not None and not rp) else
                          'replan_pass' if t_pass is not None else 'failed')
                segs.append({'s_in': round(a, 2), 's_out': round(b, 2), 't_in': t_in, 't_pass': t_pass,
                             'replans': len(rp), 'status': status,
                             'causes': [e['cause'] for e in ev if e['t'] in rp]})
    out['segments'] = segs
    return out, tr


def _job(args):
    sc, cfg, seed, dwb_o, want_signals = args
    rec = RN.run_episode(sc, cfg, seed, dwb_o)
    f, tr = features(rec, sc)
    sig = None
    if want_signals and tr is not None and len(tr):
        cols = rec['trace_columns']
        keep = ['t', 'x', 'y', 'v', 'clearance', 'phase', 'n_forward', 'n_candidates', 'fs_clearance',
                'fs_free_ratio', 'sup_state', 'block_dist', 'deviation']
        ix = [cols.index(c) for c in keep]
        sig = tr[::2, ix].astype(np.float32)        # 5 Hz
    return f, sig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--splits', required=True)
    ap.add_argument('--mechs', default='R0,R1,R2,R3')
    ap.add_argument('--mech-json', default='{}', help='JSON {tên: {replan.*}} thêm / ghi đè cơ chế')
    ap.add_argument('--sets', default='default')
    ap.add_argument('--types', default='', help='lọc type (vd. B1,B2,B3,C1,C2,C4,S1)')
    ap.add_argument('--ids', default='', help='file JSON danh sách id')
    ap.add_argument('--seeds', default='0')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--signals', action='store_true')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    mechs = dict(MECHS)
    mechs.update(json.loads(a.mech_json))
    seeds = [int(s) for s in a.seeds.split(',')]
    pool = []
    for sp in a.splits.split(','):
        if sp.endswith('_test'):
            raise SystemExit('test split bị khóa')
        pool += IO.load_split(os.path.join(a.data, sp))
    if a.types:
        ts = set(a.types.split(','))
        pool = [s for s in pool if s['type'] in ts]
    if a.ids:
        keep = set(json.load(open(a.ids)))
        pool = [s for s in pool if s['id'] in keep]
    workers = a.workers or max(1, (os.cpu_count() or 2) - 2)
    out_f = open(a.out, 'w')
    sig_all = {}
    for set_name in a.sets.split(','):
        dwb_o = SETS[set_name] if set_name in SETS else json.loads(set_name)
        for m in a.mechs.split(','):
            cfg = C.load(overrides={'replan': mechs[m]})
            t0 = time.time()
            jobs = [(sc, cfg, s, dwb_o, a.signals) for sc in pool for s in seeds]
            n = 0
            with ProcessPoolExecutor(max_workers=workers) as ex:
                for f, sig in ex.map(_job, jobs, chunksize=2):
                    f['mech'] = m
                    f['set'] = set_name
                    out_f.write(json.dumps(f, default=float) + '\n')
                    n += 1
                    if sig is not None:
                        sig_all[f"{set_name}|{m}|{f['id']}|{f['seed']}"] = sig
            out_f.flush()
            print(f'{set_name} {m}: {n} episodes, {time.time() - t0:.0f} s', flush=True)
    out_f.close()
    if a.signals and sig_all:
        np.savez_compressed(os.path.splitext(a.out)[0] + '_signals.npz', **sig_all)
    return 0


if __name__ == '__main__':
    sys.exit(main())
