#!/usr/bin/env python3
"""Kiểm thử Giai đoạn A trên Fast 2D (Master Plan v3 A1, A4–A7): cùng một tập scenario cố định, đổi MỘT nhóm cấu
hình mỗi lần, so SR/CR/STR, khoảng hở, số quỹ đạo hợp lệ, loại OFF_MAP/UNKNOWN, thời gian tính.

  a_stage_tests.py --scenarios DIR --out OUT.json --exp footprint,rotate,unknown,inflation,costmap [--seeds 0,1]
"""

import argparse
import collections
import json
import os
import sys
import time
import warnings

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import evaluator as E  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def experiments(which):
    X = {}
    if 'footprint' in which:      # A1: footprint clearing × độ phân giải local × padding
        X['footprint'] = [(f'clear{int(c)}_res{r:g}_pad{p:g}', {},
                           {'local_footprint_clearing': c, 'local_resolution': r, 'local_padding': p,
                            'local_inflation': 0.30 if p < 0.08 else 0.45})
                          for p in (0.05, 0.10) for r in (0.05, 0.025) for c in (1, 0)]
    if 'rotate' in which:         # A4
        X['rotate'] = [(f'rth_{a}', {'rotate_to_heading_angle': a}, {}) for a in (0.8, 1.6, 2.5)]
    if 'unknown' in which:        # A5
        X['unknown'] = [('U0_free', {}, {'local_track_unknown': 0}),
                        ('U1_track', {}, {'local_track_unknown': 1}),
                        ('U2_track_inf', {}, {'local_track_unknown': 1, 'inf_is_valid': 1})]
    if 'inflation' in which:      # A6
        X['inflation'] = [(f'infl{r:.2f}_wO{w:g}', {'obstacle_scale': w}, {'local_inflation': r})
                          for r in (0.30, 0.40, 0.50) for w in (0.0, 1.0, 4.0, 8.0)]
    if 'costmap' in which:        # A7
        X['costmap'] = [(f'{s:g}x{s:g}_T{t:g}', {'sim_time': t}, {'local_size': s})
                        for s, ts in ((2.0, (1.5, 2.0, 2.5, 3.0)), (4.0, (1.5, 2.0, 2.5, 3.0, 4.0, 5.0)))
                        for t in ts]
    if 'delay' in which:          # A2
        X['delay'] = [('delay0.00', {}, {'command_delay': 0.0}), ('delay0.15', {}, {'command_delay': 0.15}),
                      ('delay0.30', {}, {'command_delay': 0.30})]
    return X


def metrics(recs, by_id):
    rows = RN.rows_for_summary(recs, by_id)
    s = E.summarize(rows)
    feas = [r for r in recs if r['feasibility'] == 'feasible' and not r.get('infra_error')]
    clr = [r['min_clearance'] for r in feas if np.isfinite(r['min_clearance'])]
    cand = sum(r.get('candidates', 0) for r in feas) or 1
    rej = collections.Counter()
    for r in feas:
        rej.update(r.get('rejects', {}))
    succ = [r for r in feas if r['eval']['label'] == 'SUCCESS']
    s.update({
        'clearance_min': float(np.min(clr)) if clr else None,
        'clearance_p5': float(np.percentile(clr, 5)) if clr else None,
        'clearance_median': float(np.median(clr)) if clr else None,
        'time_success_mean': float(np.mean([r['eval']['time'] for r in succ])) if succ else None,
        'replans_mean': float(np.mean([r.get('replans', 0) for r in feas])) if feas else None,
        'valid_fraction': 1.0 - sum(rej.values()) / cand,
        'reject_fraction': {k: v / cand for k, v in rej.items()},
        'cycles_no_valid_fraction': sum(r.get('cycles_no_valid', 0) for r in feas) /
        max(1, sum(r.get('controller_cycles', 0) for r in feas)),
        'compute_ms_p99_max': float(max([r.get('compute_ms_p99', 0) for r in feas] or [0])),
        'compute_ms_mean': float(np.mean([r.get('compute_ms_mean', 0) for r in feas])) if feas else None,
        'failures': collections.Counter(f"{by_id[r['scenario_id']]['type']}:{r['eval']['label']}:"
                                        f"{r['eval']['sub_reason']}" for r in recs
                                        if r['eval']['label'] not in ('SUCCESS', 'SAFE_STOP')).most_common(10),
    })
    by_type = collections.defaultdict(list)
    for r in rows:
        by_type[r['type']].append(r)
    s['SR_by_type'] = {t: E.summarize(v)['SR'] for t, v in sorted(by_type.items())}
    s['CR_by_type'] = {t: E.summarize(v)['CR'] for t, v in sorted(by_type.items())}
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--exp', default='footprint,rotate,unknown,inflation,costmap')
    ap.add_argument('--seeds', default='0,1')
    ap.add_argument('--types', default='')
    ap.add_argument('--base-sim', default='{}', help='JSON ghi đè sim cho mọi cấu hình (vd. đã chốt)')
    ap.add_argument('--base-dwb', default='{}')
    ap.add_argument('--workers', type=int, default=0)
    a = ap.parse_args()
    pool = IO.load_split(a.scenarios)
    if a.types:
        pool = [s for s in pool if s['type'] in a.types.split(',')]
    by_id = {s['id']: s for s in pool}
    seeds = [int(s) for s in a.seeds.split(',')]
    base_sim, base_dwb = json.loads(a.base_sim), json.loads(a.base_dwb)
    result = {'scenarios': a.scenarios, 'n_scenarios': len(pool), 'seeds': seeds, 'base_sim': base_sim,
              'base_dwb': base_dwb, 'experiments': {}}
    if os.path.exists(a.out):
        result = json.load(open(a.out))
    for ename, configs in experiments(a.exp.split(',')).items():
        result['experiments'].setdefault(ename, {})
        for name, dwb_o, sim_o in configs:
            cfg = C.load()
            t0 = time.time()
            d = dict(base_dwb)
            d.update(dwb_o)
            s = dict(base_sim)
            s.update(sim_o)
            recs = RN.run_batch(pool, cfg, seeds=seeds, dwb_override=d, sim_override=s, workers=a.workers or None)
            m = metrics(recs, by_id)
            m['wall_s'] = time.time() - t0
            m['dwb_override'], m['sim_override'] = d, s
            result['experiments'][ename][name] = m
            print(f"[{ename}] {name}: SR {m['SR']:.1f} CR {m['CR']:.2f} STR {m['STR']:.1f} SSR {m['SSR']} "
                  f"clr_min {m['clearance_min']:.3f} p5 {m['clearance_p5']:.3f} valid {m['valid_fraction']:.2f} "
                  f"rej {{{', '.join(f'{k}:{v:.3f}' for k, v in m['reject_fraction'].items() if v)}}} "
                  f"p99 {m['compute_ms_p99_max']:.1f} ms  {m['failures'][:4]}", flush=True)
            with open(a.out, 'w') as f:
                json.dump(result, f, ensure_ascii=False, indent=1, default=float)
    return 0


if __name__ == '__main__':
    sys.exit(main())
