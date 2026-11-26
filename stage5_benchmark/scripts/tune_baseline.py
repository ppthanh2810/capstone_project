#!/usr/bin/env python3
"""Công cụ tối ưu DWA CỐ ĐỊNH (M0, Master Plan v3 Giai đoạn D / bước 8) — CHUẨN BỊ, chưa chạy chính thức.

Tối ưu [wP, wG, wO, wH, wS, T_sim] (cùng miền action của SAC) trên tập train/development, CHỌN bằng validation, không
bao giờ đụng test. Thuật toán: random search (Latin hypercube) + successive halving (tăng số scenario cho ứng viên
tốt) + tinh chỉnh cục bộ quanh top-k. Mục tiêu: SR cân bằng theo type, ràng buộc CR <= cr_max (vượt -> loại), hòa
-> STR thấp rồi thời gian. Mọi ứng viên qua validateParams (cùng plugin).

  tune_baseline.py --train DIR --val DIR --out OUT_DIR --budget 120 [--workers 10] [--include-current]
Đầu ra: trials.jsonl (mọi ứng viên + metric), best.yaml (dwa_general_best, khối CustomDWB.* để dán vào nav2_params),
report.json (top-k trên validation).
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
from stage5_benchmark import env as ENV  # noqa: E402
from stage5_benchmark import evaluator as E  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def score(recs, by_id, cr_max):
    rows = RN.rows_for_summary(recs, by_id)
    s = E.summarize(rows)
    by_t = collections.defaultdict(list)
    for r in rows:
        if r['feasibility'] == 'feasible':
            by_t[r['type']].append(r['label'] == 'SUCCESS')
    bal = float(np.mean([np.mean(v) for v in by_t.values()])) * 100 if by_t else 0.0
    t_succ = [r['eval']['time'] for r in recs if r['eval']['label'] == 'SUCCESS']
    s['SR_balanced'] = bal
    s['time_mean'] = float(np.mean(t_succ)) if t_succ else None
    feasible_cr = s['CR'] if s['CR'] is not None else 0.0
    s['objective'] = bal - (1e3 if feasible_cr > cr_max else 0.0) - 0.1 * (s['STR'] or 0.0)
    return s


def lhs(n, d, rng):
    u = (rng.permuted(np.tile(np.arange(n), (d, 1)), axis=1).T + rng.random((n, d))) / n
    return 2 * u - 1


def evaluate(params, scen, cfg, seeds, workers, by_id, cr_max):
    from stage5_benchmark import fast2d
    why = fast2d.validate_dwb(dict(cfg['dwb'], **params))
    if why:
        return {'invalid': why, 'objective': -1e9}
    recs = RN.run_batch(scen, cfg, seeds=seeds, dwb_override=params, workers=workers)
    return score(recs, by_id, cr_max)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--train', required=True)
    ap.add_argument('--val', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--budget', type=int, default=120, help='số ứng viên ban đầu')
    ap.add_argument('--rungs', default='60,200,0', help='số scenario train mỗi vòng halving (0 = tất cả)')
    ap.add_argument('--keep', type=float, default=0.3)
    ap.add_argument('--seeds', default='0')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--cr-max', type=float, default=2.0)
    ap.add_argument('--topk-val', type=int, default=5)
    ap.add_argument('--include-current', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cfg = C.load()
    rng = np.random.default_rng(a.seed)
    train = IO.load_split(a.train)
    val = IO.load_split(a.val)
    by_id = {s['id']: s for s in train + val}
    seeds = [int(s) for s in a.seeds.split(',')]
    cands = [ENV.action_to_params(x) for x in lhs(a.budget, 6, rng)]
    if a.include_current:
        cands.insert(0, {k: float(cfg['dwb'][k]) for k in ENV.ACTION_KEYS})
    trials = open(os.path.join(a.out, 'trials.jsonl'), 'a')
    alive = list(range(len(cands)))
    hist = {}
    order = rng.permutation(len(train))
    for rung, n in enumerate(int(x) for x in a.rungs.split(',')):
        sub = [train[i] for i in (order[:n] if n else order)]
        res = []
        for ci in alive:
            t0 = time.time()
            m = evaluate(cands[ci], sub, cfg, seeds, a.workers or None, by_id, a.cr_max)
            m['wall_s'] = time.time() - t0
            hist[(ci, rung)] = m
            res.append((m['objective'], ci))
            trials.write(json.dumps({'rung': rung, 'n_scen': len(sub), 'cand': ci, 'params': cands[ci],
                                     'metrics': {k: v for k, v in m.items() if k != 'counts'}}, default=float) + '\n')
            trials.flush()
            print(f'rung {rung} cand {ci}: obj {m["objective"]:.2f} SRb {m.get("SR_balanced", 0):.1f} '
                  f'CR {m.get("CR")} STR {m.get("STR")} {cands[ci]}', flush=True)
        res.sort(reverse=True)
        alive = [ci for _, ci in res[:max(1, int(len(res) * a.keep))]] if n else [ci for _, ci in res]
    # chọn bằng validation (top-k của rung cuối)
    report = []
    for ci in alive[:a.topk_val]:
        m = evaluate(cands[ci], val, cfg, seeds, a.workers or None, by_id, a.cr_max)
        report.append({'cand': ci, 'params': cands[ci], 'val': m})
        print(f'VAL cand {ci}: obj {m["objective"]:.2f} SRb {m["SR_balanced"]:.1f} CR {m["CR"]} STR {m["STR"]}',
              flush=True)
    report.sort(key=lambda r: r['val']['objective'], reverse=True)
    json.dump(report, open(os.path.join(a.out, 'report.json'), 'w'), indent=1, default=float)
    if report:
        best = report[0]['params']
        with open(os.path.join(a.out, 'best.yaml'), 'w') as f:
            f.write('# dwa_general_best (chọn bằng validation; KHÔNG dùng test). Dán vào controller_server.CustomDWB\n')
            for k, v in best.items():
                f.write(f'{k}: {v:.4f}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
