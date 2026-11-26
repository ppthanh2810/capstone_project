#!/usr/bin/env python3
"""Công cụ tối ưu DWA CỐ ĐỊNH (M0, Master Plan V4 bước 8).

Tối ưu [wP, wG, wO, wH, wS, T_sim] (cùng miền action của SAC) trên tập train/development, CHỌN bằng validation, không
bao giờ đụng test. Thuật toán: random search (Latin hypercube) + successive halving (tăng số scenario cho ứng viên
tốt; không có bước tinh chỉnh cục bộ). Mục tiêu: SR cân bằng theo type − 0.1·STR + 5·q_time (V4: điểm thời gian
phân xử khi SR chạm trần), ràng buộc CR <= cr_max (vượt -> loại). Mọi ứng viên qua validateParams (cùng plugin).

  tune_baseline.py --train DIR --val DIR[,DIR2,...] --out OUT_DIR --budget 120 [--workers 10] [--include-current]
--val: split ĐẦU TIÊN dùng để chọn (objective); MỌI split trong --val phải có CR <= cr_max (V4), nếu không ứng viên
bị loại. Đầu ra: trials.jsonl (mọi ứng viên + metric), best.yaml (dwa_general_best, khối CustomDWB.*),
report.json (top-k trên validation, SR/CR/SSR/STR theo split / type / mức, chi phí: episode, giờ·lõi, wall).
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


TIME_WEIGHT = 5.0     # V4: điểm thời gian q (0..1) x 5 ~ tối đa 5 điểm SR; chỉ phân xử khi SR hòa (trần, §8.1)


def q_time(r, sc):
    """Điểm thời gian như sensitivity_study: success · T_ref / max(T_ref, t), T_ref = L_ref / 0.3."""
    if r['eval']['label'] != 'SUCCESS':
        return 0.0
    t_ref = max(1.0, sc['l_ref'] / 0.3)
    return t_ref / max(t_ref, r['eval']['time'])


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
    q = [q_time(r, by_id[r['scenario_id']]) for r in recs if r['feasibility'] == 'feasible']
    s['q_time'] = float(np.mean(q)) if q else 0.0
    s['objective'] = (bal - (1e3 if feasible_cr > cr_max else 0.0) - 0.1 * (s['STR'] or 0.0) +
                      TIME_WEIGHT * s['q_time'])
    return s


def lhs(n, d, rng):
    u = (rng.permuted(np.tile(np.arange(n), (d, 1)), axis=1).T + rng.random((n, d))) / n
    return 2 * u - 1


COST = {'episodes': 0, 'sim_s': 0.0, 'cpu_s': 0.0}


def evaluate(params, scen, cfg, seeds, workers, by_id, cr_max, keep_recs=False):
    from stage5_benchmark import fast2d
    why = fast2d.validate_dwb(dict(cfg['dwb'], **params))
    if why:
        return {'invalid': why, 'objective': -1e9}
    recs = RN.run_batch(scen, cfg, seeds=seeds, dwb_override=params, workers=workers)
    COST['episodes'] += len(recs)
    COST['sim_s'] += sum(r.get('sim_s', 0.0) for r in recs)
    COST['cpu_s'] += sum(r.get('wall_s', 0.0) for r in recs)
    m = score(recs, by_id, cr_max)
    if keep_recs:
        m['_recs'] = recs
    return m


def breakdown(recs, by_id):
    rows = RN.rows_for_summary(recs, by_id)
    out = {'all': E.summarize(rows)}
    for key in ('type', 'level'):
        g = collections.defaultdict(list)
        for r in rows:
            g[r[key]].append(r)
        out['by_' + key] = {k: E.summarize(v) for k, v in sorted(g.items(), key=lambda kv: str(kv[0]))}
    out['failures'] = collections.Counter(
        f"{by_id[r['scenario_id']]['type']}:{r['eval']['label']}:{r['eval']['sub_reason']}" for r in recs
        if r['eval']['label'] not in ('SUCCESS', 'SAFE_STOP')).most_common()
    return out


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
    t_start = time.time()
    val_dirs = a.val.split(',')
    if any(os.path.basename(os.path.normpath(d)).endswith('_test') for d in val_dirs + [a.train]):
        raise SystemExit('test split bị khóa: không dùng để tối ưu / chọn')
    os.makedirs(a.out, exist_ok=True)
    cfg = C.load()
    rng = np.random.default_rng(a.seed)
    train = IO.load_split(a.train)
    vals = {os.path.basename(os.path.normpath(d)): IO.load_split(d) for d in val_dirs}
    by_id = {s['id']: s for s in train + [x for v in vals.values() for x in v]}
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
        entry = {'cand': ci, 'params': cands[ci], 'splits': {}}
        for k, (name, scen) in enumerate(vals.items()):
            m = evaluate(cands[ci], scen, cfg, seeds, a.workers or None, by_id, a.cr_max, keep_recs=True)
            recs = m.pop('_recs', [])
            entry['splits'][name] = {'metrics': {kk: v for kk, v in m.items() if kk != 'counts'},
                                     'breakdown': breakdown(recs, by_id)}
            if k == 0:
                entry['val'] = m
        entry['cr_ok_all'] = all((sp['metrics'].get('CR') or 0.0) <= a.cr_max for sp in entry['splits'].values())
        if not entry['cr_ok_all']:
            entry['val']['objective'] -= 1e3
        report.append(entry)
        m = entry['val']
        print(f'VAL cand {ci}: obj {m["objective"]:.2f} SRb {m["SR_balanced"]:.1f} CR {m["CR"]} STR {m["STR"]} '
              f'CR_ok_all {entry["cr_ok_all"]} ' +
              ' '.join(f'{n}:SR {sp["metrics"].get("SR")} SSR {sp["metrics"].get("SSR")}'
                       for n, sp in entry['splits'].items()), flush=True)
    report.sort(key=lambda r: r['val']['objective'], reverse=True)
    cost = dict(COST, wall_s=time.time() - t_start, cpu_h=COST['cpu_s'] / 3600.0, sim_h=COST['sim_s'] / 3600.0,
                candidates=len(cands), rungs=a.rungs, workers=a.workers)
    json.dump({'report': report, 'cost': cost}, open(os.path.join(a.out, 'report.json'), 'w'), indent=1,
              default=float)
    print('COST', json.dumps(cost, default=float), flush=True)
    if report:
        best = report[0]['params']
        with open(os.path.join(a.out, 'best.yaml'), 'w') as f:
            f.write('# dwa_general_best (chọn bằng validation; KHÔNG dùng test). Dán vào controller_server.CustomDWB\n')
            for k, v in best.items():
                f.write(f'{k}: {v:.4f}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
