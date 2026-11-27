#!/usr/bin/env python3
"""Đánh giá bộ tham số DWA CỐ ĐỊNH (M0) dưới Navigation V5 đã đóng băng — Master Plan v5 bước 8.

Mỗi episode ghi một dòng (jsonl.gz, không trace): nhãn evaluator, thời gian, quãng đường, hở GT min, replan (thời
điểm, nguyên nhân, trạng thái DWA), đoạn tương tác + trạng thái vượt (LAR, cùng định nghĩa replan_compare.py),
q_time, split, bộ tham số, seed. Tổng hợp bằng baseline_report.py.

  baseline_eval.py --data ~/capstone_ws/bench_data --splits v3/general_val,v3/general_s1_val,stress_pilot \
      --sets sets.json --names default,general_best --seeds 0,1,2 --out ev.jsonl.gz
sets.json: {tên: {path_distance_scale: .., ...}} (khóa thiếu = giá trị nav2_params.yaml). Test split bị khóa.
Kiểm tra cấu hình đóng băng (freeze_v5.py --check) trước khi chạy; --no-freeze-check để bỏ qua.
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings('ignore')

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from replan_compare import features  # noqa: E402
from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def q_time(label, t, l_ref):
    """Điểm thời gian (như sensitivity_study / tune_baseline): SUCCESS · T_ref / max(T_ref, t), T_ref = L_ref / 0.3."""
    if label != 'SUCCESS':
        return 0.0
    t_ref = max(1.0, l_ref / 0.3)
    return t_ref / max(t_ref, t)


def _job(args):
    sc, cfg, seed, dwb_o = args
    rec = RN.run_episode(sc, cfg, seed, dwb_o)
    f, _ = features(rec, sc)
    f['l_ref'] = sc.get('l_ref', 0.0)
    f['q'] = q_time(f['label'], f['time'], f['l_ref'])
    f['wall_s'] = rec.get('wall_s', 0.0)
    f['sim_s'] = rec.get('sim_s', 0.0)
    f['compute_ms_p99'] = rec.get('compute_ms_p99', 0.0)
    return f


def run(pool, cfg, seeds, dwb_o, workers):
    jobs = [(sc, cfg, s, dwb_o) for sc in pool for s in seeds]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(_job, jobs, chunksize=2))


def load_pool(data, split):
    if os.path.basename(os.path.normpath(split)).endswith('_test'):
        raise SystemExit(f'{split}: test split bị khóa (bước 8 không dùng Final Test)')
    return IO.load_split(os.path.join(data, split))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=os.path.expanduser('~/capstone_ws/bench_data'))
    ap.add_argument('--splits', required=True)
    ap.add_argument('--sets', default='', help='file JSON {tên: dwb override}')
    ap.add_argument('--names', default='default')
    ap.add_argument('--seeds', default='0')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-freeze-check', action='store_true')
    a = ap.parse_args()
    if not a.no_freeze_check:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'freeze_v5.py'), '--check'], capture_output=True,
                           text=True)
        print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-300:], flush=True)
        if r.returncode:
            raise SystemExit('cấu hình V5 khác bản đóng băng')
    sets = json.load(open(a.sets)) if a.sets else {}
    sets.setdefault('default', {})
    seeds = [int(s) for s in a.seeds.split(',')]
    workers = a.workers or max(1, (os.cpu_count() or 2) - 2)
    cfg = C.load()
    out = gzip.open(a.out, 'at')
    for sp in a.splits.split(','):
        pool = load_pool(a.data, sp)
        for name in a.names.split(','):
            t0 = time.time()
            rows = run(pool, cfg, seeds, sets[name], workers)
            for f in rows:
                f['split'] = os.path.basename(os.path.normpath(sp))
                f['set'] = name
                out.write(json.dumps(f, default=float) + '\n')
            out.flush()
            n = len(rows)
            fe = [r for r in rows if r['feas'] == 'feasible']
            sr = 100 * sum(r['label'] == 'SUCCESS' for r in fe) / max(1, len(fe))
            print(f'{sp} {name}: {n} ep, SR {sr:.1f}, {time.time() - t0:.0f} s wall, '
                  f'{sum(r["wall_s"] for r in rows) / 3600:.2f} core-h', flush=True)
    out.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
