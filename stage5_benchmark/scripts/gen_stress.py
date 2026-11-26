#!/usr/bin/env python3
"""Pilot bộ STRESS-TEST (Master Plan V4 §8.2, ĐỀ XUẤT) — chỉ khi benchmark chính chạm trần (sensitivity_study).

Họ scenario định nghĩa TRƯỚC khi chạy (luật hình học cố định, không lọc theo kết quả DWA/SAC), cùng luật khả thi /
vô nghiệm / khe / khoảng cách start của generator v3, mọi vật cản là vật cản MỚI (unknown, Δ ∉ M):
  X1 occluded   B1/B2 ngay sau góc rẽ 0.4–0.9 m (chỉ thấy khi còn ~1 m)
  X2 clutter    C2 8–12 vật cản mới, khe ≥ 0.60
  X3 gapcorner  C1 khe 0.60–0.66 ngay sau góc rẽ 0.6–1.2 m
  X4 lateblock  B3 tường chặn tuyến ngay sau góc rẽ 0.5–1.5 m (phải replan muộn)
  X5 slalom     C3 xen kẽ dày (bước 1.0–1.2 m, lệch 0.20–0.35 m)
Seed namespace riêng (STRESS_INDEX·10^7) -> không trùng dataset v3; kiểm tra hash scenario / layout với mọi split v3.

  gen_stress.py --out ~/capstone_ws/bench_data/stress_pilot --per-family 30 [--workers 10] [--data ~/capstone_ws/bench_data/v3]
"""
import argparse
import collections
import json
import os
import sys
import warnings
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings('ignore')

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import curriculum as CU  # noqa: E402
from stage5_benchmark import generator as GN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402

STRESS_INDEX = 30
FAMILIES = {
    'X1': {'types': ['B1', 'B2'], 'hint': 'hard', 'gen': {'b_corner_offset': (0.4, 0.9), 'require_corner': True}},
    'X2': {'types': ['C2'], 'hint': 'hard', 'gen': {'c2_count': [8, 12], 'force_unknown': True}},
    'X3': {'types': ['C1'], 'hint': 'hard', 'gen': {'c1_corner_offset': (0.6, 1.2), 'force_unknown': True}},
    'X4': {'types': ['B3'], 'hint': 'hard', 'gen': {'b3_corner_offset': (0.5, 1.5)}},
    'X5': {'types': ['C3'], 'hint': 'hard', 'gen': {'c3_spacing': [1.0, 1.2], 'c3_offset': [0.20, 0.35],
                                                     'force_unknown': True}},
}


def _job(a):
    fam, stype, seed, hint, gen, nav = a
    sc, reasons = GN.generate_one(stype, seed, gen, nav, hint=hint)
    if sc is not None:
        sc['family'] = fam
    return fam, sc, reasons


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--per-family', type=int, default=30)
    ap.add_argument('--workers', type=int, default=10)
    ap.add_argument('--data', default=os.path.expanduser('~/capstone_ws/bench_data/v3'))
    a = ap.parse_args()
    cfg = C.load()
    nav = GN.nav_params(cfg)
    splits = [os.path.join(a.data, d) for d in sorted(os.listdir(a.data)) if os.path.isdir(os.path.join(a.data, d))]
    res_hash, res_map, _ = CU.reserved_keys(splits)
    got = collections.defaultdict(list)
    stats = collections.Counter()
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for rnd in range(30):
            jobs = []
            for fi, (fam, spec) in enumerate(FAMILIES.items()):
                if len(got[fam]) >= a.per_family:
                    continue
                gen = dict(cfg['gen'], **spec['gen'])
                for q in range(2 * a.per_family):
                    k = rnd * 10 ** 4 + q
                    t = spec['types'][q % len(spec['types'])]
                    jobs.append((fam, t, STRESS_INDEX * 10 ** 7 + fi * 10 ** 5 + k, spec['hint'], gen, nav))
            if not jobs:
                break
            for fam, sc, reasons in ex.map(_job, jobs, chunksize=1):
                stats[f'{fam}:attempt'] += 1
                if sc is None:
                    continue
                if sc['hash'] in res_hash or IO.map_id_of(sc['map']) in res_map:
                    stats[f'{fam}:leak_rejected'] += 1
                    continue
                if len(got[fam]) >= a.per_family:
                    continue
                g = cfg['gen']
                sc['difficulty']['level'] = ('easy' if sc['difficulty']['score'] < g['easy_max'] else
                                             'medium' if sc['difficulty']['score'] < g['medium_max'] else 'hard')
                sc['split'] = 'stress_pilot'
                sc['id'] = f"stress_{fam}_{sc['type']}_{len(got[fam]):03d}"
                got[fam].append(sc)
            print(rnd, {f: len(v) for f, v in got.items()}, flush=True)
    allsc = [s for v in got.values() for s in v]
    summary = {'families': {f: {'n': len(v), 'levels': collections.Counter(s['difficulty']['level'] for s in v),
                                'types': collections.Counter(s['type'] for s in v),
                                'spec': FAMILIES[f]} for f, v in got.items()},
               'attempts': dict(stats)}
    IO.save_split(a.out, allsc, summary)
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
