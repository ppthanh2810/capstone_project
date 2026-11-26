#!/usr/bin/env python3
"""Pilot (Master Plan v3 C1–C3): sinh một tập nhỏ mỗi type × hint, đo tỷ lệ sinh hợp lệ, phân bố độ khó, rồi chạy
Fast 2D (DWA hiện tại, nhiều seed) và chấm bằng Episode Evaluator. Không sinh dataset chính thức.

  ros2 run stage5_benchmark pilot.py --out ~/capstone_ws/bench_runs/pilot --per-type 30 --seeds 0,1,2 [--profile X]
  (--maps classroom,house: thêm pilot trên map mục tiêu; --no-run: chỉ sinh)
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
from stage5_benchmark import generator as GN  # noqa: E402
from stage5_benchmark import layouts as LY  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def gen_pool(types, per_type, gen, nav, seed0, base=None, map_name=None):
    pool, stats = [], {}
    for t in types:
        st = {'attempted': 0, 'accepted': 0, 'reasons': collections.Counter(), 'levels': collections.Counter()}
        for k in range(per_type):
            hint = ['easy', 'medium', 'hard'][k % 3]
            seed = seed0 + 7919 * (GN.TYPES + ['S1']).index(t) + k
            sc, reasons = GN.generate_one(t, seed, gen, nav, base=base, hint=hint, map_name=map_name)
            st['attempted'] += 1
            st['reasons'].update(reasons)
            if sc:
                st['accepted'] += 1
                st['levels'][sc['difficulty']['level']] += 1
                sc['id'] = f"{map_name or 'gen'}_{t}_{k:04d}"
                pool.append(sc)
        st['reasons'] = dict(st['reasons'].most_common(8))
        st['levels'] = dict(st['levels'])
        stats[t] = st
    return pool, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--per-type', type=int, default=30)
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--types', default=','.join(GN.TYPES + ['S1']))
    ap.add_argument('--maps', default='', help='classroom,house')
    ap.add_argument('--profile', default=None)
    ap.add_argument('--no-run', action='store_true')
    ap.add_argument('--seed0', type=int, default=900000)
    ap.add_argument('--load', default='', help='dùng lại tập đã sinh (thư mục chứa manifest.jsonl)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cfg = C.load(a.profile)
    gen, nav = cfg['gen'], GN.nav_params(cfg)
    t0 = time.time()
    if a.load:
        pool = IO.load_split(a.load)
        stats = json.load(open(os.path.join(a.load, 'summary.json'))).get('generation', {})
    else:
        types = a.types.split(',')
        pool, stats = gen_pool(types, a.per_type, gen, nav, a.seed0)
        for mname in [m for m in a.maps.split(',') if m]:
            M, roi, _ = LY.target_map(mname)
            p2, s2 = gen_pool([t for t in types if t != 'C4'], max(6, a.per_type // 3), gen, nav, a.seed0 + 50000,
                              base=(M, roi, {'family': mname}), map_name=mname)
            pool += p2
            stats.update({f'{mname}:{k}': v for k, v in s2.items()})
        IO.save_split(os.path.join(a.out, 'scenarios'), pool, {'generation': stats})
    gen_time = time.time() - t0
    print(f'generated {len(pool)} scenarios in {gen_time:.1f} s')
    for k, v in stats.items():
        print(f"  {k:16s} {v['accepted']}/{v['attempted']} levels {v['levels']} rejects {v['reasons']}")
    if a.no_run:
        return 0
    seeds = [int(s) for s in a.seeds.split(',')]
    t1 = time.time()
    recs = RN.run_batch(pool, cfg, seeds=seeds)
    run_time = time.time() - t1
    by_id = {sc['id']: sc for sc in pool}
    rows = RN.rows_for_summary(recs, by_id)
    summary = {'all': E.summarize(rows), 'by_type': {}, 'by_level': {}, 'by_map': {}}
    for key, fn in (('by_type', lambda r: r['type']), ('by_level', lambda r: r['level']),
                    ('by_map', lambda r: by_id[r['scenario_id']].get('map_name'))):
        groups = collections.defaultdict(list)
        for r in rows:
            groups[fn(r)].append(r)
        summary[key] = {k: E.summarize(v) for k, v in sorted(groups.items(), key=lambda kv: str(kv[0]))}
    fails = collections.Counter((by_id[r['scenario_id']]['type'], r['eval']['label'], r['eval']['sub_reason'])
                                for r in recs if r['eval']['label'] not in ('SUCCESS', 'SAFE_STOP'))
    summary['failures'] = [list(k) + [v] for k, v in fails.most_common()]
    summary['timing'] = {'generation_s': gen_time, 'run_wall_s': run_time, 'episodes': len(recs),
                         'sim_s_total': float(sum(r.get('sim_s', 0) for r in recs)),
                         'compute_ms_p99_max': float(max(r.get('compute_ms_p99', 0) for r in recs))}
    summary['profile'] = cfg['profile']
    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1, default=float)
    with open(os.path.join(a.out, 'episodes.jsonl'), 'w') as f:
        for r in recs:
            r = {k: v for k, v in r.items() if k not in ('trace', 'tb')}
            f.write(json.dumps(r, ensure_ascii=False, default=float) + '\n')
    print(json.dumps(summary['all'], default=float))
    for k, v in summary['by_type'].items():
        print(f"  {k}: SR {v['SR']} CR {v['CR']} SSR {v['SSR']} STR {v['STR']} n {v['n']} {v['counts']}")
    print('failures:', summary['failures'][:15])
    print('timing:', summary['timing'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
