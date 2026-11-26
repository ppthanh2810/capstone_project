#!/usr/bin/env python3
"""So sánh Fast 2D với Gazebo trên CÙNG scenario × NHIỀU bộ tham số DWA (Master Plan v3 C2, gate G3).

  sim2gz_compare.py --manifest DIR --gazebo gz.jsonl --param-sets sets.json --out cmp.json [--seeds 0,1,2,3,4]
Chỉ số mỗi (scenario, bộ tham số): nhãn Gazebo vs phân bố nhãn Fast 2D (nhiều seed), thời gian tới đích, quãng đường,
khoảng hở nhỏ nhất, số replan, độ dài A* ban đầu. Tổng: tỷ lệ trùng nhãn, sai lệch tương đối thời gian/quãng đường,
tương quan thứ hạng giữa các bộ tham số (Fast 2D có xếp hạng bộ tham số giống Gazebo không).
"""

import argparse
import collections
import json
import sys
import warnings

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--gazebo', required=True)
    ap.add_argument('--param-sets', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seeds', default='0,1,2,3,4')
    a = ap.parse_args()
    cfg = C.load()
    sets = json.load(open(a.param_sets))
    pool = {s['id']: s for s in IO.load_split(a.manifest)}
    gz = [json.loads(line) for line in open(a.gazebo)]
    ids = sorted({g['scenario_id'] for g in gz})
    seeds = [int(s) for s in a.seeds.split(',')]
    rows = []
    for sname, params in sets.items():
        recs = RN.run_batch([pool[i] for i in ids], cfg, seeds=seeds, dwb_override=params)
        by = collections.defaultdict(list)
        for r in recs:
            by[r['scenario_id']].append(r)
        for sid in ids:
            g = [x for x in gz if x['scenario_id'] == sid and x.get('param_set') == sname]
            if not g:
                continue
            g = g[0]
            f = by[sid]
            flabels = collections.Counter(r['eval']['label'] for r in f)
            fmaj = flabels.most_common(1)[0][0]
            succ_t = [r['eval']['time'] for r in f if r['eval']['label'] == 'SUCCESS']
            def outcome(st):
                st = (st or 'none').split(':')[0].split(' ')[0]
                return 'COLLISION' if False else st
            g_out = 'COLLISION' if g['eval']['label'] == 'COLLISION' else outcome(g.get('terminal_state'))
            f_out = collections.Counter('COLLISION' if r['eval']['label'] == 'COLLISION' else
                                        outcome(r.get('terminal_state')) for r in f)
            row = {
                'scenario': sid, 'type': pool[sid]['type'], 'param_set': sname,
                'gazebo_label': g['eval']['label'], 'gazebo_sub': g['eval'].get('sub_reason', ''),
                'fast2d_labels': dict(flabels), 'fast2d_majority': fmaj,
                'label_match': g['eval']['label'] == fmaj,
                # kết cục của bridge (SUCCEEDED / FAILED / ABORTED / none) — không phụ thuộc sai số AMCL tại đích
                'gazebo_outcome': g_out, 'fast2d_outcomes': dict(f_out),
                'outcome_match': g_out == f_out.most_common(1)[0][0],
                'gazebo_loc_error': g.get('loc_error_at_terminal'),
                'label_in_fast2d': g['eval']['label'] in flabels,
                'gazebo_time': g['eval'].get('time'), 'fast2d_time_mean': float(np.mean(succ_t)) if succ_t else None,
                'gazebo_path': g['eval'].get('path_length'),
                'fast2d_path_mean': float(np.mean([r['eval']['path_length'] for r in f])),
                'gazebo_clearance': g.get('min_clearance'),
                'fast2d_clearance_mean': float(np.mean([r['min_clearance'] for r in f])),
                'gazebo_replans': g.get('replans'), 'fast2d_replans_mean': float(np.mean([r['replans'] for r in f])),
                'gazebo_astar0': g.get('initial_path_length'),
                'fast2d_astar0': float(np.mean([r.get('initial_path_length', 0) for r in f])),
            }
            rows.append(row)
            print(json.dumps(row, default=float), flush=True)
    both = [r for r in rows if r['gazebo_outcome'] == 'SUCCEEDED' and r['fast2d_time_mean']]
    rel_t = [abs(r['gazebo_time'] - r['fast2d_time_mean']) / r['gazebo_time'] for r in both]
    rel_p = [abs(r['gazebo_path'] - r['fast2d_path_mean']) / max(r['gazebo_path'], 1e-6) for r in both]
    rel_a = [abs(r['gazebo_astar0'] - r['fast2d_astar0']) / max(r['gazebo_astar0'], 1e-6) for r in rows
             if r['gazebo_astar0']]
    # thứ hạng bộ tham số theo thời gian trung bình trên các scenario thành công ở cả hai
    rank = {}
    for sim in ('gazebo', 'fast2d'):
        key = 'gazebo_time' if sim == 'gazebo' else 'fast2d_time_mean'
        rank[sim] = {s: float(np.mean([r[key] for r in both if r['param_set'] == s] or [np.nan])) for s in sets}
    summary = {'n': len(rows), 'label_match_rate': float(np.mean([r['label_match'] for r in rows])) if rows else None,
               'outcome_match_rate': float(np.mean([r['outcome_match'] for r in rows])) if rows else None,
               'label_in_fast2d_rate': float(np.mean([r['label_in_fast2d'] for r in rows])) if rows else None,
               'time_rel_err_median': float(np.median(rel_t)) if rel_t else None,
               'path_rel_err_median': float(np.median(rel_p)) if rel_p else None,
               'astar0_rel_err_median': float(np.median(rel_a)) if rel_a else None,
               'mean_time_by_param_set': rank}
    json.dump({'rows': rows, 'summary': summary}, open(a.out, 'w'), indent=1, default=float)
    print(json.dumps(summary, indent=1, default=float))
    return 0


if __name__ == '__main__':
    sys.exit(main())
