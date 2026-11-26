#!/usr/bin/env python3
"""Q6 / replan muộn (Master Plan V4 §3.5): so sánh trigger replan trên CÙNG scenario (ghép cặp với V3).

Mỗi cấu hình = ghi đè goal_path_bridge.replan.* (ReplanSupervisor dùng chung bridge + Fast 2D). Báo cáo SR/CR/STR,
hở GT nhỏ nhất, thời gian (episode thành công ở CẢ HAI cấu hình), số replan, replan "thừa" (episode V3 thành công
không cần replan mà cấu hình mới vẫn replan), và bảng chuyển kết cục so với V3.

  replan_study.py --data ~/capstone_ws/bench_data/v3 --splits general_train,general_val --out res.json \
      --configs '{"V3": {}, "E1": {"early_valid_ratio": 0.1}}' [--seeds 0,1]
"""
import argparse
import collections
import json
import os
import sys
import warnings

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import evaluator as E  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def compact(r, sc):
    return {'id': r['scenario_id'], 'seed': r['seed'], 'type': sc['type'], 'level': sc['difficulty']['level'],
            'feas': sc['feasibility'], 'label': r['eval']['label'], 'sub': r['eval']['sub_reason'],
            'time': r['eval']['time'], 'clr': r['min_clearance'], 'replans': r.get('replans', 0),
            'orep': r.get('obstacle_replans', 0), 'arep': r.get('abort_replans', 0),
            'early': sum(1 for _, s in r.get('events', []) if 'forward-valid' in s or 'early_distance' in s),
            'path_length': r['eval']['path_length'], 'stag': r.get('max_goal_stagnation', 0.0),
            'term': r.get('terminal_state', '')[:80]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--splits', required=True)
    ap.add_argument('--configs', required=True, help='JSON {tên: {replan.*}}; tên đầu tiên = tham chiếu')
    ap.add_argument('--dwb', default='{}', help='JSON ghi đè CustomDWB.* chung cho mọi cấu hình')
    ap.add_argument('--seeds', default='0')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    configs = json.loads(a.configs)
    seeds = [int(s) for s in a.seeds.split(',')]
    pool = []
    for sp in a.splits.split(','):
        if sp.endswith('_test'):
            raise SystemExit('test split bị khóa')
        pool += IO.load_split(os.path.join(a.data, sp))
    by = {s['id']: s for s in pool}
    eps = {}
    for name, rp in configs.items():
        cfg = C.load(overrides={'replan': rp})
        recs = RN.run_batch(pool, cfg, seeds=seeds, dwb_override=json.loads(a.dwb), workers=a.workers or None)
        eps[name] = {(r['scenario_id'], r['seed']): compact(r, by[r['scenario_id']]) for r in recs}
        print(name, 'done', flush=True)
    ref = list(configs)[0]
    out = {'configs': configs, 'splits': a.splits, 'seeds': seeds, 'summary': {}, 'transitions': {}}
    for name, E_ in eps.items():
        rows = [{'label': e['label'], 'feasibility': e['feas'], 'type': e['type'], 'level': e['level']}
                for e in E_.values()]
        s = E.summarize(rows)
        by_t = collections.defaultdict(list)
        for e in E_.values():
            by_t[e['type']].append(e)
        s['by_type'] = {t: {'SR': 100 * np.mean([x['label'] == 'SUCCESS' for x in v]),
                            'STR': 100 * np.mean([x['label'] == 'STUCK' for x in v]),
                            'replans_mean': float(np.mean([x['replans'] for x in v]))}
                        for t, v in sorted(by_t.items()) if v[0]['feas'] == 'feasible'}
        clr = [e['clr'] for e in E_.values() if np.isfinite(e['clr'])]
        s['min_clearance'] = float(min(clr))
        s['clearance_p05'] = float(np.percentile(clr, 5))
        both = [k for k in E_ if E_[k]['label'] == 'SUCCESS' and eps[ref][k]['label'] == 'SUCCESS']
        s['paired_success_n'] = len(both)
        s['time_mean_paired'] = float(np.mean([E_[k]['time'] for k in both])) if both else None
        s['time_mean_paired_ref'] = float(np.mean([eps[ref][k]['time'] for k in both])) if both else None
        s['path_mean_paired'] = float(np.mean([E_[k]['path_length'] for k in both])) if both else None
        s['path_mean_paired_ref'] = float(np.mean([eps[ref][k]['path_length'] for k in both])) if both else None
        s['replans_total'] = int(sum(e['replans'] for e in E_.values()))
        s['early_triggers'] = int(sum(e['early'] for e in E_.values()))
        # replan "thừa": V3 thành công KHÔNG replan, cấu hình này replan
        s['unneeded_replan_eps'] = int(sum(1 for k, e in E_.items() if e['replans'] > 0 and
                                           eps[ref][k]['label'] == 'SUCCESS' and eps[ref][k]['replans'] == 0))
        n_types = ('N1', 'N2', 'N3')
        s['nominal_replan_eps'] = int(sum(1 for e in E_.values() if e['type'] in n_types and e['replans'] > 0))
        out['summary'][name] = s
        tr = collections.Counter((eps[ref][k]['label'], e['label']) for k, e in E_.items())
        out['transitions'][name] = {f'{x}->{y}': n for (x, y), n in sorted(tr.items()) if x != y}
        out['transitions'][name + '_detail'] = [
            {'id': k[0], 'seed': k[1], 'type': e['type'], 'ref': f"{eps[ref][k]['label']}:{eps[ref][k]['sub']}",
             'new': f"{e['label']}:{e['sub']}", 'replans': e['replans'], 'early': e['early']}
            for k, e in sorted(E_.items()) if e['label'] != eps[ref][k]['label']]
        fe = [e for e in E_.values() if e['feas'] == 'feasible' and e['label'] == 'SUCCESS']
        s['stagnation_success_max'] = float(max((e['stag'] for e in fe), default=0.0))
        s['stagnation_success_p99'] = float(np.percentile([e['stag'] for e in fe], 99)) if fe else 0.0
        un = [e for e in E_.values() if e['feas'] == 'unsolvable']
        s['unsolvable_detail'] = [{'id': e['id'], 'label': e['label'], 'time': e['time'], 'stag': e['stag'],
                                   'term': e['term']} for e in un]
        print(f"{name}: SR {s['SR']:.2f} SSR {s['SSR']} CR {s['CR']:.2f} STR {s['STR']:.2f} minclr {s['min_clearance']:.3f} "
              f"t {s['time_mean_paired']:.2f}/{s['time_mean_paired_ref']:.2f} replans {s['replans_total']} "
              f"early {s['early_triggers']} unneeded {s['unneeded_replan_eps']} nominal {s['nominal_replan_eps']} "
              f"{out['transitions'][name]}", flush=True)
    json.dump(out, open(a.out, 'w'), indent=1, default=float)
    return 0


if __name__ == '__main__':
    sys.exit(main())
