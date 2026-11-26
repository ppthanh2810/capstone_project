#!/usr/bin/env python3
"""Chạy một cấu hình DWA cố định trên các split (Fast 2D) và tổng hợp SR/CR/SSR/STR theo type / mức / map.
Dùng cho tham chiếu M0 trên train/validation. KHÔNG chạy trên *_test trước khi đóng băng (tham số --allow-test).

  eval_split.py --data ~/capstone_ws/bench_data/v3 --splits general_val,general_s1_val --out res.json [--dwb '{}']
"""
import argparse, collections, json, os, sys, warnings
warnings.filterwarnings('ignore')
from stage5_benchmark import config as C, evaluator as E, runner as RN, scenario_io as IO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--splits', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--dwb', default='{}')
    ap.add_argument('--seeds', default='0')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--allow-test', action='store_true')
    a = ap.parse_args()
    cfg = C.load()
    out = {'dwb_override': json.loads(a.dwb), 'splits': {}}
    for sp in a.splits.split(','):
        if sp.endswith('_test') and not a.allow_test:
            print(f'skip {sp}: test split bị khóa (--allow-test)')
            continue
        pool = IO.load_split(os.path.join(a.data, sp))
        by = {s['id']: s for s in pool}
        recs = RN.run_batch(pool, cfg, seeds=[int(x) for x in a.seeds.split(',')], dwb_override=json.loads(a.dwb),
                            workers=a.workers or None)
        rows = RN.rows_for_summary(recs, by)
        res = {'all': E.summarize(rows)}
        for key in ('type', 'level'):
            g = collections.defaultdict(list)
            for r in rows:
                g[r[key]].append(r)
            res['by_' + key] = {k: E.summarize(v) for k, v in sorted(g.items(), key=lambda kv: str(kv[0]))}
        res['failures'] = collections.Counter(f"{by[r['scenario_id']]['type']}:{r['eval']['label']}:"
                                              f"{r['eval']['sub_reason']}" for r in recs
                                              if r['eval']['label'] not in ('SUCCESS', 'SAFE_STOP')).most_common()
        res['compute_ms_p99_max'] = max(r.get('compute_ms_p99', 0) for r in recs)
        out['splits'][sp] = res
        a_ = res['all']
        print(sp, {k: a_[k] for k in ('n', 'SR', 'CR', 'SSR', 'STR')}, res['failures'][:6], flush=True)
        json.dump(out, open(a.out, 'w'), indent=1, default=float)
    return 0


if __name__ == '__main__':
    sys.exit(main())
