#!/usr/bin/env python3
"""Sinh dataset cố định (Master Plan v3 §7): General 900/300/300, Classroom & House 120/100/100 mỗi map, S1 riêng.

  gen_scenarios.py --out ~/capstone_ws/bench_data/v3 [--workers 10] [--only general_train,...] [--scale 0.1]

- Phân tầng theo bảng dataset.* trong benchmark.yaml (bảng 7.2 đã điều chỉnh theo pilot, ghi lý do trong file).
- Seed namespace riêng mỗi split (split_index·10^7 + type·10^5 + k) -> layout tổng quát không trùng giữa split.
- Map mục tiêu: cặp (start, goal) gộp theo ô 1 m; một cặp ô (và láng giềng) chỉ thuộc một split -> không có nhiệm vụ
  gần trùng giữa fine-tune / validation / test.
- Kiểm tra rò rỉ: hash scenario và hash map trùng giữa split -> báo lỗi.
- Không đổi type khi sinh thiếu: ô thiếu chuyển sang mức kề CÙNG type (ghi lại), rồi tới type khác cùng mức.
"""

import argparse
import collections
import hashlib
import json
import math
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings('ignore')

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import generator as GN  # noqa: E402
from stage5_benchmark import layouts as LY  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402

SPLIT_INDEX = {'general_train': 1, 'general_val': 2, 'general_test': 3, 'general_s1_val': 4, 'general_s1_test': 5,
               'classroom_finetune': 6, 'classroom_val': 7, 'classroom_test': 8, 'classroom_s1_val': 9,
               'classroom_s1_test': 10, 'house_finetune': 11, 'house_val': 12, 'house_test': 13, 'house_s1_val': 14,
               'house_s1_test': 15}
ALL_TYPES = GN.TYPES + ['S1']
# Láng giềng ô (start, goal) 1 m cũng bị khóa cho split đầu tiên dùng nó. House (18.8 x 11.2 m) quá nhỏ cho luật này
# (lần sinh 1: house_test 14/100) -> chỉ khóa đúng ô 1 m (start, goal) — vẫn không có cặp nhiệm vụ trùng ô giữa split.
SG_NEIGHBORS = {'classroom': True, 'house': False}


def level_of(score, gen):
    if score is None:
        return 'unsolvable'
    return 'easy' if score < gen['easy_max'] else ('medium' if score < gen['medium_max'] else 'hard')


_BASES = {}


def _job(args):
    stype, seed, hint, map_name, gen, nav = args
    base = None
    if map_name:
        if map_name not in _BASES:
            M, roi, _ = LY.target_map(map_name)
            _BASES[map_name] = (M, roi, {'family': map_name})
        base = _BASES[map_name]
    sc, reasons = GN.generate_one(stype, seed, gen, nav, base=base, hint=hint, map_name=map_name)
    if sc is not None and map_name:
        sc['map'] = None            # map mục tiêu: không gửi lưới lớn qua tiến trình
    return stype, seed, hint, sc, reasons


def sg_key(sc, cell=1.0):
    return (math.floor(sc['start'][0] / cell), math.floor(sc['start'][1] / cell),
            math.floor(sc['goal'][0] / cell), math.floor(sc['goal'][1] / cell))


def neighbors(key):
    a, b, c, d = key
    return {(a + i, b + j, c + k, d + m) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)
            for m in (-1, 0, 1)}


def fill_split(split, plan, gen, nav, ex, map_name=None, reserved=None, log=print):
    """plan: {(type, level): count}. Trả (scenarios, stats)."""
    sidx = SPLIT_INDEX[split]
    need = collections.Counter(plan)
    got = collections.defaultdict(list)
    stats = {'attempted': collections.Counter(), 'accepted': collections.Counter(), 'reasons': collections.Counter(),
             'moved': [], 'shortfall': {}}
    types = sorted({t for t, _ in plan}, key=ALL_TYPES.index)
    next_k = {t: 0 for t in types}
    max_jobs = {t: gen.get('max_attempts', 200) * max(1, sum(v for (tt, _), v in plan.items() if tt == t) // 20 + 1)
                for t in types}
    for _round in range(40):
        jobs = []
        for t in types:
            open_levels = [lv for (tt, lv), n in need.items() if tt == t and len(got[(tt, lv)]) < n]
            hints = ['easy', 'medium', 'hard'] if open_levels == ['any'] else open_levels
            if not open_levels or next_k[t] >= max_jobs[t]:
                continue
            missing = sum(need[(t, lv)] - len(got[(t, lv)]) for lv in open_levels)
            batch = min(max(8, 2 * missing), max_jobs[t] - next_k[t])
            for q in range(batch):
                k = next_k[t] + q
                hint = hints[k % len(hints)] if t != 'S1' else 'any'
                seed = sidx * 10 ** 7 + ALL_TYPES.index(t) * 10 ** 5 + k
                jobs.append((t, seed, hint, map_name, gen, nav))
            next_k[t] += batch
        if not jobs:
            break
        for t, seed, hint, sc, reasons in ex.map(_job, jobs, chunksize=1):
            stats['attempted'][t] += 1
            stats['reasons'].update({f'{t}:{k}': v for k, v in reasons.items()})
            if sc is None:
                continue
            lv = level_of(sc['difficulty'].get('score'), gen) if t != 'S1' else 'unsolvable'
            sc['difficulty']['level'] = lv
            bucket = (t, 'any') if (t, 'any') in need else (t, lv)
            if len(got[bucket]) >= need.get(bucket, 0):
                continue
            if reserved is not None:
                key = sg_key(sc)
                near = neighbors(key) if SG_NEIGHBORS.get(map_name, True) else {key}
                if any(n in reserved and reserved[n] != split for n in near):
                    stats['reasons'][f'{t}:leakage_sg_reserved'] += 1
                    continue
                reserved.setdefault(key, split)
            sc['split'] = split
            sc['id'] = f'{split}_{t}_{lv[0]}_{len(got[bucket]):04d}'
            got[bucket].append(sc)
            stats['accepted'][t] += 1
        # ô không thể sinh: chuyển phần thiếu sang mức kề cùng type khi đã hết ngân sách type đó
        for (t, lv), n in list(need.items()):
            if len(got[(t, lv)]) < n and next_k[t] >= max_jobs[t]:
                miss = n - len(got[(t, lv)])
                if lv in ('any', 'unsolvable'):
                    stats['shortfall'][f'{t}:{lv}'] = miss       # không có mức kề: ghi thiếu
                    need[(t, lv)] = len(got[(t, lv)])
                    continue
                need[(t, lv)] = len(got[(t, lv)])
                order = {'easy': ['medium', 'hard'], 'medium': ['hard', 'easy'], 'hard': ['medium', 'easy']}.get(lv, [])
                for alt in order:
                    if alt in ('easy', 'medium', 'hard'):
                        need[(t, alt)] = need.get((t, alt), 0) + miss
                        stats['moved'].append([t, lv, alt, miss])
                        max_jobs[t] += gen.get('max_attempts', 200)
                        break
        log(f'  [{split}] round {_round}: ' + ', '.join(
            f'{t}:{sum(len(got[(t, lv)]) for lv in ("easy", "medium", "hard", "unsolvable", "any"))}' for t in types))
    out = []
    for key in sorted(got, key=lambda k: (ALL_TYPES.index(k[0]), k[1])):
        out += got[key]
    for (t, lv), n in need.items():
        if len(got[(t, lv)]) < n:
            stats['shortfall'][f'{t}:{lv}'] = n - len(got[(t, lv)])
    stats = {k: (dict(v) if isinstance(v, collections.Counter) else v) for k, v in stats.items()}
    stats['table'] = {f'{t}:{lv}': len(v) for (t, lv), v in got.items()}
    return out, stats


def plan_from(spec, types):
    """spec: {type: {easy: n, medium: n, hard: n}} | {'per_level': n} (cân bằng theo mức, chia đều type)."""
    if 'per_level' in spec:
        plan = {}
        for lv in ('easy', 'medium', 'hard'):
            allowed = [t for t in types if lv not in spec.get('exclude', {}).get(t, [])]
            base, rem = divmod(spec['per_level'], len(allowed))
            for q, t in enumerate(allowed):
                plan[(t, lv)] = base + (1 if q < rem else 0)
        return plan
    if 'any_total' in spec:      # map mục tiêu: chia đều type, không ép mức khó (ghi phân bố thực tế)
        base, rem = divmod(spec['any_total'], len(types))
        return {(t, 'any'): base + (1 if q < rem else 0) for q, t in enumerate(types)}
    return {(t, lv): n for t, row in spec.items() for lv, n in row.items() if n}


def config_hash():
    h = hashlib.sha1()
    for p in (C.nav2_params_path(), C.benchmark_yaml_path()):
        h.update(open(p, 'rb').read())
    return h.hexdigest()[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--only', default='')
    ap.add_argument('--scale', type=float, default=1.0, help='nhân số lượng (pilot nhỏ)')
    a = ap.parse_args()
    cfg = C.load()
    gen, nav = cfg['gen'], GN.nav_params(cfg)
    ds = __import__('yaml').safe_load(open(C.benchmark_yaml_path()))['dataset']
    os.makedirs(a.out, exist_ok=True)
    summary = {'config_hash': config_hash(), 'generator_version': IO.GENERATOR_VERSION, 'splits': {},
               'thresholds': {'easy_max': gen['easy_max'], 'medium_max': gen['medium_max']}}
    sp = os.path.join(a.out, 'summary.json')
    if os.path.exists(sp):
        summary['splits'] = json.load(open(sp)).get('splits', {})
    only = set(a.only.split(',')) if a.only else None
    reserved = {'classroom': {}, 'house': {}}
    t_all = time.time()
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for split, spec in ds['splits'].items():
            if only and split not in only:
                continue
            map_name = spec.get('map')
            types = spec['types']
            plan = plan_from(spec['plan'], types)
            if a.scale != 1.0:
                plan = {k: max(1, round(v * a.scale)) if v else 0 for k, v in plan.items()}
            t0 = time.time()
            print(f'== {split}: {sum(plan.values())} scenarios ({map_name or "generated"})', flush=True)
            scs, st = fill_split(split, plan, gen, nav, ex, map_name=map_name,
                                 reserved=reserved[map_name] if map_name else None,
                                 log=lambda m: print(m, flush=True))
            if map_name:
                M, _, _ = LY.target_map(map_name)
                for s in scs:
                    s['map'] = M
            IO.save_split(os.path.join(a.out, split), scs, {'split': split, 'plan': {f'{t}:{lv}': n for (t, lv), n in
                                                                                       plan.items()}, 'stats': st})
            st['seconds'] = time.time() - t0
            st['n'] = len(scs)
            summary['splits'][split] = st
            with open(sp, 'w') as f:
                json.dump(summary, f, ensure_ascii=False, indent=1)
    # rò rỉ: hash scenario + hash layout tổng quát trùng giữa split
    seen, layout_seen, dups = {}, {}, []
    for split in summary['splits']:
        p = os.path.join(a.out, split, 'manifest.jsonl')
        if not os.path.exists(p):
            continue
        for line in open(p):
            r = json.loads(line)
            if r['hash'] in seen and seen[r['hash']] != split:
                dups.append(['scenario', r['id'], seen[r['hash']]])
            seen.setdefault(r['hash'], split)
            if r['map_name'] == 'generated':
                if r['map_id'] in layout_seen and layout_seen[r['map_id']] != split:
                    dups.append(['layout', r['id'], layout_seen[r['map_id']]])
                layout_seen.setdefault(r['map_id'], split)
    summary['leakage'] = {'duplicates_across_splits': dups, 'ok': not dups}
    summary['total_seconds'] = time.time() - t_all
    with open(sp, 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print('leakage ok' if not dups else f'LEAKAGE: {dups[:5]}')
    return 0 if not dups else 1


if __name__ == '__main__':
    sys.exit(main())
