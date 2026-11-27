#!/usr/bin/env python3
"""Tổng hợp kết quả baseline_eval.py (Master Plan v5 bước 8): SR, CR, STR, SSR, LAR, replan, thời gian, quãng đường,
hở min — theo nhóm benchmark (general / classroom / house / stress), bộ tham số, type, mức, type×mức; độ ổn định theo
seed (SR từng seed, độ lệch chuẩn); thời gian / quãng đường GHÉP CẶP (episode SUCCESS ở mọi bộ được so sánh trong
cùng nhóm); danh sách trường hợp thất bại.

  baseline_report.py --inputs a.jsonl.gz[,b.jsonl.gz] --out report.json --md report.md [--sets default,general_best]
Nhóm: general = general_val + general_s1_val, classroom = classroom_val + classroom_s1_val, house tương tự,
stress = stress_pilot (chẩn đoán, ngoài dataset v3), train = general_train / *_finetune (chỉ khi có).
"""
import argparse
import collections
import gzip
import json
import math
import sys

import numpy as np


def group_of(split):
    if split.startswith('stress'):
        return 'stress'
    if split.endswith('_finetune') or split.endswith('_train'):
        return 'train:' + split
    return split.split('_')[0]


def load(paths):
    rows = []
    for p in paths:
        with gzip.open(p, 'rt') as f:
            rows += [json.loads(x) for x in f]
    # trùng (cùng split, set, id, seed): giữ bản cuối
    d = {(r['split'], r['set'], r['id'], r['seed']): r for r in rows}
    return list(d.values())


def pct(a, b):
    return 100.0 * a / b if b else None


def metrics(rs):
    f = [r for r in rs if r['feas'] == 'feasible' and r['label'] != 'SYSTEM_ERROR']
    u = [r for r in rs if r['feas'] == 'unsolvable' and r['label'] != 'SYSTEM_ERROR']
    lab = collections.Counter(r['label'] for r in f)
    labu = collections.Counter(r['label'] for r in u)
    succ = [r for r in f if r['label'] == 'SUCCESS']
    out = {'n': len(rs), 'Nf': len(f), 'Nu': len(u), 'labels': dict(lab), 'labels_unsolvable': dict(labu),
           'SR': pct(lab['SUCCESS'], len(f)), 'STR': pct(lab['STUCK'], len(f)), 'TOR': pct(lab['TIMEOUT'], len(f)),
           'CR': pct(lab['COLLISION'] + labu['COLLISION'], len(f) + len(u)),
           'SSR': pct(labu['SAFE_STOP'], len(u)),
           'system_errors': sum(1 for r in rs if r['label'] == 'SYSTEM_ERROR')}
    if f:
        out['replan_rate'] = pct(sum(1 for r in f if r['replans'] > 0), len(f))
        out['replan_per_ep'] = float(np.mean([r['replans'] for r in f]))
        segs = [s for r in f for s in r.get('segments', []) if s['status'] != 'not_reached']
        st = collections.Counter(s['status'] for s in segs)
        out['segments'] = len(segs)
        out['seg_status'] = dict(st)
        out['LAR'] = pct(st['local'], len(segs))
        out['q'] = float(np.mean([r['q'] for r in f]))
        clr = [r['clr'] for r in f + u if math.isfinite(r['clr'])]
        out['clr_min'] = float(np.min(clr)) if clr else None
        out['clr_p05'] = float(np.percentile(clr, 5)) if clr else None
        out['clr_mean'] = float(np.mean(clr)) if clr else None
    if succ:
        out['time_mean'] = float(np.mean([r['time'] for r in succ]))
        out['path_mean'] = float(np.mean([r['path_length'] for r in succ]))
        out['path_ratio'] = float(np.mean([r['path_length'] / max(0.1, r['l_init'] or r['l_ref']) for r in succ]))
    if u:
        out['S1_time'] = float(np.mean([r['time'] for r in u]))
        out['S1_replans'] = float(np.mean([r['replans'] for r in u]))
    seeds = sorted({r['seed'] for r in rs})
    if len(seeds) > 1:
        per = {}
        for s in seeds:
            fs = [r for r in f if r['seed'] == s]
            us = [r for r in u if r['seed'] == s]
            per[s] = {'SR': pct(sum(r['label'] == 'SUCCESS' for r in fs), len(fs)),
                      'STR': pct(sum(r['label'] == 'STUCK' for r in fs), len(fs)),
                      'SSR': pct(sum(r['label'] == 'SAFE_STOP' for r in us), len(us)),
                      'q': float(np.mean([r['q'] for r in fs])) if fs else None}
        out['per_seed'] = per
        for k in ('SR', 'STR', 'SSR', 'q'):
            v = [p[k] for p in per.values() if p[k] is not None]
            if v:
                out[k + '_seed_std'] = float(np.std(v, ddof=1)) if len(v) > 1 else 0.0
                out[k + '_seed_range'] = [float(min(v)), float(max(v))]
        # scenario ổn định: kết cục như nhau ở mọi seed
        by = collections.defaultdict(set)
        for r in f:
            by[r['id']].add(r['label'] == 'SUCCESS')
        out['scen_consistent'] = pct(sum(1 for v in by.values() if len(v) == 1), len(by))
        out['scen_always_success'] = pct(sum(1 for v in by.values() if v == {True}), len(by))
        out['scen_never_success'] = pct(sum(1 for v in by.values() if v == {False}), len(by))
    causes = collections.Counter(e['cause'] for r in f for e in r.get('replan_events', []))
    out['replan_causes'] = dict(causes.most_common())
    out['failures'] = collections.Counter(f"{r['type']}:{r['label']}:{r['sub']}" for r in f + u
                                          if r['label'] not in ('SUCCESS', 'SAFE_STOP')).most_common(12)
    return out


def paired(rows, sets):
    """Thời gian / quãng đường trên episode SUCCESS ở MỌI bộ trong `sets` (cùng split, id, seed)."""
    ok = collections.defaultdict(dict)
    for r in rows:
        if r['set'] in sets and r['label'] == 'SUCCESS':
            ok[(r['split'], r['id'], r['seed'])][r['set']] = r
    keys = [k for k, v in ok.items() if len(v) == len(sets)]
    out = {'n': len(keys)}
    for s in sets:
        out[s] = {'time': float(np.mean([ok[k][s]['time'] for k in keys])) if keys else None,
                  'path': float(np.mean([ok[k][s]['path_length'] for k in keys])) if keys else None,
                  'clr_mean': float(np.mean([ok[k][s]['clr'] for k in keys])) if keys else None}
    return out


def build(rows, sets=None):
    sets = sets or sorted({r['set'] for r in rows})
    G = collections.defaultdict(list)
    for r in rows:
        G[(group_of(r['split']), r['set'])].append(r)
    groups = sorted({g for g, _ in G})
    rep = {'sets': sets, 'groups': groups, 'summary': {}, 'by_type': {}, 'by_level': {}, 'by_type_level': {},
           'paired': {}, 'failure_cases': {}}
    for g in groups:
        rep['summary'][g] = {s: metrics(G[(g, s)]) for s in sets if G.get((g, s))}
        present = [s for s in sets if G.get((g, s))]
        if len(present) > 1:
            rep['paired'][g] = paired([r for s in present for r in G[(g, s)]], present)
        for key, name in (('type', 'by_type'), ('level', 'by_level')):
            T = collections.defaultdict(dict)
            for s in present:
                sub = collections.defaultdict(list)
                for r in G[(g, s)]:
                    sub[r[key] if key == 'type' else ('unsolvable' if r['feas'] != 'feasible' else r[key])].append(r)
                for k, v in sub.items():
                    T[k][s] = metrics(v)
            rep[name][g] = dict(sorted(T.items()))
        TL = collections.defaultdict(dict)
        for s in present:
            sub = collections.defaultdict(list)
            for r in G[(g, s)]:
                sub[f"{r['type']}:{r['level']}"].append(r)
            for k, v in sub.items():
                m = metrics(v)
                TL[k][s] = {x: m.get(x) for x in ('n', 'SR', 'STR', 'SSR', 'CR', 'replan_rate', 'LAR', 'time_mean')}
        rep['by_type_level'][g] = dict(sorted(TL.items()))
        for s in present:
            fails = collections.defaultdict(list)
            for r in G[(g, s)]:
                if r['label'] not in ('SUCCESS', 'SAFE_STOP'):
                    fails[r['id']].append(r)
            rep['failure_cases'].setdefault(g, {})[s] = sorted(
                ({'id': k, 'type': v[0]['type'], 'level': v[0]['level'], 'n_fail': len(v),
                  'labels': sorted({f"{r['label']}:{r['sub']}" for r in v}),
                  'replans': [r['replans'] for r in v]} for k, v in fails.items()),
                key=lambda x: (-x['n_fail'], x['id']))
    return rep


def f1(x, d=1):
    return '–' if x is None else f'{x:.{d}f}'


def to_md(rep):
    md = []
    for g in rep['groups']:
        S = rep['summary'][g]
        md.append(f'### {g}\n')
        md.append('| Bộ | n | SR | CR | STR | SSR | Replan % ep | Replan/ep | LAR | t (s) | L (m) | hở min / p05 | q |')
        md.append('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
        for s, m in S.items():
            sd = f" ±{m['SR_seed_std']:.1f}" if 'SR_seed_std' in m else ''
            md.append(f"| {s} | {m['n']} | {f1(m['SR'])}{sd} | {f1(m['CR'], 2)} | {f1(m['STR'])} | {f1(m['SSR'])} | "
                      f"{f1(m.get('replan_rate'))} | {f1(m.get('replan_per_ep'), 2)} | {f1(m.get('LAR'))} | "
                      f"{f1(m.get('time_mean'))} | {f1(m.get('path_mean'), 2)} | "
                      f"{f1(m.get('clr_min'), 3)} / {f1(m.get('clr_p05'), 3)} | {f1(m.get('q'), 3)} |")
        if g in rep['paired']:
            P = rep['paired'][g]
            md.append(f"\nGhép cặp ({P['n']} episode SUCCESS ở mọi bộ): " + '; '.join(
                f"{s} t {f1(P[s]['time'])} s, L {f1(P[s]['path'], 2)} m" for s in rep['summary'][g]))
        md.append('\n| Type | ' + ' | '.join(S) + ' |')
        md.append('|---|' + '---|' * len(S))
        for t, v in rep['by_type'][g].items():
            cells = []
            for s in S:
                m = v.get(s, {})
                if t == 'S1':
                    cells.append(f"SSR {f1(m.get('SSR'))}")
                else:
                    cells.append(f"{f1(m.get('SR'))} / {f1(m.get('STR'))} / {f1(m.get('replan_rate'), 0)} / "
                                 f"{f1(m.get('LAR'), 0)}")
            md.append(f'| {t} | ' + ' | '.join(cells) + ' |')
        md.append('\n(ô = SR / STR / replan % ep / LAR)\n')
        md.append('| Mức | ' + ' | '.join(S) + ' |')
        md.append('|---|' + '---|' * len(S))
        for t, v in rep['by_level'][g].items():
            md.append(f'| {t} | ' + ' | '.join(
                (f"SSR {f1(v.get(s, {}).get('SSR'))}" if t == 'unsolvable' else
                 f"{f1(v.get(s, {}).get('SR'))} / {f1(v.get(s, {}).get('STR'))}") for s in S) + ' |')
        md.append('')
    return '\n'.join(md)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--inputs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    ap.add_argument('--sets', default='')
    a = ap.parse_args()
    rows = load(a.inputs.split(','))
    rep = build(rows, a.sets.split(',') if a.sets else None)
    json.dump(rep, open(a.out, 'w'), indent=1, default=float, ensure_ascii=False)
    md = to_md(rep)
    if a.md:
        open(a.md, 'w').write(md + '\n')
    print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
