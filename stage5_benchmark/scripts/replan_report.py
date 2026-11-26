#!/usr/bin/env python3
"""Tổng hợp kết quả replan_compare.py (Master Plan V5 §3.6) -> JSON + bảng Markdown.

Chỉ số (feasible trừ khi ghi khác): SR, CR, STR, SSR (S1, vô nghiệm), Replan Rate (% episode có >= 1 replan),
Replan Count (trung bình / episode), LAR (Local Avoidance Rate = đoạn tương tác vượt qua KHÔNG replan / đoạn đã
tới), thời gian + quãng đường (ghép cặp: episode SUCCESS ở MỌI cơ chế so sánh), hở GT nhỏ nhất (min, p05),
nguyên nhân replan, số lần robot bị ép vào ô INSCRIBED (trap).
Phân loại ghép cặp (cùng scenario, seed, bộ tham số):
  r1_replan_dwa_ok   R1 replan trong đoạn nhưng R2 / R3 vượt đoạn đó KHÔNG replan
  needs_global       R3 thất bại, R2 thành công nhờ replan  (bắt buộc cần Global Replanning)
  fewer_replan_worse R2 ít replan hơn R1 và kết cục xấu hơn / hở nhỏ hơn 0.03 m
  false_stuck        R2 replan do LOCAL_BLOCKED mà R3 (chỉ DWA) vẫn thành công không replan
  sac_potential      scenario mà ÍT NHẤT một bộ tham số khác vượt KHÔNG replan còn bộ default phải replan / thất bại

  replan_report.py --inputs a.jsonl[,b.jsonl] --out report.json [--md report.md] [--ref R1]
"""
import argparse
import collections
import json
import math
import sys

import numpy as np

TYPES = ['N1', 'N2', 'N3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3', 'C4', 'S1']


def load(paths):
    recs = []
    for p in paths.split(','):
        with open(p) as f:
            recs += [json.loads(line) for line in f if line.strip()]
    return recs


def summarize(rs, paired_keys=None):
    f = [r for r in rs if r['feas'] == 'feasible' and r['label'] != 'SYSTEM_ERROR']
    u = [r for r in rs if r['feas'] != 'feasible' and r['label'] != 'SYSTEM_ERROR']
    out = {'n': len(f), 'n_unsolvable': len(u)}
    if f:
        lab = collections.Counter(r['label'] for r in f)
        out.update({'SR': 100 * lab['SUCCESS'] / len(f), 'CR': 100 * lab['COLLISION'] / len(f),
                    'STR': 100 * lab['STUCK'] / len(f), 'TOR': 100 * lab['TIMEOUT'] / len(f),
                    'replan_rate': 100 * float(np.mean([r['replans'] > 0 for r in f])),
                    'replan_count': float(np.mean([r['replans'] for r in f])),
                    'obstacle_replan_count': float(np.mean([r['orep'] for r in f])),
                    'trap_eps': int(sum(r['traps'] > 0 for r in f))})
        clr = [r['clr'] for r in f if math.isfinite(r['clr'])]
        out['clr_min'] = float(min(clr)) if clr else None
        out['clr_p05'] = float(np.percentile(clr, 5)) if clr else None
        segs = [s for r in f for s in r['segments'] if s['status'] != 'not_reached']
        st = collections.Counter(s['status'] for s in segs)
        out['segments'] = len(segs)
        out['LAR'] = 100 * st['local'] / len(segs) if segs else None
        out['seg_status'] = dict(st)
        if paired_keys is not None:
            pk = [r for r in f if (r['id'], r['seed']) in paired_keys]
            out['paired_n'] = len(pk)
            out['time_paired'] = float(np.mean([r['time'] for r in pk])) if pk else None
            out['path_paired'] = float(np.mean([r['path_length'] for r in pk])) if pk else None
    if u:
        lab = collections.Counter(r['label'] for r in u)
        out['SSR'] = 100 * lab['SAFE_STOP'] / len(u)
        out['S1_replan_count'] = float(np.mean([r['replans'] for r in u]))
        out['S1_time'] = float(np.mean([r['time'] for r in u]))
    causes = collections.Counter(e['cause'] for r in rs for e in r['replan_events'])
    out['causes'] = dict(causes)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--inputs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    ap.add_argument('--ref', default='R1')
    ap.add_argument('--final', default='R2s', help='cơ chế được đánh giá trong phân loại ghép cặp (vai trò "R2")')
    a = ap.parse_args()
    recs = load(a.inputs)
    by = collections.defaultdict(dict)           # (set, mech) -> {(id, seed): rec}
    for r in recs:
        by[(r['set'], r['mech'])][(r['id'], r['seed'])] = r
    sets = sorted({k[0] for k in by}, key=lambda s: (s != 'default', s))
    mechs = sorted({k[1] for k in by})
    report = {'sets': sets, 'mechs': mechs, 'summary': {}, 'by_type': {}, 'paired': {}, 'trigger_state': {}}
    md = []
    for st in sets:
        ms = [m for m in mechs if (st, m) in by]
        keys = set.intersection(*[set(by[(st, m)]) for m in ms])
        paired = {k for k in keys if all(by[(st, m)][k]['label'] == 'SUCCESS' for m in ms)}
        report['summary'][st] = {m: summarize([by[(st, m)][k] for k in keys], paired) for m in ms}
        report['by_type'][st] = {}
        for t in TYPES:
            kt = [k for k in keys if by[(st, ms[0])][k]['type'] == t]
            if kt:
                report['by_type'][st][t] = {m: summarize([by[(st, m)][k] for k in kt]) for m in ms}
        # trạng thái DWA lúc kích hoạt replan (theo nguyên nhân)
        trig = {}
        for m in ms:
            g = collections.defaultdict(list)
            for k in keys:
                for e in by[(st, m)][k]['replan_events']:
                    g[e['cause']].append(e)
            trig[m] = {c: {'n': len(v), 'fs_free_med': float(np.median([e['fs_free'] for e in v])),
                           'fs_clr_med': float(np.median([e['fs_clr'] for e in v])),
                           'block_dist_med': float(np.median([e['block_dist'] for e in v])),
                           'v_med': float(np.median([e['v'] for e in v])),
                           'nf_ratio_med': float(np.median([e['nf'] / max(1, e['nc']) for e in v])),
                           'gt_clr_med': float(np.median([e['gt_clr'] for e in v]))} for c, v in g.items()}
        report['trigger_state'][st] = trig
        # ---- phân loại ghép cặp ----
        pc = {}
        fin = a.final if (st, a.final) in by else 'R2'
        if all(m in ms for m in ('R1', fin, 'R3')):
            R1, R2, R3 = by[(st, 'R1')], by[(st, fin)], by[(st, 'R3')]
            r1_dwa_ok, needs_global, worse, false_stuck = [], [], [], []
            for k in keys:
                a1, a2, a3 = R1[k], R2[k], R3[k]
                for i, s1 in enumerate(a1['segments']):
                    if s1['replans'] == 0 or s1['status'] == 'not_reached':
                        continue
                    ok2 = i < len(a2['segments']) and a2['segments'][i]['status'] == 'local'
                    ok3 = i < len(a3['segments']) and a3['segments'][i]['status'] == 'local'
                    if ok2 or ok3:
                        r1_dwa_ok.append({'id': k[0], 'seed': k[1], 'type': a1['type'], 'seg': i, 'R2_local': ok2,
                                          'R3_local': ok3, 'R1_causes': s1['causes']})
                if a2['feas'] == 'feasible' and a3['label'] != 'SUCCESS' and a2['label'] == 'SUCCESS' and \
                        a2['replans'] > 0:
                    needs_global.append({'id': k[0], 'type': a2['type'], 'R3': f"{a3['label']}:{a3['sub']}",
                                         'R2_causes': [e['cause'] for e in a2['replan_events']]})
                rank = {'SUCCESS': 0, 'TIMEOUT': 1, 'STUCK': 2, 'SAFE_STOP': 0, 'COLLISION': 3}
                if a2['replans'] < a1['replans'] and (rank.get(a2['label'], 2) > rank.get(a1['label'], 2) or
                                                      a2['clr'] < min(a1['clr'], 0.03)):
                    worse.append({'id': k[0], 'type': a2['type'], 'R1': a1['label'], 'R2': a2['label'],
                                  'R1_replans': a1['replans'], 'R2_replans': a2['replans'],
                                  'clr': [round(a1['clr'], 3), round(a2['clr'], 3)]})
                if a3['label'] == 'SUCCESS' and a2['replans'] > 0 and \
                        any(e['cause'].startswith('local_') or e['cause'] == 'no_useful_progress'
                            for e in a2['replan_events']):
                    false_stuck.append({'id': k[0], 'type': a2['type'],
                                        'R2_causes': [e['cause'] for e in a2['replan_events']],
                                        'state': [{x: e[x] for x in ('fs_free', 'fs_clr', 'block_dist', 'v', 'nf', 'nc')}
                                                  for e in a2['replan_events']]})
            pc = {'final': fin, 'r1_replan_dwa_ok': len(r1_dwa_ok), 'r1_replan_dwa_ok_R2': sum(x['R2_local'] for x in r1_dwa_ok),
                  'r1_replan_dwa_ok_R3': sum(x['R3_local'] for x in r1_dwa_ok),
                  'needs_global': len(needs_global),
                  'needs_global_by_type': dict(collections.Counter(x['type'] for x in needs_global)),
                  'fewer_replan_worse': worse, 'false_stuck': false_stuck,
                  'r1_replan_dwa_ok_by_type': dict(collections.Counter(x['type'] for x in r1_dwa_ok)),
                  'examples': {'r1_replan_dwa_ok': r1_dwa_ok[:30], 'needs_global': needs_global[:30]}}
        report['paired'][st] = pc
    # tiềm năng SAC: so default với các bộ khác cùng cơ chế
    sac = {}
    for m in mechs:
        if ('default', m) not in by:
            continue
        others = [s for s in sets if s != 'default' and (s, m) in by]
        if not others:
            continue
        d = by[('default', m)]
        cnt = collections.Counter()
        ex = []
        for k, r in d.items():
            if r['feas'] != 'feasible':
                continue
            bad = r['label'] != 'SUCCESS' or r['orep'] > 0
            if not bad:
                continue
            alts = [s for s in others if k in by[(s, m)] and by[(s, m)][k]['label'] == 'SUCCESS' and
                    by[(s, m)][k]['orep'] == 0]
            if alts:
                cnt[r['type']] += 1
                ex.append({'id': k[0], 'default': f"{r['label']}/{r['orep']}rp", 'alt_sets': alts})
        sac[m] = {'n': int(sum(cnt.values())), 'by_type': dict(cnt), 'examples': ex[:20]}
    report['sac_potential'] = sac
    json.dump(report, open(a.out, 'w'), indent=1, default=float)

    # ---- Markdown (bảng hẹp) ----
    def f1(x, nd=1):
        return '–' if x is None else f'{x:.{nd}f}'
    for st in sets:
        S = report['summary'][st]
        md.append(f'### Bộ tham số `{st}`\n')
        md.append('| Cơ chế | SR | CR | STR | SSR | Replan % | Replan/ep | LAR | t (s) | L (m) | hở min |')
        md.append('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
        for m, s in S.items():
            md.append(f"| {m} | {f1(s.get('SR'))} | {f1(s.get('CR'), 2)} | {f1(s.get('STR'))} | {f1(s.get('SSR'))} | "
                      f"{f1(s.get('replan_rate'))} | {f1(s.get('replan_count'), 2)} | {f1(s.get('LAR'))} | "
                      f"{f1(s.get('time_paired'))} | {f1(s.get('path_paired'), 2)} | {f1(s.get('clr_min'), 3)} |")
        md.append('')
    if md:
        md.append('### Theo nhóm (bộ default): SR / Replan % / LAR\n')
        T = report['by_type'].get('default', {})
        ms = list(report['summary']['default'])
        md.append('| Nhóm | ' + ' | '.join(ms) + ' |')
        md.append('|---|' + '---|' * len(ms))
        for t, v in T.items():
            cells = []
            for m in ms:
                s = v[m]
                if t == 'S1':
                    cells.append(f"SSR {f1(s.get('SSR'))} / {f1(s.get('S1_replan_count'), 1)} rp")
                else:
                    cells.append(f"{f1(s.get('SR'))} / {f1(s.get('replan_rate'), 0)} / {f1(s.get('LAR'), 0)}")
            md.append(f'| {t} | ' + ' | '.join(cells) + ' |')
    if a.md:
        open(a.md, 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))
    return 0


if __name__ == '__main__':
    sys.exit(main())
