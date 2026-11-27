#!/usr/bin/env python3
"""Hình cho bước 8 (Baseline M0, Navigation V5).

episodes: mỗi scenario một hình, mỗi bộ tham số một cột. Hàng trên: map M (xám), vật cản mới Δ (đỏ, robot không
  biết trước), A* ban đầu (xanh đứt), A* sau replan (cam), quỹ đạo GT (đen, tô màu theo v), vị trí replan (sao +
  nguyên nhân), start (tròn) / goal (vuông). Hàng dưới: trạng thái DWA — cung tự do, fs_clearance, n_fwd/n_cand,
  v, trạng thái ReplanSupervisor, hở GT.
    baseline_figures.py episodes --split v3/general_val --ids a,b --sets sets.json --names default,general_best
        --out figs [--seed 0]
  --auto ev.jsonl.gz: tự chọn ca (thành công tự né, thành công cần replan, thất bại, khác biệt default ↔ best).
summary: hình tóm tắt tối ưu (J theo lần thử + best-so-far, tham số top-10, SR theo type của các bộ).
    baseline_figures.py summary --tune DIR --report report.json --out figs
"""
import argparse
import collections
import gzip
import json
import math
import os
import sys
import warnings

warnings.filterwarnings('ignore')

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from replan_compare import classify  # noqa: E402
from replan_figures import draw_shape  # noqa: E402
from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import env as ENV  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402

SUP = ['NORMAL', 'PATH_BLOCKED', 'LOCAL_BLOCKED', 'GLOBAL_BLOCKED', 'SAFE_STOP']


def auto_cases(ev_path, names, per=2):
    rows = [json.loads(x) for x in gzip.open(ev_path, 'rt')]
    rows = [r for r in rows if r['seed'] == 0 and r['set'] in names]
    by = collections.defaultdict(dict)
    for r in rows:
        by[(r['split'], r['id'])][r['set']] = r
    ref, best = names[0], names[-1]
    pick = collections.OrderedDict()

    def add(tag, cond, key=None):
        c = [(k, v) for k, v in sorted(by.items()) if len(v) == len(names) and cond(v)]
        if key:
            c.sort(key=key)
        for k, v in c[:per]:
            pick.setdefault(k, tag)

    seg_local = lambda r: any(s['status'] == 'local' for s in r.get('segments', []))  # noqa: E731
    add('thành công, tự né (không replan)', lambda v: v[best]['label'] == 'SUCCESS' and v[best]['replans'] == 0 and
        seg_local(v[best]) and v[best]['level'] == 'hard', key=lambda kv: -kv[1][best]['l_ref'])
    add('thành công nhờ replan', lambda v: v[best]['label'] == 'SUCCESS' and v[best]['replans'] > 0 and
        v[best]['type'] in ('B1', 'B2', 'C2'), key=lambda kv: -kv[1][best]['replans'])
    add('thành công nhờ replan (B3)', lambda v: v[best]['label'] == 'SUCCESS' and v[best]['replans'] > 0 and
        v[best]['type'] == 'B3', key=None)
    add('thất bại (bộ tốt nhất)', lambda v: v[best]['feas'] == 'feasible' and v[best]['label'] != 'SUCCESS')
    add('S1 dừng an toàn', lambda v: v[best]['feas'] == 'unsolvable' and v[best]['label'] == 'SAFE_STOP', key=None)
    add('S1 không dừng an toàn', lambda v: v[best]['feas'] == 'unsolvable' and v[best]['label'] != 'SAFE_STOP')
    if ref != best:
        add('khác biệt: default thất bại, best thành công',
            lambda v: v[ref]['label'] != 'SUCCESS' and v[best]['label'] == 'SUCCESS' and v[ref]['feas'] == 'feasible')
        add('khác biệt: best ít replan hơn', lambda v: v[best]['label'] == 'SUCCESS' and
            v[ref]['replans'] > v[best]['replans'], key=lambda kv: kv[1][best]['replans'] - kv[1][ref]['replans'])
    return pick


def episodes(a):
    sets = json.load(open(a.sets)) if a.sets else {}
    sets.setdefault('default', {})
    names = a.names.split(',')
    os.makedirs(a.out, exist_ok=True)
    cfg = C.load()
    if a.auto:
        cases = auto_cases(a.auto, names, a.per)
    else:
        cases = collections.OrderedDict(((a.split, i), '') for i in a.ids.split(','))
    pools = {}
    index = []
    for (split, sid), tag in cases.items():
        sp = split if '/' in split else (a.split or split)
        if os.path.basename(sp).endswith('_test'):
            raise SystemExit('test split bị khóa')
        if sp not in pools:
            path = os.path.join(a.data, sp)
            if not os.path.exists(path):
                path = os.path.join(a.data, 'v3', sp)
            pools[sp] = {s['id']: s for s in IO.load_split(path)}
        sc = pools[sp][sid]
        recs = {n: RN.run_episode(sc, cfg, a.seed, sets[n]) for n in names}
        fig, axes = plt.subplots(2, len(names), figsize=(5.6 * len(names), 9.0),
                                 gridspec_kw={'height_ratios': [2.3, 1]}, squeeze=False)
        M = sc['map']
        data = np.asarray(M.data)
        ox, oy, res = M.origin[0], M.origin[1], M.res
        ext = [ox, ox + data.shape[1] * res, oy, oy + data.shape[0] * res]
        img = np.where(data >= 50, 0.15, np.where(data < 0, 0.6, 1.0))
        allxy = [np.asarray(r['trace'])[:, 1:3] for r in recs.values() if len(r['trace'])]
        allxy += [np.asarray(p[1]) for r in recs.values() for p in r.get('plans', [])]
        allxy = np.vstack(allxy) if allxy else np.zeros((1, 2))
        lo, hi = allxy.min(0) - 1.0, allxy.max(0) + 1.0
        for j, n in enumerate(names):
            r = recs[n]
            ax = axes[0, j]
            ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
            for sh in sc.get('obstacles') or []:
                draw_shape(ax, sh)
            for k, (t, P) in enumerate(r.get('plans', [])):
                P = np.asarray(P)
                if k == 0:
                    ax.plot(P[:, 0], P[:, 1], '--', color='#1f77b4', lw=1.6, label='A* ban đầu (map M)')
                else:
                    ax.plot(P[:, 0], P[:, 1], '-', color='#ff7f0e', lw=1.2, alpha=0.85,
                            label='A* sau replan' if k == 1 else None)
            tr = np.asarray(r['trace'])
            ix = {c: i for i, c in enumerate(r['trace_columns'] or [])}
            if len(tr):
                sc_ = ax.scatter(tr[::3, 1], tr[::3, 2], c=np.abs(tr[::3, 4]), cmap='viridis', vmin=0, vmax=0.3,
                                 s=4, zorder=3)
                ax.plot(tr[:, 1], tr[:, 2], '-', color='k', lw=0.6, alpha=0.6, label='robot (GT)')
            ev = [(t, s[len('REPLANNING: '):]) for t, s in r['events'] if s.startswith('REPLANNING: ')]
            for t, why in ev:
                k = int(np.argmin(np.abs(tr[:, 0] - t)))
                ax.plot(tr[k, 1], tr[k, 2], '*', color='#9467bd', ms=15, mec='k', zorder=4)
                ax.annotate(classify(why), (tr[k, 1], tr[k, 2]), fontsize=7, xytext=(4, 4),
                            textcoords='offset points')
            ax.plot(*sc['start'][:2], 'o', color='#2ca02c', ms=9, mec='k')
            ax.plot(*sc['goal'][:2], 's', color='#2ca02c', ms=9, mec='k')
            ax.set_xlim(lo[0], hi[0])
            ax.set_ylim(lo[1], hi[1])
            ax.set_aspect('equal')
            e = r['eval']
            sub = f" ({e['sub_reason']})" if e['sub_reason'] else ''
            ax.set_title(f"{n}: {e['label']}{sub}\n{e['time']:.1f} s, L {e['path_length']:.1f} m, replan "
                         f"{r['replans']}, hở min {r['min_clearance']:.3f} m", fontsize=9)
            if j == 0:
                ax.legend(fontsize=7, loc='best')
            if j == len(names) - 1 and len(tr):
                fig.colorbar(sc_, ax=ax, fraction=0.035, label='v (m/s)')
            bx = axes[1, j]
            if len(tr):
                t = tr[:, 0]
                bx.plot(t, tr[:, ix['fs_free_ratio']], color='#2ca02c', lw=1, label='cung tự do')
                bx.plot(t, np.clip(tr[:, ix['fs_clearance']], 0, 1), color='#8c564b', lw=1, label='fs_clearance m')
                nc = np.maximum(1, tr[:, ix['n_candidates']])
                bx.plot(t, tr[:, ix['n_forward']] / nc, color='#1f77b4', lw=0.8, alpha=0.7, label='n_fwd/n_cand')
                bx.plot(t, tr[:, ix['v']] / 0.3, color='k', lw=0.8, alpha=0.7, label='v / 0.3')
                bx.plot(t, np.clip(tr[:, ix['clearance']], 0, 1), color='#e377c2', lw=0.8, label='hở GT m')
                bx.step(t, tr[:, ix['sup_state']] / 4.0, color='#9467bd', lw=1.2, where='post',
                        label='trạng thái replan / 4')
                for tt, _ in ev:
                    bx.axvline(tt, color='#9467bd', ls=':', lw=1)
                bx.set_ylim(-0.05, 1.1)
                bx.set_xlabel('t (s)')
                if j == 0:
                    bx.legend(fontsize=6, loc='upper right', ncol=3)
        fig.suptitle(f"{tag + ': ' if tag else ''}{sid} ({sc['type']}, {sc['difficulty']['level']}, "
                     f"{os.path.basename(sp)}), seed {a.seed}", fontsize=11)
        fig.tight_layout()
        fn = f"{sid}.png" if sid.startswith(os.path.basename(sp)) else f"{os.path.basename(sp)}_{sid}.png"
        fig.savefig(os.path.join(a.out, fn), dpi=105)
        plt.close(fig)
        index.append({'file': fn, 'case': tag, 'split': os.path.basename(sp), 'id': sid, 'type': sc['type'],
                      'level': sc['difficulty']['level'],
                      'result': {n: {'label': recs[n]['eval']['label'], 'sub': recs[n]['eval']['sub_reason'],
                                     'time': round(recs[n]['eval']['time'], 1), 'replans': recs[n]['replans'],
                                     'clr': round(recs[n]['min_clearance'], 3)} for n in names}})
        print(fn, tag, {n: (v['label'], v['replans']) for n, v in index[-1]['result'].items()}, flush=True)
    json.dump(index, open(os.path.join(a.out, 'index.json'), 'w'), indent=1, ensure_ascii=False)


def summary(a):
    os.makedirs(a.out, exist_ok=True)
    for d in a.tune.split(','):
        T = [json.loads(x) for x in open(os.path.join(d, 'trials.jsonl'))]
        T = [t for t in T if t['J'] is not None]
        J = np.array([t['J'] for t in T])
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
        col = ['#7f7f7f' if t['how'] == 'init' else '#1f77b4' for t in T]
        ax[0].scatter(range(len(J)), J, c=col, s=18)
        ax[0].plot(np.maximum.accumulate(J), color='#d62728', lw=1.5, label='tốt nhất đến nay')
        ax[0].set_ylim(max(J.min(), np.percentile(J, 10) - 2), J.max() + 0.5)
        ax[0].set_xlabel('lần thử (xám = thiết kế ban đầu, xanh = BO)')
        ax[0].set_ylabel('J train')
        ax[0].legend(fontsize=8)
        top = sorted(T, key=lambda t: -t['J'])[:10]
        U = np.array([ENV.params_to_action(t['params']) for t in top]) * 0.5 + 0.5
        cur = ENV.params_to_action({k: float(C.load()['dwb'][k]) for k in ENV.ACTION_KEYS}) * 0.5 + 0.5
        for k, u in enumerate(U):
            ax[1].plot(range(6), u, '-o', ms=3, lw=2.2 if k == 0 else 0.8, alpha=1 if k == 0 else 0.5,
                       color='#d62728' if k == 0 else '#1f77b4')
        ax[1].plot(range(6), cur, '--s', color='k', ms=4, lw=1, label='hiện tại')
        ax[1].set_xticks(range(6))
        lab = [f'{k.split("_")[0]}\n[{lo:g},{hi:g}]' for k, lo, hi in zip(ENV.ACTION_KEYS, ENV.ACTION_LOW,
                                                                           ENV.ACTION_HIGH)]
        ax[1].set_xticklabels(lab, fontsize=8)
        ax[1].set_ylabel('vị trí trong miền (0..1)')
        ax[1].set_title('top-10 theo J train (đỏ = cao nhất)', fontsize=9)
        ax[1].legend(fontsize=8)
        fig.suptitle(os.path.basename(os.path.normpath(d)))
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, f'tune_{os.path.basename(os.path.normpath(d))}.png'), dpi=110)
        plt.close(fig)
    if a.report:
        rep = json.load(open(a.report))
        for g in rep['groups']:
            T = rep['by_type'][g]
            sets = [s for s in rep['sets'] if s in rep['summary'][g]]
            types = [t for t in T if t != 'S1']
            if not types:
                continue
            fig, ax = plt.subplots(figsize=(1.0 + 0.9 * len(types), 3.6))
            w = 0.8 / len(sets)
            for k, s in enumerate(sets):
                v = [T[t].get(s, {}).get('SR') or 0 for t in types]
                ax.bar(np.arange(len(types)) + k * w - 0.4 + w / 2, v, w, label=s)
            ax.set_xticks(range(len(types)))
            ax.set_xticklabels(types)
            ax.set_ylim(max(0, min(T[t].get(s, {}).get('SR') or 100 for t in types for s in sets) - 10), 100.5)
            ax.set_ylabel('SR (%)')
            ax.set_title(f'{g}: SR theo type', fontsize=10)
            ax.legend(fontsize=7, ncol=len(sets))
            fig.tight_layout()
            fig.savefig(os.path.join(a.out, f'sr_by_type_{g}.png'), dpi=110)
            plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    e = sp.add_parser('episodes')
    e.add_argument('--data', default=os.path.expanduser('~/capstone_ws/bench_data'))
    e.add_argument('--split', default='')
    e.add_argument('--ids', default='')
    e.add_argument('--auto', default='')
    e.add_argument('--per', type=int, default=2)
    e.add_argument('--sets', default='')
    e.add_argument('--names', default='default')
    e.add_argument('--seed', type=int, default=0)
    e.add_argument('--out', required=True)
    s = sp.add_parser('summary')
    s.add_argument('--tune', required=True)
    s.add_argument('--report', default='')
    s.add_argument('--out', required=True)
    a = ap.parse_args()
    return episodes(a) if a.cmd == 'episodes' else summary(a)


if __name__ == '__main__':
    sys.exit(main())
