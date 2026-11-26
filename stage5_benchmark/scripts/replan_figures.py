#!/usr/bin/env python3
"""Hình minh họa cơ chế Replanning (Master Plan V5 §3.6): mỗi scenario một hình, mỗi cơ chế một cột.

Hàng trên: map M (xám), vật cản mới Δ (đỏ, KHÔNG có trong map của robot), Global Path ban đầu (xanh đứt), các Global
Path sau replan (cam), quỹ đạo GT của robot (đen), vị trí kích hoạt replan (sao, ghi nguyên nhân), start/goal.
Hàng dưới: trạng thái DWA theo thời gian — tỷ lệ cung tiến tự do (fs_free_ratio), khoảng hở fs_clearance,
tỷ lệ quỹ đạo tiến hợp lệ n_forward/n_candidates (phụ thuộc tham số DWA), v đo, trạng thái ReplanSupervisor
(0 NORMAL, 1 PATH_BLOCKED, 2 LOCAL_BLOCKED, 3 GLOBAL_BLOCKED), vạch dọc = replan.

  replan_figures.py --data ~/capstone_ws/bench_data/v3 --split general_val --ids a,b --mechs R1,R2,R3 --out dir
"""
import argparse
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replan_compare import MECHS, SETS, classify  # noqa: E402
from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def draw_shape(ax, sh):
    if sh['type'] == 'circle':
        ax.add_patch(plt.Circle((sh['x'], sh['y']), sh['r'], color='#d62728', alpha=0.8))
        return
    c, s = math.cos(sh['yaw']), math.sin(sh['yaw'])
    pts = []
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        lx, ly = dx * sh['sx'] / 2, dy * sh['sy'] / 2
        pts.append((sh['x'] + c * lx - s * ly, sh['y'] + s * lx + c * ly))
    ax.add_patch(plt.Polygon(pts, color='#d62728', alpha=0.8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--split', required=True)
    ap.add_argument('--ids', required=True)
    ap.add_argument('--mechs', default='R1,R2,R3')
    ap.add_argument('--mech-json', default='{}')
    ap.add_argument('--set', default='default')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    if a.split.endswith('_test'):
        raise SystemExit('test split bị khóa')
    mechs = dict(MECHS)
    mechs.update(json.loads(a.mech_json))
    os.makedirs(a.out, exist_ok=True)
    pool = {s['id']: s for s in IO.load_split(os.path.join(a.data, a.split))}
    dwb_o = SETS.get(a.set, {})
    ms = a.mechs.split(',')
    for sid in a.ids.split(','):
        sc = pool[sid]
        recs = {}
        for m in ms:
            cfg = C.load(overrides={'replan': mechs[m]})
            recs[m] = RN.run_episode(sc, cfg, a.seed, dwb_o)
        fig, axes = plt.subplots(2, len(ms), figsize=(5.2 * len(ms), 8.6),
                                 gridspec_kw={'height_ratios': [2.2, 1]}, squeeze=False)
        M = sc['map']
        data = np.asarray(M.data)
        ox, oy = M.origin[0], M.origin[1]
        res = M.res
        ext = [ox, ox + data.shape[1] * res, oy, oy + data.shape[0] * res]
        img = np.where(data >= 50, 0.15, np.where(data < 0, 0.6, 1.0))
        # vùng quan tâm: bao quỹ đạo + đường
        allxy = [np.asarray(r['trace'])[:, 1:3] for r in recs.values() if len(r['trace'])]
        allxy += [np.asarray(p[1]) for r in recs.values() for p in r.get('plans', [])]
        allxy = np.vstack(allxy) if allxy else np.zeros((1, 2))
        lo = allxy.min(0) - 1.0
        hi = allxy.max(0) + 1.0
        for j, m in enumerate(ms):
            r = recs[m]
            ax = axes[0, j]
            ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
            for sh in sc.get('obstacles') or []:
                draw_shape(ax, sh)
            plans = r.get('plans', [])
            for k, (t, P) in enumerate(plans):
                P = np.asarray(P)
                if k == 0:
                    ax.plot(P[:, 0], P[:, 1], '--', color='#1f77b4', lw=1.6, label='A* ban đầu')
                else:
                    ax.plot(P[:, 0], P[:, 1], '-', color='#ff7f0e', lw=1.2, alpha=0.8,
                            label='A* sau replan' if k == 1 else None)
            tr = np.asarray(r['trace'])
            cols = r['trace_columns']
            ix = {c: i for i, c in enumerate(cols)}
            if len(tr):
                ax.plot(tr[:, 1], tr[:, 2], '-', color='k', lw=1.4, label='robot (GT)')
            ev = [(t, s[len('REPLANNING: '):]) for t, s in r['events'] if s.startswith('REPLANNING: ')]
            for t, why in ev:
                k = int(np.argmin(np.abs(tr[:, 0] - t)))
                ax.plot(tr[k, 1], tr[k, 2], '*', color='#9467bd', ms=14, mec='k')
                ax.annotate(classify(why), (tr[k, 1], tr[k, 2]), fontsize=7, xytext=(4, 4),
                            textcoords='offset points')
            ax.plot(*sc['start'][:2], 'o', color='#2ca02c', ms=8)
            ax.plot(*sc['goal'][:2], 's', color='#2ca02c', ms=8)
            ax.set_xlim(lo[0], hi[0])
            ax.set_ylim(lo[1], hi[1])
            ax.set_aspect('equal')
            e = r['eval']
            segs = ''
            ax.set_title(f"{m}: {e['label']} {e['time']:.1f} s, replan {r['replans']}, hở {r['min_clearance']:.3f} m{segs}",
                         fontsize=9)
            if j == 0:
                ax.legend(fontsize=7, loc='best')
            # ---- trạng thái DWA ----
            bx = axes[1, j]
            if len(tr):
                t = tr[:, 0]
                bx.plot(t, tr[:, ix['fs_free_ratio']], color='#2ca02c', lw=1, label='cung tự do (fs)')
                bx.plot(t, np.clip(tr[:, ix['fs_clearance']], 0, 1), color='#8c564b', lw=1, label='fs_clearance m')
                nc = np.maximum(1, tr[:, ix['n_candidates']])
                bx.plot(t, tr[:, ix['n_forward']] / nc, color='#1f77b4', lw=0.8, alpha=0.7, label='n_fwd/n_cand')
                bx.plot(t, tr[:, ix['v']] / 0.3, color='k', lw=0.8, alpha=0.7, label='v / 0.3')
                bx.step(t, tr[:, ix['sup_state']] / 3.0, color='#9467bd', lw=1.2, where='post',
                        label='trạng thái / 3')
                for tt, _ in ev:
                    bx.axvline(tt, color='#9467bd', ls=':', lw=1)
                bx.set_ylim(-0.05, 1.1)
                bx.set_xlabel('t (s)')
                if j == 0:
                    bx.legend(fontsize=6, loc='upper right', ncol=2)
        fig.suptitle(f"{sid} ({sc['type']}, {sc['difficulty']['level']}), bộ {a.set}, seed {a.seed}", fontsize=11)
        fig.tight_layout()
        p = os.path.join(a.out, f'{sid}_{a.set}.png')
        fig.savefig(p, dpi=110)
        plt.close(fig)
        print(p, {m: (recs[m]['eval']['label'], recs[m]['replans']) for m in ms}, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
