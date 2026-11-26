#!/usr/bin/env python3
"""Parameter Sensitivity + Ceiling Effect + Oracle Gap (Master Plan V4 §8.1) — Fast 2D, CHỈ train/development.

N bộ [wP, wG, wO, wH, wS, T_sim] (LHS trong miền action SAC + bộ hiện tại + các góc một-chiều) × một tập scenario
phân tầng (type × mức) × 2 seed. Seed A dùng để CHỌN (bộ cố định tốt nhất, oracle theo scenario), seed B để ĐO
(oracle chéo seed: loại phần "gap" chỉ do nhiễu seed).

Chỉ số mỗi episode: success (0/1) và điểm thời gian q = success · T_ref / max(T_ref, t), T_ref = L_A*(M) / 0.3
(1 = đi thẳng tốc độ tối đa; giống SPL theo thời gian).
V5 (2026-11-27) — Oracle Gap ĐÚNG ĐỊNH NGHĨA, cùng tập scenario và cùng các bộ tham số:
  J(θ, s)      = trung bình q (hoặc success) của bộ θ trên scenario s qua MỌI seed
  best fixed   = max_θ mean_s J(θ, s)   (ràng buộc CR <= 2 %)
  oracle       = mean_s max_θ J(θ, s)   (chọn bộ tốt nhất cho TỪNG scenario, θ chỉ trong các bộ đã thử)
  gap          = oracle − best fixed >= 0 (theo cấu trúc). Đây là cận trên dư địa cho chính sách chọn tham số theo
                 scenario (SAC chỉnh trong episode có thể vượt hoặc thấp hơn).
  gap_cross    = chọn trên seed A đo seed B và ngược lại, trung bình: cận dưới bảo thủ (loại phần gap chỉ do nhiễu
                 seed; v4 chỉ báo số này với 1 chiều nên âm).
  noise_floor  = gap tính được nếu các bộ chỉ khác nhau do nhiễu seed (hoán vị nhãn bộ trong từng scenario).
Ghi thêm theo bộ: tỷ lệ episode có replan, số replan (SAC đổi tham số có làm đổi replan không).
Nhóm phân biệt: tỷ lệ scenario mà các bộ cho kết cục khác nhau; độ lệch chuẩn SR giữa các bộ theo type/mức.

  sensitivity_study.py --data ~/capstone_ws/bench_data/v3 --split general_train --per-cell 5 --sets 30 \
      --out res.json [--seeds 0,1] [--workers 11]
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
from stage5_benchmark import env as ENV  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402


def lhs(n, d, rng):
    u = (rng.permuted(np.tile(np.arange(n), (d, 1)), axis=1).T + rng.random((n, d))) / n
    return 2 * u - 1


def stratified(pool, per_cell, rng):
    cells = collections.defaultdict(list)
    for s in pool:
        cells[(s['type'], s['difficulty']['level'])].append(s)
    out = []
    for k in sorted(cells):
        v = cells[k]
        idx = rng.permutation(len(v))[:per_cell]
        out += [v[i] for i in idx]
    return out


def param_sets(n_lhs, cfg, rng):
    cur = {k: float(cfg['dwb'][k]) for k in ENV.ACTION_KEYS}
    sets = {'current': cur}
    # góc một chiều: mỗi tham số ở biên dưới / trên của miền action, còn lại = hiện tại
    for i, k in enumerate(ENV.ACTION_KEYS):
        for side, v in (('lo', ENV.ACTION_LOW[i]), ('hi', ENV.ACTION_HIGH[i])):
            p = dict(cur)
            p[k] = float(v)
            sets[f'{k}_{side}'] = p
    for j, x in enumerate(lhs(n_lhs, len(ENV.ACTION_KEYS), rng)):
        sets[f'lhs{j:02d}'] = ENV.action_to_params(x)
    return sets


def q_score(e, sc):
    if e['label'] != 'SUCCESS':
        return 0.0
    t_ref = max(1.0, sc['l_ref'] / 0.3)
    return t_ref / max(t_ref, e['time'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--split', default='general_train')
    ap.add_argument('--per-cell', type=int, default=5)
    ap.add_argument('--sets', type=int, default=17, help='số bộ LHS (cộng 1 hiện tại + 12 góc)')
    ap.add_argument('--seeds', default='0,1')
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--cr-max', type=float, default=2.0)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', required=True)
    ap.add_argument('--sets-from', default='', help='JSON kết quả sensitivity trước: dùng lại đúng các bộ tham số')
    ap.add_argument('--replan', default='{}', help='JSON ghi đè goal_path_bridge.replan.* (so sánh cơ chế)')
    ap.add_argument('--all', action='store_true', help='dùng toàn bộ split (không lấy mẫu phân tầng)')
    a = ap.parse_args()
    if a.split.endswith('_test'):
        raise SystemExit('test split bị khóa')
    rng = np.random.default_rng(a.seed)
    cfg = C.load(overrides={'replan': json.loads(a.replan)})
    pool = []
    for sp in a.split.split(','):
        if sp.endswith('_test'):
            raise SystemExit('test split bị khóa')
        pool += IO.load_split(os.path.join(a.data, sp))
    scen = pool if a.all else stratified(pool, a.per_cell, rng)
    by = {s['id']: s for s in scen}
    if a.sets_from:     # dùng ĐÚNG các bộ của một lần chạy trước (so sánh hai tập scenario)
        sets = {k: v['params'] for k, v in json.load(open(a.sets_from))['per_set'].items()}
    else:
        sets = param_sets(a.sets, cfg, rng)
    seeds = [int(s) for s in a.seeds.split(',')]
    sa, sb = seeds[0], seeds[-1]
    res = {}   # set -> {(id, seed): compact}
    for name, p in sets.items():
        recs = RN.run_batch(scen, cfg, seeds=seeds, dwb_override=p, workers=a.workers or None)
        res[name] = {(r['scenario_id'], r['seed']): {'label': r['eval']['label'], 'time': r['eval']['time'],
                                                      'clr': r['min_clearance'], 'replans': r.get('replans', 0),
                                                      'orep': r.get('obstacle_replans', 0)}
                     for r in recs}
        print(name, 'SR', round(100 * np.mean([e['label'] == 'SUCCESS' for e in res[name].values()]), 1),
              'CR', round(100 * np.mean([e['label'] == 'COLLISION' for e in res[name].values()]), 2), flush=True)
    ids = sorted(by)
    names = list(sets)
    feas = [i for i in ids if by[i]['feasibility'] == 'feasible']

    def mat(seed, kind):
        f = (lambda e, i: float(e['label'] == 'SUCCESS')) if kind == 'succ' else (lambda e, i: q_score(e, by[i]))
        return np.array([[f(res[n][(i, seed)], i) for i in feas] for n in names])

    SA, SB = mat(sa, 'succ'), mat(sb, 'succ')
    QA, QB = mat(sa, 'q'), mat(sb, 'q')
    CR = np.array([np.mean([res[n][(i, s)]['label'] == 'COLLISION' for i in ids for s in seeds]) * 100
                   for n in names])
    ok = CR <= a.cr_max
    best = int(np.argmax(np.where(ok, QA.mean(1), -1)))
    best_sr = int(np.argmax(np.where(ok, SA.mean(1) + 1e-3 * QA.mean(1), -1)))
    # oracle theo scenario: chọn trên seed A (q, hòa -> success), đo trên seed B
    pick = np.argmax(QA + 1e-6 * SA, axis=0)
    cols = np.arange(len(feas))
    oracle = {'SR_cross': float(SB[pick, cols].mean() * 100), 'q_cross': float(QB[pick, cols].mean()),
              'SR_same': float(SA.max(0).mean() * 100), 'q_same': float(QA.max(0).mean()),
              'SR_any_set_any_seed': float(np.maximum(SA, SB).max(0).mean() * 100)}
    fixed = {'name': names[best], 'params': sets[names[best]], 'SR_B': float(SB[best].mean() * 100),
             'q_B': float(QB[best].mean()), 'SR_A': float(SA[best].mean() * 100), 'q_A': float(QA[best].mean()),
             'best_by_SR': names[best_sr], 'SR_B_best_by_SR': float(SB[best_sr].mean() * 100)}
    cur = names.index('current')
    current = {'SR_A': float(SA[cur].mean() * 100), 'SR_B': float(SB[cur].mean() * 100),
               'q_A': float(QA[cur].mean()), 'q_B': float(QB[cur].mean())}
    # nhiễu seed: cùng bộ, khác seed -> khác kết cục
    seed_flip = float(np.mean(SA != SB) * 100)
    # nhóm
    groups = {}
    for key in ('type', 'level'):
        g = collections.defaultdict(list)
        for c, i in enumerate(feas):
            g[by[i]['type'] if key == 'type' else by[i]['difficulty']['level']].append(c)
        for gname, cs in sorted(g.items()):
            cs = np.array(cs)
            sr_sets = SA[:, cs].mean(1) * 100
            q_sets = QA[:, cs].mean(1)
            disc = float(np.mean([len(set(SA[:, c])) > 1 for c in cs]) * 100)
            groups[f'{key}:{gname}'] = {
                'n': int(len(cs)), 'SR_current': float(sr_sets[cur]), 'SR_best_fixed': float(SB[best, cs].mean() * 100),
                'SR_min_set': float(sr_sets.min()), 'SR_max_set': float(sr_sets.max()), 'SR_std_sets': float(sr_sets.std()),
                'q_current': float(q_sets[cur]), 'q_best_fixed_B': float(QB[best, cs].mean()),
                'q_oracle_cross': float(QB[pick[cs], cs].mean()), 'q_std_sets': float(q_sets.std()),
                'discriminative_pct': disc,
                'oracle_gap_q': float(QB[pick[cs], cs].mean() - QB[best, cs].mean()),
                'oracle_gap_SR': float((SB[pick[cs], cs].mean() - SB[best, cs].mean()) * 100)}
    # độ khó: tương quan Dscore với tỷ lệ thất bại trung bình qua các bộ
    from scipy.stats import spearmanr
    dscore = np.array([by[i]['difficulty']['score'] for i in feas])
    fail = 1 - (SA.mean(0) + SB.mean(0)) / 2
    qmean = (QA.mean(0) + QB.mean(0)) / 2
    rho_f = spearmanr(dscore, fail)
    rho_q = spearmanr(dscore, qmean)
    per_set = {n: {'params': sets[n], 'SR_A': float(SA[k].mean() * 100), 'SR_B': float(SB[k].mean() * 100),
                   'q_A': float(QA[k].mean()), 'q_B': float(QB[k].mean()), 'CR': float(CR[k]),
                   'STR': float(np.mean([res[n][(i, s)]['label'] == 'STUCK' for i in feas for s in seeds]) * 100)}
               for k, n in enumerate(names)}
    # ---- V5: Oracle Gap đúng định nghĩa (cùng dữ liệu, J = trung bình qua seed) ----
    QM, SM = (QA + QB) / 2, (SA + SB) / 2
    okq = np.where(ok[:, None], QM, -1)
    oks = np.where(ok[:, None], SM, -1)
    bf_q = int(np.argmax(np.where(ok, QM.mean(1), -1)))
    bf_s = int(np.argmax(np.where(ok, SM.mean(1) + 1e-3 * QM.mean(1), -1)))
    gap_def = {
        'best_fixed_q': names[bf_q], 'q_best_fixed': float(QM[bf_q].mean()), 'q_oracle': float(okq.max(0).mean()),
        'gap_q': float(okq.max(0).mean() - QM[bf_q].mean()),
        'best_fixed_sr': names[bf_s], 'SR_best_fixed': float(SM[bf_s].mean() * 100),
        'SR_oracle': float(oks.max(0).mean() * 100), 'gap_SR': float((oks.max(0).mean() - SM[bf_s].mean()) * 100),
        'q_current': float(QM[cur].mean()), 'SR_current': float(SM[cur].mean() * 100)}
    # cận dưới chéo seed (hai chiều)
    pa = np.argmax(np.where(ok[:, None], QA + 1e-6 * SA, -1), axis=0)
    pb = np.argmax(np.where(ok[:, None], QB + 1e-6 * SB, -1), axis=0)
    ba = int(np.argmax(np.where(ok, QA.mean(1), -1)))
    bb = int(np.argmax(np.where(ok, QB.mean(1), -1)))
    gap_def['gap_q_cross'] = float(((QB[pa, cols].mean() - QB[ba].mean()) + (QA[pb, cols].mean() - QA[bb].mean())) / 2)
    gap_def['gap_SR_cross'] = float(((SB[pa, cols].mean() - SB[ba].mean()) + (SA[pb, cols].mean() - SA[bb].mean())) / 2 * 100)
    # sàn nhiễu: hoán vị ngẫu nhiên nhãn (A, B) trong từng (bộ, scenario) không đổi kỳ vọng -> gap do nhiễu seed
    # ước lượng bằng gap của ma trận "chỉ nhiễu": thay mỗi J(θ,s) bằng trung bình qua θ + chênh A−B của θ.
    rng2 = np.random.default_rng(0)
    floors = []
    base_q = QM.mean(0, keepdims=True)
    for _ in range(20):
        sign = rng2.choice([-1.0, 1.0], size=QA.shape)
        Qn = base_q + sign * (QA - QB) / 2
        Qn = np.where(ok[:, None], Qn, -1)
        floors.append(Qn.max(0).mean() - Qn[int(np.argmax(np.where(ok, Qn.mean(1), -1)))].mean())
    gap_def['noise_floor_q'] = float(np.mean(floors))
    # replan theo bộ (SAC đổi tham số -> đổi replan?)
    for k, n in enumerate(names):
        per_ep = [res[n][(i, s)] for i in feas for s in seeds]
        per_set_rep = {'replan_ep_pct': float(np.mean([e['orep'] > 0 for e in per_ep]) * 100),
                       'obstacle_replans_mean': float(np.mean([e['orep'] for e in per_ep])),
                       'replans_mean': float(np.mean([e['replans'] for e in per_ep]))}
        gap_def.setdefault('replans_by_set', {})[n] = per_set_rep
    out = {'oracle_gap_v5': gap_def, 'replan': json.loads(a.replan), 'split': a.split, 'n_scenarios': len(ids), 'n_feasible': len(feas), 'seeds': seeds, 'sets': len(names),
           'episodes': len(ids) * len(seeds) * len(names), 'current': current, 'best_fixed': fixed,
           'oracle': oracle, 'oracle_gap': {'SR': oracle['SR_cross'] - fixed['SR_B'], 'q': oracle['q_cross'] - fixed['q_B']},
           'seed_flip_pct': seed_flip,
           'ceiling': {'sets_SR_ge_95': int(np.sum(SA.mean(1) >= 0.95)), 'scenarios_all_sets_succeed_pct':
                       float(np.mean(SA.min(0) == 1) * 100), 'scenarios_no_set_succeeds_pct':
                       float(np.mean(np.maximum(SA, SB).max(0) == 0) * 100)},
           'difficulty_spearman': {'dscore_vs_fail': [float(rho_f[0]), float(rho_f[1])],
                                   'dscore_vs_q': [float(rho_q[0]), float(rho_q[1])]},
           'groups': groups, 'per_set': per_set, 'scenario_ids': ids}
    json.dump(out, open(a.out, 'w'), indent=1, default=float)
    print(json.dumps({k: out[k] for k in ('current', 'best_fixed', 'oracle', 'oracle_gap', 'seed_flip_pct',
                                          'ceiling', 'difficulty_spearman')}, indent=1, default=float))
    print(json.dumps({k: v for k, v in gap_def.items() if k != 'replans_by_set'}, indent=1, default=float))
    return 0


if __name__ == '__main__':
    sys.exit(main())
