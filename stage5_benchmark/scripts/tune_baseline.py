#!/usr/bin/env python3
"""Tối ưu DWA CỐ ĐỊNH (M0, Master Plan v5 bước 8) dưới Navigation V5 đã đóng băng.

Biến: [wP, wG, wO, wH, wS, T_sim] = [path_distance_scale, carrot_distance_scale, obstacle_scale, heading_scale,
speed_scale, sim_time], CÙNG miền action của SAC (env.ACTION_LOW/HIGH) để so sánh M0 ↔ M1 công bằng.

Phương pháp (v5, thay LHS + successive halving của v3):
  1. Thiết kế ban đầu: bộ hiện tại (nav2_params.yaml) + bộ --warm (nếu có) + n_init điểm Latin hypercube.
  2. Bayesian Optimization: Gaussian Process (Matern 5/2, ARD, nhiễu học được; siêu tham số tối đa log-likelihood
     biên, nhiều điểm khởi đầu) trên J chuẩn hóa, hàm thu Expected Improvement (incumbent = max trung bình hậu
     nghiệm tại các điểm đã chạy -> chịu nhiễu), tối đa EI bằng 20 000 điểm ngẫu nhiên + lân cận top-5 + L-BFGS-B.
     Tự viết bằng numpy/scipy (không cài thêm gói); tuần tự vì mỗi lần thử đã dùng hết lõi (song song theo episode).
  3. Mục tiêu trên TRAIN (cùng seed mọi lần thử -> so sánh ghép cặp):
        J = SR_bal - 0.5·STR + 3·q    (SR_bal = SR trung bình theo type, %; STR %; q điểm thời gian 0..1)
     CR > cr_max -> J - 20 (và không bao giờ được chọn). 3·q chỉ phân xử: q tăng 0.03 ~ 1 episode / 900.
  4. CHỌN trên VALIDATION (top-k theo J train + bộ hiện tại, nhiều seed), quy tắc từ điển định trước:
        (a) CR <= cr_max trên MỌI split trong --val và --cr-check;  (b) SR >= SR_max - sr_tol;
        (c) STR <= STR_min + str_tol;  (d) SSR >= SSR_max - ssr_tol;  (e) q lớn nhất.
  Test split bị khóa (dừng nếu tên split kết thúc bằng _test). Chạy freeze_v5.py --check trước.

  tune_baseline.py --train v3/general_train --val v3/general_val,v3/general_s1_val \
      --cr-check v3/classroom_val,v3/house_val --out DIR [--n-init 12 --n-iter 48 --topk 6 --seeds-val 0,1,2]
Đầu ra: trials.jsonl (mọi lần thử: tham số, J, metric train, metric theo type, GP), train_episodes.jsonl.gz,
val_episodes.jsonl.gz, report.json (validation, quy tắc chọn, chi phí, độ dài đặc trưng GP), best.yaml.
"""
import argparse
import collections
import gzip
import json
import os
import subprocess
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402
from scipy.optimize import minimize  # noqa: E402
from scipy.stats import norm  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import baseline_eval as BE  # noqa: E402
import baseline_report as BR  # noqa: E402
from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import env as ENV  # noqa: E402

KEYS = ENV.ACTION_KEYS
LOW, HIGH = ENV.ACTION_LOW, ENV.ACTION_HIGH
W_STR, W_Q, CR_PEN = 0.5, 3.0, 20.0


# ------------------------------------------------------------------ GP-BO (numpy/scipy)
def matern52(A, B, ls):
    d = np.sqrt(np.maximum(0.0, (((A[:, None, :] - B[None, :, :]) / ls) ** 2).sum(-1)))
    s5 = np.sqrt(5.0) * d
    return (1.0 + s5 + 5.0 / 3.0 * d ** 2) * np.exp(-s5)


class GP:
    def __init__(self, X, y, rng):
        self.X = X
        self.mu, self.sd = float(np.mean(y)), float(np.std(y) or 1.0)
        self.y = (y - self.mu) / self.sd
        d = X.shape[1]
        bounds = [(np.log(0.05), np.log(5.0))] * d + [(np.log(0.05), np.log(20.0)), (np.log(1e-4), np.log(1.0))]
        best = None
        for k in range(6):
            th0 = np.array([rng.uniform(lo, hi) for lo, hi in bounds]) if k else np.r_[np.zeros(d) + np.log(0.5),
                                                                                         0.0, np.log(0.05)]
            r = minimize(self._nll, th0, method='L-BFGS-B', bounds=bounds)
            if best is None or r.fun < best.fun:
                best = r
        self.theta = best.x
        self._fit(best.x)

    def _unpack(self, th):
        d = self.X.shape[1]
        return np.exp(th[:d]), np.exp(th[d]), np.exp(th[d + 1])

    def _nll(self, th):
        ls, s2, n2 = self._unpack(th)
        K = s2 * matern52(self.X, self.X, ls) + (n2 + 1e-8) * np.eye(len(self.X))
        try:
            L = np.linalg.cholesky(K)
        except np.linalg.LinAlgError:
            return 1e10
        a = np.linalg.solve(L.T, np.linalg.solve(L, self.y))
        return 0.5 * self.y @ a + np.log(np.diag(L)).sum()

    def _fit(self, th):
        self.ls, self.s2, self.n2 = self._unpack(th)
        K = self.s2 * matern52(self.X, self.X, self.ls) + (self.n2 + 1e-8) * np.eye(len(self.X))
        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, self.y))

    def predict(self, Xs):
        Ks = self.s2 * matern52(Xs, self.X, self.ls)
        m = Ks @ self.alpha
        v = np.linalg.solve(self.L, Ks.T)
        var = np.maximum(1e-12, self.s2 - (v ** 2).sum(0))
        return m, np.sqrt(var)          # thang chuẩn hóa


def propose(X, y, rng, xi=0.01):
    gp = GP(X, y, rng)
    m_obs, _ = gp.predict(X)
    inc = m_obs.max()

    def ei(Z):
        m, s = gp.predict(np.atleast_2d(Z))
        z = (m - inc - xi) / s
        return (m - inc - xi) * norm.cdf(z) + s * norm.pdf(z)

    cand = rng.random((20000, X.shape[1]))
    top = X[np.argsort(-y)[:5]]
    loc = np.clip(top[rng.integers(len(top), size=4000)] + rng.normal(0, 0.08, (4000, X.shape[1])), 0, 1)
    cand = np.vstack([cand, loc])
    e = np.concatenate([ei(cand[i:i + 4000]) for i in range(0, len(cand), 4000)])
    best_x, best_e = None, -1.0
    for x0 in cand[np.argsort(-e)[:5]]:
        r = minimize(lambda z: -ei(z)[0], x0, method='L-BFGS-B', bounds=[(0, 1)] * X.shape[1])
        if -r.fun > best_e and np.min(np.abs(X - r.x).max(1)) > 1e-3:
            best_x, best_e = r.x, -r.fun
    if best_x is None:
        best_x = cand[np.argmax(e)]
    return best_x, float(best_e), gp


def lhs(n, d, rng):
    return (rng.permuted(np.tile(np.arange(n), (d, 1)), axis=1).T + rng.random((n, d))) / n


def u_to_params(u):
    return {k: float(round(v, 4)) for k, v in zip(KEYS, LOW + np.asarray(u) * (HIGH - LOW))}


def params_to_u(p):
    return (np.array([p[k] for k in KEYS], float) - LOW) / (HIGH - LOW)


# ------------------------------------------------------------------ đánh giá
COST = collections.Counter()


def run_rows(pool, cfg, seeds, params, workers, split):
    jobs = [(sc, cfg, s, params) for sc in pool for s in seeds]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        rows = list(ex.map(BE._job, jobs, chunksize=2))
    for r in rows:
        r['split'] = split
    COST['episodes'] += len(rows)
    COST['core_s'] += sum(r['wall_s'] for r in rows)
    COST['sim_s'] += sum(r['sim_s'] for r in rows)
    return rows


def train_objective(rows, cr_max):
    m = BR.metrics(rows)
    by_t = collections.defaultdict(list)
    for r in rows:
        if r['feas'] == 'feasible' and r['label'] != 'SYSTEM_ERROR':
            by_t[r['type']].append(r['label'] == 'SUCCESS')
    m['SR_bal'] = float(np.mean([np.mean(v) for v in by_t.values()]) * 100)
    m['SR_by_type'] = {t: float(np.mean(v) * 100) for t, v in sorted(by_t.items())}
    J = m['SR_bal'] - W_STR * (m['STR'] or 0.0) + W_Q * (m.get('q') or 0.0)
    m['cr_ok'] = (m['CR'] or 0.0) <= cr_max
    m['J'] = J - (0.0 if m['cr_ok'] else CR_PEN)
    return m


def select(entries, cr_max, sr_tol, str_tol, ssr_tol):
    """Quy tắc từ điển (định trước khi chạy). entries: list {name, val:{SR, STR, SSR, q}, cr_ok_all}."""
    log = []
    pool = [e for e in entries if e['cr_ok_all']]
    log.append(f'(a) CR <= {cr_max} %: {len(pool)}/{len(entries)}')
    if not pool:
        return None, log
    sr_max = max(e['val']['SR'] for e in pool)
    pool = [e for e in pool if e['val']['SR'] >= sr_max - sr_tol]
    log.append(f'(b) SR >= {sr_max:.2f} - {sr_tol}: {[e["name"] for e in pool]}')
    str_min = min(e['val']['STR'] for e in pool)
    pool = [e for e in pool if e['val']['STR'] <= str_min + str_tol]
    log.append(f'(c) STR <= {str_min:.2f} + {str_tol}: {[e["name"] for e in pool]}')
    if any(e['val'].get('SSR') is not None for e in pool):
        ssr_max = max(e['val'].get('SSR') or 0.0 for e in pool)
        pool = [e for e in pool if (e['val'].get('SSR') or 0.0) >= ssr_max - ssr_tol]
        log.append(f'(d) SSR >= {ssr_max:.2f} - {ssr_tol}: {[e["name"] for e in pool]}')
    pool.sort(key=lambda e: -e['val']['q'])
    log.append(f'(e) q max: {pool[0]["name"]} (q {pool[0]["val"]["q"]:.4f})')
    return pool[0], log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=os.path.expanduser('~/capstone_ws/bench_data'))
    ap.add_argument('--train', required=True)
    ap.add_argument('--val', required=True, help='split chọn (gộp lại); SR/STR/SSR/q đo trên các split này')
    ap.add_argument('--cr-check', default='', help='split validation chỉ dùng kiểm tra CR <= cr_max')
    ap.add_argument('--out', required=True)
    ap.add_argument('--n-init', type=int, default=12)
    ap.add_argument('--n-iter', type=int, default=48)
    ap.add_argument('--seeds-train', default='0')
    ap.add_argument('--seeds-val', default='0,1,2')
    ap.add_argument('--topk', type=int, default=6)
    ap.add_argument('--warm', default='', help='file JSON {tên: params} thêm vào thiết kế ban đầu')
    ap.add_argument('--cr-max', type=float, default=2.0)
    ap.add_argument('--sr-tol', type=float, default=0.5)
    ap.add_argument('--str-tol', type=float, default=0.3)
    ap.add_argument('--ssr-tol', type=float, default=1.2)
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    t_start = time.time()
    names = [a.train] + a.val.split(',') + [x for x in a.cr_check.split(',') if x]
    if any(os.path.basename(os.path.normpath(s)).endswith('_test') for s in names):
        raise SystemExit('test split bị khóa: không dùng để tối ưu / chọn')
    r = subprocess.run([sys.executable, os.path.join(HERE, 'freeze_v5.py'), '--check'], capture_output=True, text=True)
    print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-300:], flush=True)
    if r.returncode:
        raise SystemExit('cấu hình V5 khác bản đóng băng')
    frozen = json.load(open(os.path.join(C.repo_root(), 'stage5_benchmark', 'config', 'navigation_v5_frozen.json')))
    os.makedirs(a.out, exist_ok=True)
    cfg = C.load()
    workers = a.workers or max(1, (os.cpu_count() or 2) - 1)
    rng = np.random.default_rng(a.seed)
    seeds_t = [int(s) for s in a.seeds_train.split(',')]
    seeds_v = [int(s) for s in a.seeds_val.split(',')]
    train = BE.load_pool(a.data, a.train)
    tname = os.path.basename(os.path.normpath(a.train))
    from stage5_benchmark import fast2d

    # ---------------- thiết kế ban đầu
    init = [('current', {k: float(cfg['dwb'][k]) for k in KEYS})]
    if a.warm:
        init += list(json.load(open(a.warm)).items())
    init += [(f'lhs{i}', u_to_params(u)) for i, u in enumerate(lhs(a.n_init, len(KEYS), rng))]
    trials = []
    tf = open(os.path.join(a.out, 'trials.jsonl'), 'w')
    ep_f = gzip.open(os.path.join(a.out, 'train_episodes.jsonl.gz'), 'wt')

    def do_trial(name, params, how, extra=None):
        why = fast2d.validate_dwb(dict(cfg['dwb'], **params))
        t0 = time.time()
        if why:
            rec = {'i': len(trials), 'name': name, 'how': how, 'params': params, 'invalid': why, 'J': None}
        else:
            rows = run_rows(train, cfg, seeds_t, params, workers, tname)
            for x in rows:
                x['set'] = f't{len(trials)}'
                ep_f.write(json.dumps(x, default=float) + '\n')
            ep_f.flush()
            m = train_objective(rows, a.cr_max)
            keep = ('SR', 'SR_bal', 'STR', 'TOR', 'CR', 'q', 'replan_rate', 'replan_per_ep', 'LAR', 'time_mean',
                    'path_mean', 'clr_min', 'clr_p05', 'SR_by_type', 'cr_ok', 'J')
            rec = {'i': len(trials), 'name': name, 'how': how, 'params': params,
                   'metrics': {k: m.get(k) for k in keep}, 'J': m['J'], 'wall_s': time.time() - t0}
        if extra:
            rec.update(extra)
        trials.append(rec)
        tf.write(json.dumps(rec, default=float) + '\n')
        tf.flush()
        mm = rec.get('metrics', {})
        print(f"[{rec['i']:3d}] {how:5s} {name:8s} J {rec['J'] if rec['J'] is None else round(rec['J'], 2)} "
              f"SRb {mm.get('SR_bal', 0):.1f} SR {mm.get('SR', 0):.1f} STR {mm.get('STR', 0):.1f} "
              f"CR {mm.get('CR', 0):.2f} q {mm.get('q', 0):.3f} rp {mm.get('replan_rate', 0):.0f}% "
              f"{time.time() - t0:.0f}s {params}", flush=True)

    for name, p in init:
        do_trial(name, p, 'init')
    best_gp = None
    for it in range(a.n_iter):
        ok = [t for t in trials if t['J'] is not None]
        X = np.array([params_to_u(t['params']) for t in ok])
        y = np.array([t['J'] for t in ok])
        u, e, gp = propose(X, y, rng)
        best_gp = gp
        do_trial(f'bo{it}', u_to_params(u), 'bo', {'ei': e, 'gp_ls': gp.ls.tolist(), 'gp_noise': float(gp.n2)})
    ep_f.close()
    t_train = time.time() - t_start
    ok = [t for t in trials if t['J'] is not None]
    if best_gp is None:
        best_gp = GP(np.array([params_to_u(t['params']) for t in ok]), np.array([t['J'] for t in ok]), rng)
    # độ dài đặc trưng cuối (thang [0,1] của miền): nhỏ = tham số quan trọng
    gp_info = {'lengthscales': dict(zip(KEYS, best_gp.ls.tolist())), 'signal_var': float(best_gp.s2),
               'noise_var': float(best_gp.n2), 'y_mean': best_gp.mu, 'y_std': best_gp.sd}
    pm, ps = best_gp.predict(np.array([params_to_u(t['params']) for t in ok]))
    for t, m_, s_ in zip(ok, pm, ps):
        t['gp_mean'] = float(m_ * best_gp.sd + best_gp.mu)

    # ---------------- validation (top-k theo J train + bộ hiện tại)
    ranked = sorted(ok, key=lambda t: -t['J'])
    finalists = ranked[:a.topk]
    if not any(t['name'] == 'current' for t in finalists):
        finalists.append(next(t for t in ok if t['name'] == 'current'))
    vals = {os.path.basename(os.path.normpath(s)): BE.load_pool(a.data, s) for s in a.val.split(',')}
    crs = {os.path.basename(os.path.normpath(s)): BE.load_pool(a.data, s) for s in a.cr_check.split(',') if s}
    vf = gzip.open(os.path.join(a.out, 'val_episodes.jsonl.gz'), 'wt')
    entries = []
    t_v0 = time.time()
    for t in finalists:
        nm = f"t{t['i']}" if t['name'] != 'current' else 'current'
        rows_sel, rows_cr = [], {}
        for sp, pool in list(vals.items()) + list(crs.items()):
            rows = run_rows(pool, cfg, seeds_v, t['params'], workers, sp)
            for x in rows:
                x['set'] = nm
                vf.write(json.dumps(x, default=float) + '\n')
            rows_cr[sp] = BR.metrics(rows)['CR'] or 0.0
            if sp in vals:
                rows_sel += rows
        vf.flush()
        m = BR.metrics(rows_sel)
        e = {'name': nm, 'trial': t['i'], 'params': t['params'], 'J_train': t['J'],
             'val': {k: m.get(k) for k in ('SR', 'STR', 'TOR', 'SSR', 'CR', 'q', 'replan_rate', 'LAR', 'time_mean',
                                           'path_mean', 'clr_min', 'SR_seed_std', 'SR_seed_range')},
             'cr_by_split': rows_cr, 'cr_ok_all': all(v <= a.cr_max for v in rows_cr.values())}
        entries.append(e)
        v = e['val']
        print(f"VAL {nm}: SR {BR.f1(v['SR'], 2)} STR {BR.f1(v['STR'], 2)} SSR {BR.f1(v['SSR'])} CR {rows_cr} "
              f"q {BR.f1(v['q'], 4)} rp {BR.f1(v['replan_rate'])} LAR {BR.f1(v['LAR'])}", flush=True)
    vf.close()
    best, log = select(entries, a.cr_max, a.sr_tol, a.str_tol, a.ssr_tol)
    for ln in log:
        print('SELECT', ln, flush=True)
    cost = dict(COST, wall_s=time.time() - t_start, wall_train_s=t_train, wall_val_s=time.time() - t_v0,
                core_h=COST['core_s'] / 3600, sim_h=COST['sim_s'] / 3600, trials=len(trials),
                trials_valid=len(ok), finalists=len(finalists), workers=workers,
                train_episodes_per_trial=len(train) * len(seeds_t),
                val_episodes_per_finalist=(sum(map(len, vals.values())) + sum(map(len, crs.values()))) * len(seeds_v))
    rep = {'method': 'GP-BO (Matern52 ARD, EI) + validation lexicographic selection', 'train': a.train,
           'val': a.val, 'cr_check': a.cr_check, 'seeds_train': seeds_t, 'seeds_val': seeds_v,
           'objective': f'J = SR_bal - {W_STR}*STR + {W_Q}*q ; CR>{a.cr_max} -> -{CR_PEN}',
           'rule': {'cr_max': a.cr_max, 'sr_tol': a.sr_tol, 'str_tol': a.str_tol, 'ssr_tol': a.ssr_tol},
           'domain': {k: [float(lo), float(hi)] for k, lo, hi in zip(KEYS, LOW, HIGH)},
           'frozen': {k: frozen[k] for k in ('git_commit', 'nav2_params_sha256', 'fast2d_module_sha256')},
           'gp': gp_info, 'trials_ranked': [{k: t.get(k) for k in ('i', 'name', 'how', 'params', 'J', 'gp_mean')}
                                            for t in ranked],
           'validation': entries, 'selection_log': log, 'best': best, 'cost': cost}
    json.dump(rep, open(os.path.join(a.out, 'report.json'), 'w'), indent=1, default=float, ensure_ascii=False)
    print('COST', json.dumps(cost, default=float), flush=True)
    if best:
        with open(os.path.join(a.out, 'best.yaml'), 'w') as f:
            f.write(f"# M0 best ({a.train} -> chọn trên {a.val}; KHÔNG dùng test). trial {best['trial']} "
                    f"({best['name']}), V5 nav2_params {frozen['nav2_params_sha256'][:12]}\n")
            for k in KEYS:
                f.write(f"{k}: {best['params'][k]:.4f}\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
