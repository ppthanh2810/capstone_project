"""Chạy scenario trên Fast 2D (song song nhiều tiến trình), trả episode record + nhãn evaluator.

Scenario lưu theo manifest (scenario_io); map M lưu .npz cạnh manifest. Cùng record/evaluator với Gazebo.
"""

import math
import os
import time
import traceback
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from . import config as C
from . import evaluator as E


def scenario_to_fast2d(sc):
    M = sc['map']
    data = M.data if hasattr(M, 'data') else M['data']
    origin = M.origin if hasattr(M, 'origin') else M['origin']
    res = M.res if hasattr(M, 'res') else M['resolution']
    return {'map': {'origin': list(origin), 'resolution': res, 'data': data}, 'obstacles': sc['obstacles'],
            'start': sc['start'], 'goal': sc['goal']}


def run_episode(sc, cfg, seed=0, dwb_override=None, sim_override=None, keep_trace=True, controller=None):
    """Một episode Fast 2D. controller(ep) -> None: callback mỗi 0.1 s (M1/SAC đổi tham số); None = M0."""
    from . import fast2d
    sim = C.sim_for_fast2d(cfg['sim'])
    sim['seed'] = float(seed)
    sim['timeout'] = float(sc.get('timeout', 120.0))
    sim['follow_timeout'] = max(sim.get('follow_timeout', 120.0), sim['timeout'] + 5.0)
    if sim_override:
        sim.update({k: float(v) for k, v in sim_override.items()})
    dwb = dict(cfg['dwb'])
    if dwb_override:
        dwb.update(dwb_override)
    t0 = time.time()
    rec = {'sim': 'fast2d', 'feasibility': sc['feasibility'], 'goal': sc['goal'], 'timeout': sim['timeout'],
           'scenario_id': sc.get('id', ''), 'seed': seed}
    try:
        ep = fast2d.Episode(scenario_to_fast2d(sc), sim, dwb, cfg['replan'])
        if controller is None:
            ep.run()
        else:
            while not ep.done:
                controller(ep)
                ep.step(0.1)
        r = ep.result()
    except Exception as e:  # noqa: BLE001 - lỗi mô phỏng = SYSTEM_ERROR, không làm hỏng cả lô
        rec.update({'terminal_state': 'none', 'infra_error': f'exception: {e}', 'trace': np.zeros((0, 6)),
                    'min_clearance': math.inf, 'wall_s': time.time() - t0, 'tb': traceback.format_exc()[-500:]})
        rec['eval'] = E.evaluate(rec, cfg['eval'])
        return rec
    warm = sim.get('warmup', 1.0)
    tr = r['trace']
    if len(tr):
        tr = tr.copy()
        tr[:, 0] -= warm
    rec.update({
        'terminal_state': r['terminal_state'],
        't_terminal': r['t_terminal'] - warm if r['t_terminal'] >= 0 else -1.0,
        'timed_out': r['timeout'], 'trace': tr if keep_trace else tr[:, :6],
        'min_clearance': r['min_clearance'], 'infra_error': r['error'],
        'replans': r['replans'], 'obstacle_replans': r['obstacle_replans'], 'abort_replans': r['abort_replans'],
        'max_goal_stagnation': r['max_goal_stagnation'],
        'initial_path_length': r['initial_path_length'], 'plan_lengths': r['plan_lengths'],
        'controller_cycles': r['controller_cycles'], 'cycles_no_valid': r['cycles_no_valid'],
        'cycles_holding': r['cycles_holding'], 'rejects': r['rejects'], 'candidates': r['candidates'],
        'compute_ms_mean': r['compute_ms_mean'], 'compute_ms_p99': r['compute_ms_p99'],
        'compute_ms_max': r['compute_ms_max'], 'missed_deadlines': r['missed_deadlines'],
        'stale_cycles': r['stale_cycles'], 'events': [(round(t - warm, 2), s) for t, s in r['events']],
        'wall_s': time.time() - t0, 'sim_s': r['t_end'] - warm,
        'plans': [(round(t - warm, 2), a) for t, a in r.get('plans', [])] if keep_trace else [],
        'trace_columns': r.get('trace_columns'),
    })
    rec['eval'] = E.evaluate(rec, cfg['eval'])
    return rec


def _worker(args):
    sc, cfg, seed, dwb_o, sim_o = args
    rec = run_episode(sc, cfg, seed, dwb_o, sim_o)
    rec.pop('trace', None)      # không gửi trace lớn qua tiến trình
    rec.pop('plans', None)
    return rec


def run_batch(scenarios, cfg, seeds=(0,), dwb_override=None, sim_override=None, workers=None):
    jobs = [(sc, cfg, s, dwb_override, sim_override) for sc in scenarios for s in seeds]
    workers = workers or max(1, (os.cpu_count() or 2) - 2)
    out = []
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for rec in ex.map(_worker, jobs, chunksize=1):
            out.append(rec)
    return out


def rows_for_summary(recs, scen_by_id):
    rows = []
    for r in recs:
        sc = scen_by_id.get(r['scenario_id'], {})
        rows.append({'label': r['eval']['label'], 'feasibility': r['feasibility'], 'type': sc.get('type'),
                     'level': sc.get('difficulty', {}).get('level'), 'scenario_id': r['scenario_id']})
    return rows
