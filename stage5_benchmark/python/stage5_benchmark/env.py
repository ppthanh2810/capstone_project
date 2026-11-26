"""Môi trường kiểu Gymnasium cho SAC-DWA (M1) trên Fast 2D — CHUẨN BỊ, CHƯA TRAINING (Master Plan v3 Giai đoạn D).

API giống gymnasium.Env (không phụ thuộc gymnasium): reset(seed, options) -> (obs, info); step(action) ->
(obs, reward, terminated, truncated, info). Action ∈ [-1, 1]^6 -> [wP, wG, wO, wH, wS, T_sim] (miền ĐỀ XUẤT, xem
docs); policy giữ action trong decision_period giây (mặc định 0.5 s = 5 chu kỳ DWA). An toàn KHÔNG phụ thuộc action:
DwaCore vẫn loại cứng va chạm / phanh / unknown; validateParams từ chối action lỗi (giữ tham số cũ, info['param_error']).

Observation chỉ gồm dữ liệu robot thật có (LiDAR, costmap cục bộ, đường A*, odometry, trạng thái bridge/DWA);
ground truth W chỉ dùng cho reward (khoảng hở) và evaluator.
"""

import math

import numpy as np

from . import config as C
from . import evaluator as E
from . import runner as RN

ACTION_KEYS = ['path_distance_scale', 'carrot_distance_scale', 'obstacle_scale', 'heading_scale', 'speed_scale',
               'sim_time']
ACTION_LOW = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 1.5])
ACTION_HIGH = np.array([2.0, 4.0, 8.0, 1.5, 3.0, 3.0])
PHASES = ['IDLE', 'FOLLOW_PATH', 'ROTATE_TO_PATH', 'ROTATE_TO_GOAL', 'BLOCKED']
N_SECTORS = 36


def action_to_params(a, low=ACTION_LOW, high=ACTION_HIGH):
    a = np.clip(np.asarray(a, float), -1.0, 1.0)
    v = low + (a + 1.0) * 0.5 * (high - low)
    return {k: float(x) for k, x in zip(ACTION_KEYS, v)}


def params_to_action(p, low=ACTION_LOW, high=ACTION_HIGH):
    v = np.array([p[k] for k in ACTION_KEYS], float)
    return 2.0 * (v - low) / (high - low) - 1.0


class SacDwaEnv:
    def __init__(self, scenarios, cfg=None, decision_period=0.5, sector_range=3.0, reward=None, rng_seed=0,
                 action_low=None, action_high=None):
        self.scenarios = scenarios
        self.cfg = cfg or C.load()
        self.dp = decision_period
        self.sector_range = sector_range
        self.low = np.array(action_low if action_low is not None else ACTION_LOW, float)
        self.high = np.array(action_high if action_high is not None else ACTION_HIGH, float)
        self.rw = dict(progress=10.0, time=-0.05, success=20.0, collision=-30.0, stuck=-15.0, clearance=-0.5,
                       clearance_ref=0.10, smooth=-0.02)
        if reward:
            self.rw.update(reward)
        self.rng = np.random.default_rng(rng_seed)
        self.ep = None
        self.obs_dim = N_SECTORS + 6 + 4 + len(PHASES) + 6 + 5
        self.act_dim = len(ACTION_KEYS)

    # -------------------------------------------------------------- observation
    def _obs(self):
        ep = self.ep
        scan = np.asarray(ep.scan(), float)
        n = len(scan)
        ang = np.arange(n) * 2 * math.pi / n
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        front = np.abs(ang) < math.pi / 2
        r = np.where(np.isfinite(scan), scan, self.sector_range)
        r = np.clip(r, 0.0, self.sector_range)
        sect = np.full(N_SECTORS, self.sector_range)
        idx = ((ang[front] + math.pi / 2) / math.pi * N_SECTORS).astype(int).clip(0, N_SECTORS - 1)
        np.minimum.at(sect, idx, r[front])
        x, y, th = ep.pose()
        c = ep.last_control()
        cx, cy = c['carrot'][0] - x, c['carrot'][1] - y
        ca, sa = math.cos(-th), math.sin(-th)
        carrot_r = (ca * cx - sa * cy, sa * cx + ca * cy)
        gx, gy, gth = self.sc['goal']
        dg = math.hypot(gx - x, gy - y)
        v, w = ep.measured_velocity()
        lv, lw = ep.last_command()
        phase = np.zeros(len(PHASES))
        if c['phase'] in PHASES:
            phase[PHASES.index(c['phase'])] = 1.0
        nc = max(1, c['n_candidates'])
        flags = [float(ep.blocked), float(c['carrot_blocked']), c['n_valid'] / nc, c['n_forward_valid'] / nc,
                 min(1.0, ep.result()['replans'] / 5.0) if ep.done else 0.0]
        o = np.concatenate([
            sect / self.sector_range,
            [carrot_r[0] / 1.0, carrot_r[1] / 1.0, math.tanh(dg / 5.0), c['heading_error'] / math.pi,
             c['carrot_shift'], ep.progress / max(1.0, self.sc['l_ref'])],
            [v / 0.3, w / 1.0, lv / 0.3, lw / 1.0],
            phase,
            self.last_action,
            flags,
        ]).astype(np.float32)
        return o

    # -------------------------------------------------------------- API
    def reset(self, seed=None, options=None):
        from . import fast2d
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        self.sc = options.get('scenario') or self.scenarios[int(self.rng.integers(len(self.scenarios)))]
        sim = C.sim_for_fast2d(self.cfg['sim'])
        sim['seed'] = float(self.rng.integers(1 << 30))
        sim['timeout'] = float(self.sc.get('timeout', 120.0))
        sim['follow_timeout'] = sim['timeout'] + 5.0
        dwb = dict(self.cfg['dwb'])
        init = options.get('init_params')
        if init:
            dwb.update(init)
        self.ep = fast2d.Episode(RN.scenario_to_fast2d(self.sc), sim, dwb, self.cfg['replan'])
        self.ep.step(sim.get('warmup', 1.0) + 0.1)
        self.last_action = params_to_action({k: dwb[k] for k in ACTION_KEYS}, self.low, self.high)
        self.prev_progress = 0.0
        return self._obs(), {'scenario_id': self.sc.get('id')}

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        params = action_to_params(a, self.low, self.high)
        err = self.ep.set_dwb(params)
        r_smooth = self.rw['smooth'] * float(np.sum((a - self.last_action) ** 2))
        self.last_action = a if not err else self.last_action
        self.ep.step(self.dp)
        prog = self.ep.progress
        clr = self.ep.gt_clearance()
        r = self.rw['progress'] * max(-0.5, prog - self.prev_progress) + self.rw['time'] + r_smooth
        if clr < self.rw['clearance_ref']:
            r += self.rw['clearance'] * (self.rw['clearance_ref'] - max(clr, 0.0)) / self.rw['clearance_ref']
        self.prev_progress = prog
        terminated = truncated = False
        info = {'param_error': err}
        if self.ep.done:
            rec = self._record()
            lab = rec['eval']['label']
            info['label'] = lab
            info['record'] = rec
            r += {'SUCCESS': self.rw['success'], 'COLLISION': self.rw['collision'],
                  'STUCK': self.rw['stuck']}.get(lab, 0.0)
            terminated = lab in ('SUCCESS', 'COLLISION', 'STUCK', 'SAFE_STOP')
            truncated = not terminated
        return self._obs(), float(r), terminated, truncated, info

    def _record(self):
        r = self.ep.result()
        warm = self.cfg['sim'].get('warmup', 1.0)
        tr = r['trace'].copy()
        if len(tr):
            tr[:, 0] -= warm
        rec = {'sim': 'fast2d', 'feasibility': self.sc['feasibility'], 'goal': self.sc['goal'],
               'terminal_state': r['terminal_state'],
               't_terminal': r['t_terminal'] - warm if r['t_terminal'] >= 0 else -1.0,
               'timed_out': r['timeout'], 'trace': tr, 'min_clearance': r['min_clearance'], 'infra_error': r['error'],
               'replans': r['replans']}
        rec['eval'] = E.evaluate(rec, self.cfg['eval'])
        return rec
