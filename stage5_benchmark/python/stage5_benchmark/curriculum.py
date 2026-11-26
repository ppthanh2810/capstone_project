"""Curriculum Scheduler + Stratified Sampling + Hybrid Generation cho SAC-DWA (Master Plan V4 §9.2). CHƯA TRAINING.

Ba giai đoạn ĐÃ CHỐT (không đổi): tỷ lệ Easy/Medium/Hard 80/20/0 -> 30/50/20 -> 33/34/33.
Chuyển giai đoạn: đủ `min_steps` bước của giai đoạn VÀ SR cuộn (cửa sổ `window` episode gần nhất của giai đoạn, chỉ
episode khả thi) >= `promote_sr`; hoặc quá `max_steps` (tránh kẹt). Không bao giờ lùi giai đoạn.

Stratified Sampling: mức ~ tỷ lệ giai đoạn -> type đều trong các type CÓ ở mức đó (bảng 7.2: B3 không Easy, C1 chỉ
Hard...) -> scenario ít được dùng nhất trong ô (type, mức) (vòng tròn có xáo trộn, không lặp lại sớm).

Hybrid: xác suất `offline_ratio` (0.70) lấy từ pool cố định (general_train / *_finetune), còn lại (0.30) từ
generator online. Online chạy nền (ProcessPool) đổ vào buffer theo ô (type, mức thật do difficulty() tính); ô trống
-> lấy offline và ĐẾM (`fallback`). Chống rò rỉ: seed namespace online riêng (ONLINE_INDEX·10^7, khác mọi split),
loại scenario có hash scenario / hash layout / ô (start, goal) 1 m trùng validation hoặc test.
"""

import collections
import math
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

STAGES = (
    {'name': 'C1', 'mix': {'easy': 0.80, 'medium': 0.20, 'hard': 0.00}},
    {'name': 'C2', 'mix': {'easy': 0.30, 'medium': 0.50, 'hard': 0.20}},
    {'name': 'C3', 'mix': {'easy': 0.33, 'medium': 0.34, 'hard': 0.33}},
)
LEVELS = ('easy', 'medium', 'hard')
ONLINE_INDEX = 40          # gen_scenarios.SPLIT_INDEX dùng 1..15


class CurriculumScheduler:
    def __init__(self, min_steps=(100_000, 150_000, 0), max_steps=(300_000, 400_000, None), promote_sr=0.85,
                 window=200):
        self.min_steps, self.max_steps = list(min_steps), list(max_steps)
        self.promote_sr, self.window = promote_sr, window
        self.stage = 0
        self.stage_steps = 0
        self.total_steps = 0
        self.hist = collections.deque(maxlen=window)
        self.log = []

    @property
    def mix(self):
        return STAGES[self.stage]['mix']

    def on_steps(self, n):
        self.stage_steps += n
        self.total_steps += n
        self._maybe_promote()

    def on_episode(self, label, feasible=True):
        if feasible and label not in ('SYSTEM_ERROR',):
            self.hist.append(label == 'SUCCESS')
        self._maybe_promote()

    def rolling_sr(self):
        return float(np.mean(self.hist)) if self.hist else 0.0

    def _maybe_promote(self):
        if self.stage >= len(STAGES) - 1:
            return
        mx = self.max_steps[self.stage]
        ready = (self.stage_steps >= self.min_steps[self.stage] and len(self.hist) >= self.window and
                 self.rolling_sr() >= self.promote_sr)
        forced = mx is not None and self.stage_steps >= mx
        if ready or forced:
            self.log.append({'from': STAGES[self.stage]['name'], 'to': STAGES[self.stage + 1]['name'],
                             'total_steps': self.total_steps, 'stage_steps': self.stage_steps,
                             'rolling_sr': self.rolling_sr(), 'forced': bool(forced and not ready)})
            self.stage += 1
            self.stage_steps = 0
            self.hist.clear()

    def state(self):
        return {'stage': STAGES[self.stage]['name'], 'mix': self.mix, 'stage_steps': self.stage_steps,
                'total_steps': self.total_steps, 'rolling_sr': self.rolling_sr(), 'promotions': list(self.log)}


class StratifiedPool:
    """Pool cố định chia ô (type, mức); lấy mẫu ít-dùng-nhất có xáo trộn."""

    def __init__(self, scenarios, rng):
        self.rng = rng
        self.cells = collections.defaultdict(list)
        for s in scenarios:
            if s['feasibility'] != 'feasible':
                continue            # S1 (vô nghiệm) không vào curriculum SAC; đánh giá SSR riêng
            self.cells[(s['type'], s['difficulty']['level'])].append(s)
        self.order = {k: list(rng.permutation(len(v))) for k, v in self.cells.items()}
        self.uses = collections.Counter()

    def types_at(self, level):
        return sorted(t for (t, lv), v in self.cells.items() if lv == level and v)

    def levels(self):
        return sorted({lv for (_, lv) in self.cells})

    def take(self, t, level):
        k = (t, level)
        if not self.order.get(k):
            self.order[k] = list(self.rng.permutation(len(self.cells[k])))
        i = self.order[k].pop()
        self.uses[k] += 1
        return self.cells[k][i]


def _online_job(args):
    stype, seed, hint, gen, nav = args
    from . import generator as GN
    sc, _ = GN.generate_one(stype, seed, gen, nav, hint=hint)
    if sc is None:
        return None
    lv = 'easy' if sc['difficulty']['score'] < gen['easy_max'] else (
        'medium' if sc['difficulty']['score'] < gen['medium_max'] else 'hard')
    sc['difficulty']['level'] = lv
    from .scenario_io import map_id_of
    sc['map_id'] = map_id_of(sc['map'])
    return sc


def reserved_keys(split_dirs):
    """Hash scenario, hash layout (map_id) và ô (start, goal) 1 m của các split cần bảo vệ (val/test)."""
    from .scenario_io import load_split
    hs, maps, sg = set(), set(), set()
    for d in split_dirs:
        for s in load_split(d, with_maps=False):
            hs.add(s['hash'])
            maps.add(s.get('map_id'))
            sg.add(sg_key(s))
    return hs, maps, sg


def sg_key(sc, cell=1.0):
    return (math.floor(sc['start'][0] / cell), math.floor(sc['start'][1] / cell),
            math.floor(sc['goal'][0] / cell), math.floor(sc['goal'][1] / cell))


class OnlineGenerator:
    """Sinh nền vào buffer theo ô; chỉ layout sinh (General), không dùng map mục tiêu (tránh rò rỉ ô start/goal)."""

    def __init__(self, gen, nav, reserved, types, workers=2, buffer_per_cell=8, seed_base=0):
        self.gen, self.nav = gen, nav
        self.res_hash, self.res_map, self.res_sg = reserved
        self.types = list(types)
        self.buf = collections.defaultdict(collections.deque)
        self.cap = buffer_per_cell
        self.k = 0
        self.seed_base = seed_base
        self.stats = collections.Counter()
        self.ex = ProcessPoolExecutor(max_workers=workers) if workers > 0 else None
        self.pending = []

    def _seed(self, t):
        from .generator import TYPES
        self.k += 1
        return ONLINE_INDEX * 10 ** 7 + self.seed_base * 10 ** 6 + TYPES.index(t) * 10 ** 5 + self.k

    def _accept(self, sc):
        if sc is None:
            self.stats['gen_failed'] += 1
            return
        if sc['hash'] in self.res_hash or sc['map_id'] in self.res_map:
            self.stats['leak_rejected'] += 1
            return
        k = (sc['type'], sc['difficulty']['level'])
        if len(self.buf[k]) < self.cap:
            sc['split'] = 'online'
            sc['id'] = f"online_{sc['type']}_{sc['difficulty']['level'][0]}_{sc['seed']}"
            self.buf[k].append(sc)
            self.stats['accepted'] += 1

    def pump(self, want=None):
        """Thu kết quả xong + nộp việc cho các ô còn thiếu (ưu tiên `want` = (type, mức))."""
        if self.ex is None:
            return
        done = [f for f in self.pending if f.done()]
        for f in done:
            self.pending.remove(f)
            try:
                self._accept(f.result())
            except Exception:  # noqa: BLE001
                self.stats['gen_error'] += 1
        while len(self.pending) < 2 * self.ex._max_workers:
            if want and len(self.buf[want]) < self.cap:
                t, lv = want
            else:
                cells = [(t, lv) for t in self.types for lv in LEVELS]
                t, lv = min(cells, key=lambda c: len(self.buf[c]))
            self.pending.append(self.ex.submit(_online_job, (t, self._seed(t), lv, self.gen, self.nav)))

    def take(self, t, level):
        self.pump((t, level))
        q = self.buf[(t, level)]
        return q.popleft() if q else None

    def close(self):
        if self.ex:
            self.ex.shutdown(wait=False, cancel_futures=True)


class HybridSampler:
    """Lấy scenario cho episode tiếp theo theo giai đoạn curriculum: 70 % offline / 30 % online."""

    def __init__(self, offline, scheduler, online=None, offline_ratio=0.70, rng_seed=0):
        self.rng = np.random.default_rng(rng_seed)
        self.pool = StratifiedPool(offline, self.rng)
        self.sched = scheduler
        self.online = online
        self.offline_ratio = offline_ratio
        self.stats = collections.Counter()

    def sample(self):
        mix = self.sched.mix
        levels = [lv for lv in LEVELS if mix[lv] > 0 and self.pool.types_at(lv)]
        p = np.array([mix[lv] for lv in levels])
        lv = levels[int(self.rng.choice(len(levels), p=p / p.sum()))]
        types = self.pool.types_at(lv)
        t = types[int(self.rng.integers(len(types)))]
        src = 'offline'
        sc = None
        if self.online is not None and self.rng.random() >= self.offline_ratio:
            sc = self.online.take(t, lv)
            src = 'online' if sc is not None else 'fallback'
        if sc is None:
            sc = self.pool.take(t, lv)
        self.stats[src] += 1
        self.stats[f'{lv}'] += 1
        self.stats[f'{t}:{lv}'] += 1
        return sc, {'source': src, 'type': t, 'level': lv, 'stage': STAGES[self.sched.stage]['name']}


def default_reserved(data_dir):
    names = ['general_val', 'general_test', 'general_s1_val', 'general_s1_test', 'classroom_val', 'classroom_test',
             'house_val', 'house_test']
    return reserved_keys([os.path.join(data_dir, n) for n in names if os.path.isdir(os.path.join(data_dir, n))])


__all__ = ['STAGES', 'CurriculumScheduler', 'StratifiedPool', 'OnlineGenerator', 'HybridSampler', 'reserved_keys',
           'default_reserved']
