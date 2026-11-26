"""Scenario Generator — Rule-based + BARN-inspired (Master Plan v3 §4–§5).

Một scenario = layout/map M + vật cản mới Δ (chỉ trong world W = M ⊕ Δ) + start/goal + nhãn type/difficulty/
feasibility. A* ban đầu dùng M (NavFn thật qua fast2d); ground truth W chỉ dùng cho kiểm tra khả thi, độ khó,
LiDAR và evaluator — không bao giờ đưa vào DWA/A*/SAC.

Kiểm tra (5.3):
  - start/goal hở >= start_goal_clearance trong W; có A* trên M (NavFn, global costmap static);
  - khả thi hình học: W nở feasible_inflate (0.30) còn nối -> FEASIBLE; W nở unsolvable_inflate (0.20) mất nối ->
    UNSOLVABLE; còn lại (chỉ đi được qua khe 0.40–0.60) -> loại (vùng mơ hồ);
  - đường tắt mơ hồ: L(r 0.20) < L(r 0.30) − 0.5 m -> loại (A* có thể chọn khe 0.40–0.60);
  - khe mép-mép quanh mỗi vật Δ phải >= gap_feasible_min hoặc <= gap_unsolvable_max;
  - vật Δ cách start >= new_obstacle_min_from_start (dọc đường M và Euclid tới LiDAR), cách goal >= ..._to_goal;
  - nhãn đúng mục tiêu: B1/B2 còn đường trong hành lang ±detour_corridor quanh A*(M) (DWB tự né được), B3 thì
    không (cần replan) nhưng còn đường toàn cục; S1 vô nghiệm; N: Δ rỗng.
"""

import hashlib
import json
import math

import numpy as np

from . import grid as G
from . import layouts as LY

TYPES = ['N1', 'N2', 'N3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3', 'C4']
LEVELS = ['easy', 'medium', 'hard']


class Reject(Exception):
    pass


# ------------------------------------------------------------------------------------------------ A* (NavFn thật)
def astar(grid, start, goal, nav):
    from . import fast2d
    p = fast2d.navfn_plan(grid.to_dict(), list(start), list(goal), nav['radius'], nav['padding'],
                          nav['inflation'], nav['scaling'], False)
    return np.asarray(p)


def nav_params(cfg):
    s = cfg['sim']
    return {'radius': s['footprint_radius'], 'padding': s['global_padding'], 'inflation': s['global_inflation'],
            'scaling': s['cost_scaling']}


# ------------------------------------------------------------------------------------------------ difficulty
def difficulty(M, W, shapes, start, goal, path_m, path_w, gen):
    clr = G.clearance_field(W)
    ref = path_w if len(path_w) >= 2 else path_m
    # c: BỀ RỘNG lối đi nhỏ nhất dọc đường A* trên W (đường robot thực sự đi): tại mỗi điểm, tia vuông góc trái +
    # phải tới vật cản (tối đa 2 m mỗi bên), bỏ 0.5 m đầu/cuối. Pilot 2026-11-26: (1) "khe rộng nhất toàn tuyến" cho
    # B1/C2/C3 c ~ 0 (đường rộng nhất vòng xa vật cản); (2) khoảng hở tâm đường chỉ phản ánh inflation 0.40 của A*.
    occ = W.data >= 50
    pts, sarc = G.resample(ref, 0.05)
    widths = []
    for k in range(2, len(pts) - 2):
        if sarc[k] < 0.5 or sarc[k] > sarc[-1] - 0.5:
            continue
        dx, dy = pts[k + 2] - pts[k - 2]
        n = math.hypot(dx, dy)
        if n < 1e-9:
            continue
        nx, ny = -dy / n, dx / n
        w = 0.0
        for sgn in (1, -1):
            d = 0.0
            while d < 2.0:
                i, j = W.world_to_cell(pts[k][0] + sgn * d * nx, pts[k][1] + sgn * d * ny)
                if not (0 <= i < W.w and 0 <= j < W.h) or occ[j, i]:
                    break
                d += W.res / 2
            w += d
        widths.append(w)
    gap = float(min(widths)) if widths else 4.0
    c = float(np.clip((gen.get('c_width_easy', 1.60) - gap) / (gen.get('c_width_easy', 1.60) - 0.60), 0, 1))
    band = G.band_mask(W, ref, 1.0)
    rho_raw = float((W.data[band] >= 50).mean()) if band.any() else 0.0
    rho = float(np.clip(rho_raw / 0.30, 0, 1))
    turns = G.count_turns(ref)
    tau = float(np.clip(turns / 4.0, 0, 1))
    Lm, Lw = G.path_length(path_m), G.path_length(path_w)
    delta_raw = Lw / Lm if Lm > 0 and Lw > 0 else 1.0
    delta = float(np.clip(delta_raw - 1.0, 0, 1))
    nu_raw = visibility_distance(W, shapes, path_m)
    nu = 0.0 if nu_raw is None else float(np.clip((2.5 - nu_raw) / 1.5, 0, 1))
    score = 0.30 * c + 0.20 * rho + 0.15 * tau + 0.20 * delta + 0.15 * nu
    level = 'easy' if score < gen.get('easy_max', 0.33) else ('medium' if score < gen.get('medium_max', 0.66)
                                                                 else 'hard')
    return {'score': round(score, 4), 'level': level, 'formula': 'v3', 'c': round(c, 3), 'rho': round(rho, 3), 'tau': round(tau, 3),
            'delta': round(delta, 3), 'nu': round(nu, 3), 'gap': round(gap, 3), 'rho_raw': round(rho_raw, 3),
            'turns': turns, 'detour_ratio': round(delta_raw, 3),
            'visible_at': None if nu_raw is None else round(nu_raw, 2)}


def visibility_distance(W, shapes, path_m, lidar_range=2.5, fov=math.pi / 2, robot_r=0.20):
    """Khoảng cách dọc đường M từ chỗ LiDAR (theo hướng đường) thấy vật Δ chặn đầu tiên tới vật đó. None nếu không chặn."""
    if not shapes or len(path_m) < 2:
        return None
    pts, s = G.resample(path_m, 0.05)
    best = None
    for shp in shapes:
        d = G.shape_distance(shp, pts[:, 0], pts[:, 1])
        hit = np.nonzero(d < robot_r)[0]
        if len(hit) and (best is None or s[hit[0]] < best[0]):
            best = (s[hit[0]], shp)
    if best is None:
        return None
    s_b, shp = best
    own = G.shape_mask(W, shp, pad=W.res)
    occ = (W.data >= 50) & ~own
    # điểm trên biên vật
    if shp['type'] == 'circle':
        a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        bpts = np.stack([shp['x'] + shp['r'] * np.cos(a), shp['y'] + shp['r'] * np.sin(a)], 1)
    else:
        c, sn = math.cos(shp.get('yaw', 0)), math.sin(shp.get('yaw', 0))
        u = np.linspace(-0.5, 0.5, 7)
        loc = np.concatenate([np.stack([u * shp['sx'], np.full_like(u, sgn * shp['sy'] / 2)], 1) for sgn in (-1, 1)]
                             + [np.stack([np.full_like(u, sgn * shp['sx'] / 2), u * shp['sy']], 1) for sgn in (-1, 1)])
        bpts = np.stack([shp['x'] + c * loc[:, 0] - sn * loc[:, 1], shp['y'] + sn * loc[:, 0] + c * loc[:, 1]], 1)
    for k in range(0, len(pts), 2):
        if s[k] >= s_b:
            break
        if s_b - s[k] > lidar_range + 1.0:
            continue
        nxt = pts[min(k + 4, len(pts) - 1)]
        hd = math.atan2(nxt[1] - pts[k][1], nxt[0] - pts[k][0])
        o = (pts[k][0] + 0.15 * math.cos(hd), pts[k][1] + 0.15 * math.sin(hd))
        for b in bpts:
            rng = math.hypot(b[0] - o[0], b[1] - o[1])
            ang = abs((math.atan2(b[1] - o[1], b[0] - o[0]) - hd + math.pi) % (2 * math.pi) - math.pi)
            if rng <= lidar_range and ang < fov and G.line_of_sight(occ, W, o, (b[0], b[1])):
                return float(s_b - s[k])
    return 0.0


# ------------------------------------------------------------------------------------------------ checks
def feasibility(W, start, goal, gen):
    clr = G.clearance_field(W)
    a, b = W.world_to_cell(*start[:2]), W.world_to_cell(*goal[:2])
    if G.connected(clr, gen['feasible_inflate'], a, b):
        l30 = G.geodesic_length(clr, gen['feasible_inflate'], a, b, W.res)
        l20 = G.geodesic_length(clr, gen['unsolvable_inflate'], a, b, W.res)
        if l20 < l30 - 0.5:
            return 'ambiguous_shortcut', clr
        return 'feasible', clr
    if not G.connected(clr, gen['unsolvable_inflate'], a, b):
        return 'unsolvable', clr
    return 'ambiguous', clr


def delta_gap_ok(M, shapes, gen):
    """Khe mép-mép từ mỗi vật Δ tới vật cản khác (M hoặc Δ khác): >= gap_feasible_min hoặc <= gap_unsolvable_max."""
    for k, shp in enumerate(shapes):
        others = G.world_grid(M, [s for q, s in enumerate(shapes) if q != k])
        clr = G.clearance_field(others)
        ring = G.shape_mask(M, shp, pad=M.res) & ~G.shape_mask(M, shp, pad=-M.res)
        if not ring.any():
            ring = G.shape_mask(M, shp, pad=M.res)
        gap = float(clr[ring].min()) + M.res / 2 if ring.any() else 9.0
        if gen['gap_unsolvable_max'] < gap < gen['gap_feasible_min']:
            return False, gap
    return True, None


def start_goal_ok(clr_w, W, start, goal, gen):
    for p in (start, goal):
        i, j = W.world_to_cell(p[0], p[1])
        if not (0 <= i < W.w and 0 <= j < W.h) or clr_w[j, i] < gen['start_goal_clearance']:
            return False
    return True


def obstacles_far_enough(shapes, start, goal, path_m, gen, check_goal=True):
    if not shapes:
        return True
    pts, s = G.resample(path_m, 0.05)
    lidar = (start[0] + 0.15 * math.cos(start[2]), start[1] + 0.15 * math.sin(start[2]))
    for shp in shapes:
        if G.shape_distance(shp, np.array([lidar[0]]), np.array([lidar[1]]))[0] < gen['new_obstacle_min_from_start'] - 0.3:
            return False
        if check_goal and G.shape_distance(shp, np.array([goal[0]]), np.array([goal[1]]))[0] < \
                gen['new_obstacle_min_to_goal']:
            return False
        d = G.shape_distance(shp, pts[:, 0], pts[:, 1])
        near = np.nonzero(d < 0.6)[0]
        if len(near) and s[near[0]] < gen['new_obstacle_min_from_start']:
            return False
    return True


# ------------------------------------------------------------------------------------------------ helpers
def _sample_free(rng, clr, W, roi, min_clr, n=1):
    ok = clr >= min_clr
    if roi is not None:
        ok &= roi
    jj, ii = np.nonzero(ok)
    if len(ii) == 0:
        raise Reject('no free cell')
    k = rng.integers(0, len(ii), n)
    return [(W.origin[0] + (ii[q] + 0.5) * W.res, W.origin[1] + (jj[q] + 0.5) * W.res) for q in k]


def _heading_at(path, s_query):
    pts, s = G.resample(path, 0.05)
    k = int(np.clip(np.searchsorted(s, s_query), 0, len(pts) - 2))
    k2 = min(k + 4, len(pts) - 1)
    return pts[k], math.atan2(pts[k2][1] - pts[k][1], pts[k2][0] - pts[k][0])


def _start_goal(rng, M, roi, gen, nav, want_turns=None, l_range=None):
    clr = G.clearance_field(M)
    l_range = l_range or gen['l_ref']
    for _ in range(40):
        (sx, sy), (gx, gy) = _sample_free(rng, clr, M, roi, gen['start_goal_clearance'], 2)
        if math.hypot(gx - sx, gy - sy) < l_range[0]:
            continue
        p = astar(M, (sx, sy, 0.0), (gx, gy, 0.0), nav)
        if len(p) < 2:
            continue
        L = G.path_length(p)
        if not (l_range[0] <= L <= l_range[1]):
            continue
        if want_turns is not None:
            t = G.count_turns(p)
            if not (want_turns[0] <= t <= want_turns[1]):
                continue
        _, h0 = _heading_at(p, 0.3)
        syaw = h0 + rng.uniform(-0.8, 0.8)
        _, h1 = _heading_at(p, L - 0.3)
        return (sx, sy, float(syaw)), (gx, gy, float(h1)), p
    raise Reject('no start/goal with L_ref in range')


def _corridor_ends(rng, meta, M, nav, gen):
    wp = np.array(meta['waypoints'])
    a, b = wp[0], wp[-1]
    d0 = (wp[1] - wp[0]) / np.linalg.norm(wp[1] - wp[0])
    d1 = (wp[-1] - wp[-2]) / np.linalg.norm(wp[-1] - wp[-2])
    s = a + d0 * 0.6
    g = b - d1 * 0.6
    syaw = math.atan2(d0[1], d0[0]) + rng.uniform(-0.6, 0.6)
    gyaw = math.atan2(d1[1], d1[0])
    p = astar(M, (s[0], s[1], syaw), (g[0], g[1], gyaw), nav)
    if len(p) < 2:
        raise Reject('corridor: no A*')
    return (float(s[0]), float(s[1]), float(syaw)), (float(g[0]), float(g[1]), float(gyaw)), p


def _corner_arcs(path, eps=0.25, angle=math.radians(45)):
    """Vị trí (m dọc đường) các góc rẽ > angle của đường (RDP)."""
    s_pts = G.rdp(path, eps)
    if len(s_pts) < 3:
        return []
    pts, s = G.resample(path, 0.05)
    out = []
    v = np.diff(s_pts, axis=0)
    h = np.arctan2(v[:, 1], v[:, 0])
    for k in range(1, len(s_pts) - 1):
        if abs((h[k] - h[k - 1] + math.pi) % (2 * math.pi) - math.pi) > angle:
            d = np.hypot(pts[:, 0] - s_pts[k][0], pts[:, 1] - s_pts[k][1])
            out.append(float(s[int(np.argmin(d))]))
    return out


def _box(x, y, sx, sy, yaw=0.0):
    return {'type': 'box', 'x': float(x), 'y': float(y), 'sx': float(sx), 'sy': float(sy), 'yaw': float(yaw)}


def _point_on_path(path, s_query):
    (x, y), hd = _heading_at(path, s_query)
    return float(x), float(y), hd


def _wall_across(M, x, y, hd, thickness, extra=0.3, max_len=8.0):
    """Tường vuông góc đường tại (x, y), dài tới chạm vật cản hai bên (+extra lấn vào tường)."""
    clr = G.clearance_field(M)
    nx, ny = -math.sin(hd), math.cos(hd)
    ext = []
    for sgn in (1, -1):
        d = 0.0
        while d < max_len:
            i, j = M.world_to_cell(x + sgn * d * nx, y + sgn * d * ny)
            if not (0 <= i < M.w and 0 <= j < M.h) or clr[j, i] <= 0.0:
                break
            d += M.res
        ext.append(d)
    L = ext[0] + ext[1] + 2 * extra
    cx = x + (ext[0] - ext[1]) / 2 * nx
    cy = y + (ext[0] - ext[1]) / 2 * ny
    return cx, cy, L, ext


# ------------------------------------------------------------------------------------------------ types
def build(rng, stype, gen, nav, base=None, hint='any'):
    """Một ứng viên (chưa kiểm tra nhãn). base = (M, roi, meta) cho map mục tiêu; None = layout tổng quát."""
    target = base is not None
    if target:
        M0, roi, meta = base
        M = M0.copy()
    shapes, known = [], True
    narrow = hint == 'hard'
    if stype in ('N1', 'N2', 'N3'):
        if target:
            turns = {'N1': (0, 0), 'N2': (1, 1), 'N3': (2, 9)}[stype]
            start, goal, pm = _start_goal(rng, M, roi, gen, nav, want_turns=turns)
        else:
            n = {'N1': 0, 'N2': 1, 'N3': int(rng.integers(2, 5))}[stype]
            if stype == 'N1' and rng.random() < 0.4:
                M, meta = LY.room(rng)
                start, goal, pm = _start_goal(rng, M, None, gen, nav, want_turns=(0, 0))
            else:
                w = rng.uniform(0.8, 1.1) if narrow else (rng.uniform(1.0, 1.6) if hint == 'medium'
                                                         else rng.uniform(1.6, 2.5))
                M, meta = LY.corridor(rng, n, width=w)
                start, goal, pm = _corridor_ends(rng, meta, M, nav, gen)
    elif stype in ('B1', 'B2', 'C2', 'C3', 'S1') or (stype == 'C1'):
        if not target:
            r = rng.random()
            # hint -> hình học (độ khó cuối cùng vẫn do difficulty() quyết định, không theo hint)
            room_p = {'easy': 0.7, 'medium': 0.3, 'hard': 0.0}.get(hint, 0.45)
            wmin = {'B2': 1.8, 'C2': 1.4, 'C3': 1.3}.get(stype, 1.1)
            wr = {'easy': (max(wmin, 2.0), 2.6), 'medium': (max(wmin, 1.4), 2.2),
                  'hard': (wmin, wmin + 0.6)}.get(hint, (wmin, 2.5))
            turns = {'easy': (0, 1), 'medium': (0, 2), 'hard': (1, 3)}.get(hint, (0, 3))
            if stype == 'S1' and r < 0.5:
                M, meta = LY.corridor(rng, int(rng.integers(0, 3)))
                start, goal, pm = _corridor_ends(rng, meta, M, nav, gen)
            elif r < room_p:
                M, meta = LY.room(rng)
                start, goal, pm = _start_goal(rng, M, None, gen, nav)
            else:
                M, meta = LY.corridor(rng, int(rng.integers(turns[0], turns[1] + 1)), width=rng.uniform(*wr))
                start, goal, pm = _corridor_ends(rng, meta, M, nav, gen)
        else:
            start, goal, pm = _start_goal(rng, M, roi, gen, nav, l_range=(5.0, gen['l_ref'][1]))
        L = G.path_length(pm)
        s_lo, s_hi = gen['new_obstacle_min_from_start'] + 0.3, L - gen['new_obstacle_min_to_goal'] - 0.8
        if s_hi <= s_lo:
            raise Reject('path too short for new obstacles')
        if stype in ('B1', 'B2'):
            sz = gen['b1_size'] if stype == 'B1' else gen['b2_size']
            s_obs = rng.uniform(s_lo, s_hi)
            if hint == 'hard':          # ngay sau góc rẽ (góc khuất) nếu có
                corners = _corner_arcs(pm)
                lo_c, hi_c = gen.get('b_corner_offset', (0.6, 1.4))     # stress X1: (0.4, 0.9)
                cand = [c + rng.uniform(lo_c, hi_c) for c in corners if s_lo <= c + lo_c and c + hi_c <= s_hi]
                if gen.get('require_corner') and not cand:
                    raise Reject('stress: no corner for occluded obstacle')
                if cand:
                    s_obs = float(rng.choice(cand))
            x, y, hd = _point_on_path(pm, s_obs)
            a, b = rng.uniform(*sz), rng.uniform(*sz)
            off = rng.uniform(-0.25, 0.25) * min(a, b)
            if rng.random() < 0.3:
                shapes = [{'type': 'circle', 'x': x - off * math.sin(hd), 'y': y + off * math.cos(hd), 'r': a / 2}]
            else:
                shapes = [_box(x - off * math.sin(hd), y + off * math.cos(hd), a, b, rng.uniform(0, math.pi))]
            known = False
        elif stype == 'S1':
            x, y, hd = _point_on_path(pm, rng.uniform(s_lo, s_hi))
            if rng.random() < 0.5:
                th = rng.uniform(0.2, 0.4)
                cx, cy, Lw, _ = _wall_across(M, x, y, hd, th)
                shapes = [_box(cx, cy, th, Lw, hd)]
            else:   # rào quanh đích (cạnh trong 0.9–1.4 m), có thể chừa khe <= 0.40
                half = rng.uniform(0.45, 0.7)
                t = 0.2
                gx, gy = goal[0], goal[1]
                Lr = 2 * half + 2 * t
                shapes = [_box(gx, gy + half + t / 2, Lr, t), _box(gx, gy - half - t / 2, Lr, t),
                          _box(gx - half - t / 2, gy, t, Lr), _box(gx + half + t / 2, gy, t, Lr)]
                if rng.random() < 0.5:   # khe 0.30 m trên một cạnh
                    q = shapes.pop(int(rng.integers(0, 4)))
                    gapw = rng.uniform(0.25, 0.38)
                    if q['sx'] > q['sy']:
                        seg = (q['sx'] - gapw) / 2
                        shapes += [_box(q['x'] - gapw / 2 - seg / 2, q['y'], seg, t),
                                   _box(q['x'] + gapw / 2 + seg / 2, q['y'], seg, t)]
                    else:
                        seg = (q['sy'] - gapw) / 2
                        shapes += [_box(q['x'], q['y'] - gapw / 2 - seg / 2, t, seg),
                                   _box(q['x'], q['y'] + gapw / 2 + seg / 2, t, seg)]
            known = False
        elif stype == 'C1':
            s_c1 = rng.uniform(s_lo, s_hi)
            if gen.get('c1_corner_offset'):        # stress X3: khe hẹp ngay sau góc rẽ
                lo_c, hi_c = gen['c1_corner_offset']
                cand = [c + rng.uniform(lo_c, hi_c) for c in _corner_arcs(pm) if s_lo <= c + lo_c and c + hi_c <= s_hi]
                if not cand:
                    raise Reject('stress: no corner for C1 gap')
                s_c1 = float(rng.choice(cand))
            x, y, hd = _point_on_path(pm, s_c1)
            gapw = rng.uniform(*{'hard': (0.60, 0.66), 'easy': (0.72, 0.80)}.get(hint, gen['c1_gap']))
            th = rng.uniform(0.2, 0.6)
            off = rng.uniform(-0.08, 0.08)
            nx, ny = -math.sin(hd), math.cos(hd)
            cx, cy = x + off * nx, y + off * ny
            _, _, _, ext = _wall_across(M, cx, cy, hd, th)
            left = ext[0] - gapw / 2 + 0.3
            right = ext[1] - gapw / 2 + 0.3
            if left < 0.1 or right < 0.1:
                raise Reject('C1: corridor narrower than gap')
            shapes = [_box(cx + (gapw / 2 + left / 2) * nx, cy + (gapw / 2 + left / 2) * ny, th, left, hd),
                      _box(cx - (gapw / 2 + right / 2) * nx, cy - (gapw / 2 + right / 2) * ny, th, right, hd)]
            known = rng.random() < 0.5 and not gen.get('force_unknown')
        elif stype == 'C2':
            lo_n, hi_n = gen['c2_count']
            n = int(rng.integers(*{'easy': (lo_n, lo_n + 2), 'medium': (lo_n + 1, hi_n - 1),
                                   'hard': (hi_n - 2, hi_n + 1)}.get(hint, (lo_n, hi_n + 1))))
            for _ in range(60):
                if len(shapes) >= n:
                    break
                x, y, hd = _point_on_path(pm, rng.uniform(s_lo, s_hi))
                off = rng.uniform(-1.0, 1.0)
                sz = rng.uniform(0.2, 0.5)
                cand = _box(x - off * math.sin(hd), y + off * math.cos(hd), sz, rng.uniform(0.2, 0.5),
                            rng.uniform(0, math.pi))
                trial = shapes + [cand]
                if delta_gap_ok(M, trial, gen)[0]:
                    shapes = trial
            if len(shapes) < gen['c2_count'][0]:
                raise Reject('C2: could not place obstacles')
            known = rng.random() < 0.4 and not gen.get('force_unknown')
        elif stype == 'C3':
            sp = rng.uniform(*{'hard': (1.0, 1.3), 'easy': (1.6, 2.0)}.get(hint, gen['c3_spacing']))
            s = s_lo
            side = rng.choice([-1, 1])
            while s < s_hi:
                x, y, hd = _point_on_path(pm, s)
                off = side * rng.uniform(*gen['c3_offset'])
                sz = rng.uniform(0.3, 0.5)
                shapes.append(_box(x - off * math.sin(hd), y + off * math.cos(hd), sz, sz, hd))
                side = -side
                s += sp
            if len(shapes) < 3:
                raise Reject('C3: < 3 obstacles')
            known = rng.random() < 0.4 and not gen.get('force_unknown')
        if known and shapes:
            M = G.world_grid(M, shapes)     # vật cản thuộc bản đồ (đã biết)
            shapes = []
            # A* lại trên M mới
            pm = astar(M, start, goal, nav)
            if len(pm) < 2:
                raise Reject('known variant: no A* on M')
    elif stype == 'B3':
        if not target:
            M, meta = LY.two_route(rng)
            clr = G.clearance_field(M)
            start, goal, pm = _start_goal(rng, M, None, gen, nav, l_range=(5.0, gen['l_ref'][1]))
        else:
            start, goal, pm = _start_goal(rng, M, roi, gen, nav, l_range=(5.0, gen['l_ref'][1]))
        L = G.path_length(pm)
        s_lo, s_hi = gen['new_obstacle_min_from_start'] + 0.3, L - gen['new_obstacle_min_to_goal'] - 0.5
        if s_hi <= s_lo:
            raise Reject('B3: path too short')
        s_b3 = rng.uniform(s_lo, s_hi)
        if gen.get('b3_corner_offset'):            # stress X4: tường chặn ngay sau góc rẽ (phát hiện muộn)
            lo_c, hi_c = gen['b3_corner_offset']
            cand = [c + rng.uniform(lo_c, hi_c) for c in _corner_arcs(pm) if s_lo <= c + lo_c and c + hi_c <= s_hi]
            if not cand:
                raise Reject('stress: no corner for B3 wall')
            s_b3 = float(rng.choice(cand))
        x, y, hd = _point_on_path(pm, s_b3)
        th = rng.uniform(0.2, 0.4)
        cx, cy, Lw, ext = _wall_across(M, x, y, hd, th, max_len=3.0)
        if max(ext) >= 3.0:
            raise Reject('B3: route too wide to block')
        shapes = [_box(cx, cy, th, Lw, hd)]
        known = False
    elif stype == 'C4':
        if target:
            raise Reject('C4 only on generated layouts')
        lx = rng.uniform(5.0, 10.0)
        fill = {'easy': rng.uniform(0.10, 0.18), 'medium': rng.uniform(0.15, 0.27),
                'hard': rng.uniform(0.24, 0.35)}.get(hint, rng.uniform(*gen['c4_fill']))
        M, meta = LY.barn(rng, lx=lx, fill=fill)
        x0, x1, y0, y1 = 0.3, lx - 0.3, 0.0, meta['size'][1]
        roi = np.zeros(M.data.shape, bool)
        X, Y = M.cell_centers()
        roi |= ((X < x0 + 0.8) | (X > x1 - 0.8)) & (Y > y0) & (Y < y1)
        clrm = G.clearance_field(M)
        for _ in range(30):
            (sx, sy), = _sample_free(rng, clrm, M, roi & (X < x0 + 0.8), gen['start_goal_clearance'])
            (gx, gy), = _sample_free(rng, clrm, M, roi & (X > x1 - 0.8), gen['start_goal_clearance'])
            pm = astar(M, (sx, sy, 0.0), (gx, gy, 0.0), nav)
            if len(pm) >= 2:
                break
        else:
            raise Reject('C4: no A*')
        _, h0 = _heading_at(pm, 0.3)
        _, h1 = _heading_at(pm, G.path_length(pm) - 0.3)
        start, goal = (sx, sy, h0 + rng.uniform(-0.6, 0.6)), (gx, gy, h1)
        if rng.random() < 0.3:       # biến thể có vật cản mới (unknown) trên đường
            L = G.path_length(pm)
            if L > 5.0:
                x, y, hd = _point_on_path(pm, rng.uniform(3.3, L - 1.5))
                sz = rng.uniform(0.2, 0.4)
                shapes = [_box(x, y, sz, sz, rng.uniform(0, math.pi))]
                known = False
    else:
        raise ValueError(stype)
    if not target:
        meta = dict(meta)
    return M, shapes, start, goal, pm, known, meta


def validate(stype, M, shapes, start, goal, pm, gen, nav):
    """Kiểm tra + nhãn. Trả dict thông tin hoặc raise Reject."""
    W = G.world_grid(M, shapes)
    status, clr_w = feasibility(W, start, goal, gen)
    # Ngoài lưới map = sàn trống trong Fast 2D và Gazebo (map_to_world chỉ đùn ô occupied). Map Classroom có vùng trống
    # ngoài tòa nhà chạm mép lưới -> robot có thể vòng ra ngoài map (lần sinh 1: S1 "vô nghiệm" mà robot tới đích).
    # Nếu thêm viền trống 1 m làm đổi kết luận khả thi -> loại.
    n = int(round(1.0 / W.res))
    Wp = G.Grid(np.pad(W.data, n, constant_values=G.FREE), (W.origin[0] - n * W.res, W.origin[1] - n * W.res), W.res)
    if (W.data[[0, -1], :] < 50).any() or (W.data[:, [0, -1]] < 50).any():
        status_p, _ = feasibility(Wp, start, goal, gen)
        if status_p != status:
            raise Reject(f'border_leak ({status} -> {status_p} with free outside)')
    if not start_goal_ok(clr_w, W, start, goal, gen):
        raise Reject('start/goal clearance')
    if len(pm) < 2:
        raise Reject('no A* on M')
    L = G.path_length(pm)
    if not (gen['l_ref'][0] <= L <= gen['l_ref'][1]):
        raise Reject(f'L_ref {L:.1f} out of range')
    ok, gap = delta_gap_ok(M, shapes, gen)
    if not ok:
        raise Reject(f'delta gap {gap:.2f} in exclusion band')
    # S1 rào quanh đích: mục đích là bao kín đích -> miễn luật "vật cản cách goal >= 1.0 m" (lần sinh 1: house S1 0/10)
    if not obstacles_far_enough(shapes, start, goal, pm, gen, check_goal=stype != 'S1'):
        raise Reject('new obstacle too close to start/goal')
    if stype == 'S1':
        if status != 'unsolvable':
            raise Reject(f'S1 but {status}')
        return {'feasibility': status, 'W': W}
    if status != 'feasible':
        raise Reject(status)
    if stype in ('N1', 'N2', 'N3') and shapes:
        raise Reject('N with new obstacles')
    blocks = False
    if shapes:
        pts, _ = G.resample(pm, 0.05)
        blocks = any((G.shape_distance(s, pts[:, 0], pts[:, 1]) < 0.20).any() for s in shapes)
    if stype in ('B1', 'B2', 'B3') and not blocks:
        raise Reject('new obstacle does not block the A* path')
    if stype in ('B1', 'B2', 'B3'):
        band = G.band_mask(M, pm, gen['detour_corridor'])
        a, b = W.world_to_cell(*start[:2]), W.world_to_cell(*goal[:2])
        local = G.connected(clr_w, gen['feasible_inflate'], a, b, restrict=band)
        if stype in ('B1', 'B2') and not local:
            raise Reject('B1/B2 needs a local detour (use B3)')
        if stype == 'B3' and local:
            raise Reject('B3 must need replanning')
    pw = astar(W, start, goal, nav)
    if len(pw) < 2:
        raise Reject('feasible geometry but no A* on W (inscribed/inflation)')
    Lw = G.path_length(pw)
    if Lw > gen.get('max_detour_ratio', 3.0) * L:
        raise Reject(f'detour ratio {Lw / L:.1f} > max')
    # NavFn (NavfnPlanner) dừng trích đường sau 4·max(nx, ny) bước 0.5 ô = 2·max(nx, ny) ô: replan từ sâu trong tuyến
    # bị chặn cần ~ L + L_W -> giữ dưới 80 % giới hạn (pilot0 B3: FAILED giả vì giới hạn này).
    navfn_cap = 2.0 * max(M.w, M.h) * M.res
    if L + Lw > 0.8 * navfn_cap:
        raise Reject(f'path {L + Lw:.1f} m too long for NavFn on this map ({navfn_cap:.1f} m cap)')
    return {'feasibility': status, 'W': W, 'path_w': pw}


def scenario_hash(M, shapes, start, goal):
    h = hashlib.sha1()
    h.update(np.ascontiguousarray(M.data).tobytes())
    h.update(json.dumps([shapes, [round(v, 2) for v in start], [round(v, 2) for v in goal]],
                        sort_keys=True).encode())
    return h.hexdigest()[:16]


def generate_one(stype, seed, gen, nav, base=None, hint='any', map_name=None):
    """Một scenario hợp lệ (dict) hoặc None. Không âm thầm đổi type: chỉ thử lại cùng type."""
    rng = np.random.default_rng(seed)
    reasons = {}
    for attempt in range(gen.get('max_attempts_one', 25)):
        try:
            M, shapes, start, goal, pm, known, meta = build(rng, stype, gen, nav, base=base, hint=hint)
            info = validate(stype, M, shapes, start, goal, pm, gen, nav)
        except Reject as e:
            key = str(e).split(' ')[0]
            reasons[key] = reasons.get(key, 0) + 1
            continue
        W = info['W']
        if stype == 'S1':
            diff = {'score': None, 'level': 'unsolvable'}
        else:
            diff = difficulty(M, W, shapes, start, goal, pm, info['path_w'], gen)
        L = G.path_length(pm)
        sc = {
            'type': stype, 'group': stype[0], 'seed': int(seed), 'attempt': attempt,
            'feasibility': info['feasibility'], 'difficulty': diff,
            'knowledge': 'known' if not shapes else 'unknown',
            'map_name': map_name or 'generated', 'layout': meta if base is None else {'family': map_name},
            'start': [round(float(v), 4) for v in start], 'goal': [round(float(v), 4) for v in goal],
            'obstacles': shapes, 'l_ref': round(L, 3),
            'timeout': round(30.0 + 3.0 * L / 0.3, 1),
            'map': M,
        }
        sc['hash'] = scenario_hash(M, shapes, start, goal)
        return sc, reasons
    return None, reasons
