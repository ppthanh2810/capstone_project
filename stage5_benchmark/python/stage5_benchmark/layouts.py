"""Layout (Static Map M) cho Scenario Generator: rule-based (phòng, hành lang có góc rẽ, nhiều tuyến) + BARN-like
(cellular automata, Perille et al. 2020) + map mục tiêu (Classroom = maps/map.yaml, House = stage4 house.yaml).

Mọi layout có viền đặc >= margin quanh vùng tự do (NavFn coi 7 ô biên là vật cản; map_to_world đùn ô occupied).
"""

import math
import os

import numpy as np
from scipy import ndimage

from . import grid as G
from .config import repo_root

RES = 0.05


def _solid(lx, ly, margin):
    w, h = int(round((lx + 2 * margin) / RES)), int(round((ly + 2 * margin) / RES))
    return G.Grid(np.full((h, w), G.OCC, np.int8), (-margin, -margin), RES)


def _carve_rect(g, x0, y0, x1, y1, val=G.FREE):
    i0, j0 = g.world_to_cell(min(x0, x1), min(y0, y1))
    i1, j1 = g.world_to_cell(max(x0, x1), max(y0, y1))
    g.data[max(j0, 0):max(j1, 0), max(i0, 0):max(i1, 0)] = val


def _carve_polyline(g, pts, half_width):
    """Hành lang = tập điểm cách polyline <= half_width (góc rẽ tròn ngoài, vuông trong)."""
    X, Y = g.cell_centers()
    m = np.zeros(g.data.shape, bool)
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = np.clip(((X - ax) * dx + (Y - ay) * dy) / max(L2, 1e-12), 0, 1)
        d = np.hypot(X - (ax + t * dx), Y - (ay + t * dy))
        # vuông góc: dùng chuẩn vô cùng theo hướng đoạn để hành lang có thành phẳng
        m |= d <= half_width
    g.data[m] = G.FREE


def room(rng, lx=None, ly=None, margin=0.6):
    lx = lx or rng.uniform(4.0, 16.0)
    ly = ly or rng.uniform(4.0, 16.0)
    g = _solid(lx, ly, margin)
    _carve_rect(g, 0, 0, lx, ly)
    return g, {'family': 'room', 'size': [round(lx, 2), round(ly, 2)]}


def corridor(rng, n_turns, width=None, seg=None, margin=0.6):
    """Hành lang polyline: n_turns góc rẽ 60–120° (N1: 0, N2: 1, N3: 2–4)."""
    width = width or rng.uniform(0.8, 2.5)
    pts = [(0.0, 0.0)]
    heading = 0.0
    for k in range(n_turns + 1):
        L = seg or rng.uniform(2.5, 6.0)
        x, y = pts[-1]
        pts.append((x + L * math.cos(heading), y + L * math.sin(heading)))
        heading += rng.choice([-1, 1]) * math.radians(rng.uniform(60, 120))
    pts = np.array(pts)
    lo = pts.min(0) - width / 2 - 0.2
    pts = pts - lo
    hi = pts.max(0) + width / 2 + 0.2
    g = _solid(hi[0], hi[1], margin)
    _carve_polyline(g, [tuple(p) for p in pts], width / 2)
    return g, {'family': f'corridor{n_turns}', 'width': round(width, 2), 'waypoints': pts.round(3).tolist()}


def two_route(rng, margin=0.6):
    """Phòng có khối giữa -> 2 tuyến (B3: chặn một tuyến, còn tuyến kia)."""
    lx, ly = rng.uniform(7.0, 16.0), rng.uniform(4.5, 10.0)
    g = _solid(lx, ly, margin)
    _carve_rect(g, 0, 0, lx, ly)
    side_w = rng.uniform(0.9, 2.0)            # độ rộng mỗi tuyến
    bx0, bx1 = rng.uniform(1.8, 3.0), lx - rng.uniform(1.8, 3.0)
    _carve_rect(g, bx0, side_w, bx1, ly - side_w, G.OCC)
    return g, {'family': 'two_route', 'size': [round(lx, 2), round(ly, 2)], 'block': [bx0, side_w, bx1, ly - side_w],
               'route_width': round(side_w, 2)}


def barn(rng, lx=None, ly=None, fill=None, cell=0.15, smooth=4, margin=0.6):
    """BARN-like: CA trên ô 0.15 m (fill ban đầu, 4 vòng làm mịn: tường nếu >= 5/8 láng giềng là tường), phóng 3x."""
    lx = lx or rng.uniform(5.0, 10.0)
    ly = ly or rng.uniform(4.0, 8.0)
    fill = fill if fill is not None else rng.uniform(0.10, 0.35)
    nx, ny = int(lx / cell), int(ly / cell)
    ca = rng.random((ny, nx)) < fill
    k = np.ones((3, 3))
    k[1, 1] = 0
    for _ in range(smooth):
        n = ndimage.convolve(ca.astype(int), k, mode='constant', cval=0)
        ca = np.where(n >= 5, True, np.where(n <= 2, False, ca))
    up = int(round(cell / RES))
    big = np.kron(ca, np.ones((up, up), bool))
    g = _solid(nx * cell, ny * cell, margin)
    i0, j0 = g.world_to_cell(0.0, 0.0)
    region = g.data[j0:j0 + big.shape[0], i0:i0 + big.shape[1]]
    region[:] = np.where(big, G.OCC, G.FREE)
    return g, {'family': 'barn', 'size': [round(nx * cell, 2), round(ny * cell, 2)], 'fill': round(float(fill), 3),
               'cell': cell, 'smooth': smooth}


# ------------------------------------------------------------------------------------------------ map mục tiêu
_TARGET_CACHE = {}


def target_map(name):
    """Classroom = maps/map.yaml; House = stage4_gazebo/maps/house.yaml. ROI = vùng được lấy mẫu start/goal."""
    if name in _TARGET_CACHE:
        return _TARGET_CACHE[name]
    R = repo_root()
    path = {'classroom': os.path.join(R, 'maps', 'map.yaml'),
            'house': os.path.join(R, 'stage4_gazebo', 'maps', 'house.yaml'),
            'warehouse': os.path.join(R, 'stage4_gazebo', 'maps', 'warehouse.yaml'),
            'office': os.path.join(R, 'stage4_gazebo', 'maps', 'office.yaml')}[name]
    g = G.load_map_yaml(path)
    occ = g.data >= 50
    # ROI: bên trong bao lồi các ô tường (classroom: ngoài tòa nhà cũng là FREE trong map), co 0.5 m
    from scipy.spatial import Delaunay
    jj, ii = np.nonzero(occ)
    pts = np.stack([ii, jj], 1)
    hull = Delaunay(pts[np.random.default_rng(0).choice(len(pts), min(len(pts), 4000), replace=False)])
    J, I = np.mgrid[0:g.h, 0:g.w]
    inside = hull.find_simplex(np.stack([I.ravel(), J.ravel()], 1)) >= 0
    roi = inside.reshape(g.data.shape)
    roi = ndimage.binary_erosion(roi, iterations=int(0.5 / g.res))
    _TARGET_CACHE[name] = (g, roi, path)
    return _TARGET_CACHE[name]
