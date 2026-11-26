"""Lưới chiếm chỗ, raster vật cản, kiểm tra khả thi hình học (numpy/scipy). Quy ước như nav_msgs/OccupancyGrid:
0 free, 100 occupied, -1 unknown; data[j, i] với j = hàng theo y tăng (hàng 0 = y nhỏ nhất), ô (i, j) phủ
[ox + i·res, ox + (i+1)·res) × [oy + j·res, ...).
"""

import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

FREE, OCC, UNK = 0, 100, -1


class Grid:
    def __init__(self, data, origin=(0.0, 0.0), resolution=0.05):
        self.data = np.ascontiguousarray(data, dtype=np.int8)
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(resolution)

    @property
    def h(self):
        return self.data.shape[0]

    @property
    def w(self):
        return self.data.shape[1]

    def copy(self):
        return Grid(self.data.copy(), self.origin, self.res)

    def to_dict(self):
        return {'origin': list(self.origin), 'resolution': self.res, 'data': self.data}

    def world_to_cell(self, x, y):
        return int(math.floor((x - self.origin[0]) / self.res)), int(math.floor((y - self.origin[1]) / self.res))

    def cell_centers(self):
        xs = self.origin[0] + (np.arange(self.w) + 0.5) * self.res
        ys = self.origin[1] + (np.arange(self.h) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    def extent(self):
        return (self.origin[0], self.origin[0] + self.w * self.res, self.origin[1], self.origin[1] + self.h * self.res)

    def occupied_mask(self, unknown_is_obstacle=True):
        m = self.data >= 50
        if unknown_is_obstacle:
            m |= self.data < 0
        return m


def load_map_yaml(path):
    """map_server (trinary): pgm -> occupancy theo occupied_thresh / free_thresh / negate."""
    with open(path) as f:
        m = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path), m['image'])).convert('L'), dtype=np.float64)
    p = (255.0 - img) / 255.0 if not m.get('negate', 0) else img / 255.0
    occ = np.full(img.shape, UNK, np.int8)
    occ[p > m['occupied_thresh']] = OCC
    occ[p < m['free_thresh']] = FREE
    occ = occ[::-1, :]       # ảnh hàng 0 = y lớn nhất
    return Grid(occ, m['origin'][:2], m['resolution'])


# ------------------------------------------------------------------------------------------------ shapes
def shape_mask(grid, shp, pad=0.0):
    """Ô có TÂM nằm trong shape (+pad) -> True."""
    X, Y = grid.cell_centers()
    if shp['type'] == 'circle':
        return np.hypot(X - shp['x'], Y - shp['y']) <= shp['r'] + pad
    c, s = math.cos(shp.get('yaw', 0.0)), math.sin(shp.get('yaw', 0.0))
    lx = c * (X - shp['x']) + s * (Y - shp['y'])
    ly = -s * (X - shp['x']) + c * (Y - shp['y'])
    return (np.abs(lx) <= shp['sx'] / 2 + pad) & (np.abs(ly) <= shp['sy'] / 2 + pad)


def shape_distance(shp, px, py):
    """Khoảng cách có dấu (vectorized) từ điểm tới biên shape."""
    if shp['type'] == 'circle':
        return np.hypot(px - shp['x'], py - shp['y']) - shp['r']
    c, s = math.cos(shp.get('yaw', 0.0)), math.sin(shp.get('yaw', 0.0))
    lx = np.abs(c * (px - shp['x']) + s * (py - shp['y'])) - shp['sx'] / 2
    ly = np.abs(-s * (px - shp['x']) + c * (py - shp['y'])) - shp['sy'] / 2
    out = np.hypot(np.maximum(lx, 0), np.maximum(ly, 0))
    inside = (lx <= 0) & (ly <= 0)
    return np.where(inside, np.maximum(lx, ly), out)


def world_grid(M, shapes):
    """W = M ⊕ Δ (raster tâm ô, đủ cho kiểm tra hình học; Fast 2D dùng hình học chính xác)."""
    W = M.copy()
    for s in shapes:
        W.data[shape_mask(M, s)] = OCC
    return W


# ------------------------------------------------------------------------------------------------ clearance
def clearance_field(grid, unknown_is_obstacle=True):
    """Khoảng cách (m) từ tâm ô tới BIÊN vật cản gần nhất (xấp xỉ: EDT tâm-tâm − res/2), <= 0 trong vật cản."""
    occ = grid.occupied_mask(unknown_is_obstacle)
    if not occ.any():
        return np.full(occ.shape, 1e6)
    d = ndimage.distance_transform_edt(~occ) * grid.res - grid.res / 2.0
    d[occ] = 0.0
    return d


def free_for_radius(clr, r):
    return clr > r


def component_of(mask, cell):
    lab, _ = ndimage.label(mask, structure=np.ones((3, 3)))
    i, j = cell
    if not (0 <= j < mask.shape[0] and 0 <= i < mask.shape[1]):
        return None, lab
    return lab[j, i], lab


def connected(clr, r, a_cell, b_cell, restrict=None):
    mask = clr > r
    if restrict is not None:
        mask = mask & restrict
    ia, ja = a_cell
    ib, jb = b_cell
    if not (mask[ja, ia] and mask[jb, ib]):
        return False
    lab, _ = ndimage.label(mask, structure=np.ones((3, 3)))
    return lab[ja, ia] == lab[jb, ib]


def bottleneck_radius(clr, a_cell, b_cell, lo=0.0, hi=2.0, iters=12):
    """r lớn nhất sao cho a, b còn nối trong {clr > r} (đường "rộng nhất"); khe bottleneck ≈ 2r."""
    if not connected(clr, lo, a_cell, b_cell):
        return -1.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if connected(clr, mid, a_cell, b_cell):
            lo = mid
        else:
            hi = mid
    return lo


def geodesic_length(clr, r, a_cell, b_cell, res):
    """Độ dài đường ngắn nhất (8-láng giềng, xấp xỉ) trong {clr > r}; inf nếu không nối."""
    mask = clr > r
    ia, ja = a_cell
    ib, jb = b_cell
    if not (mask[ja, ia] and mask[jb, ib]):
        return math.inf
    # Dijkstra trên lưới bằng scipy.sparse.csgraph (8 láng giềng, chi phí 1 / √2)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    h, w = mask.shape
    idx = -np.ones(mask.shape, np.int64)
    n = int(mask.sum())
    idx[mask] = np.arange(n)
    rows, cols, vals = [], [], []
    for dj, di, c in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
        j0, j1 = max(0, -dj), h - max(0, dj)
        i0, i1 = max(0, -di), w - max(0, di)
        a = mask[j0:j1, i0:i1]
        b = mask[j0 + dj:j1 + dj, i0 + di:i1 + di]
        sa = idx[j0:j1, i0:i1]
        sb = idx[j0 + dj:j1 + dj, i0 + di:i1 + di]
        both = a & b
        rows.append(sa[both])
        cols.append(sb[both])
        vals.append(np.full(int(both.sum()), c))
    rows, cols, vals = np.concatenate(rows), np.concatenate(cols), np.concatenate(vals)
    g = coo_matrix((np.concatenate([vals, vals]), (np.concatenate([rows, cols]), np.concatenate([cols, rows]))),
                   shape=(n, n)).tocsr()
    d = dijkstra(g, indices=idx[ja, ia])
    v = d[idx[jb, ib]]
    return float(v) * res if np.isfinite(v) else math.inf


def path_length(path):
    p = np.asarray(path)
    if len(p) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))))


def resample(path, step=0.05):
    p = np.asarray(path, float)
    if len(p) < 2:
        return p
    seg = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], step)
    return np.stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])], 1), t


def rdp(points, eps):
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    n = np.hypot(*ab)
    if n < 1e-9:
        d = np.hypot(pts[:, 0] - a[0], pts[:, 1] - a[1])
    else:
        d = np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / n
    k = int(np.argmax(d))
    if d[k] > eps:
        return np.vstack([rdp(pts[:k + 1], eps)[:-1], rdp(pts[k:], eps)])
    return np.vstack([a, b])


def count_turns(path, eps=0.25, angle=math.radians(45)):
    s = rdp(path, eps)
    if len(s) < 3:
        return 0
    v = np.diff(s, axis=0)
    h = np.arctan2(v[:, 1], v[:, 0])
    d = np.abs((np.diff(h) + math.pi) % (2 * math.pi) - math.pi)
    return int(np.sum(d > angle))


def band_mask(grid, path, half_width):
    """Ô cách polyline <= half_width."""
    p, _ = resample(path, grid.res)
    if len(p) == 0:
        return np.zeros(grid.data.shape, bool)
    m = np.zeros(grid.data.shape, bool)
    for x, y in p:
        i, j = grid.world_to_cell(x, y)
        if 0 <= i < grid.w and 0 <= j < grid.h:
            m[j, i] = True
    d = ndimage.distance_transform_edt(~m) * grid.res
    return d <= half_width


def line_of_sight(occ, grid, a, b):
    """True nếu đoạn a->b (m) không đi qua ô occupied (lấy mẫu nửa ô), bỏ qua điểm cuối."""
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(L / (grid.res * 0.5)))
    for k in range(1, n):
        t = k / n
        i, j = grid.world_to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if 0 <= i < grid.w and 0 <= j < grid.h and occ[j, i]:
            return False
    return True
