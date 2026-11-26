"""Lưu / đọc scenario + manifest. Một bộ (split) = thư mục:
  manifest.jsonl    một dòng / scenario (id, type, difficulty, feasibility, knowledge, split, seed, map ref, Δ, start,
                    goal, l_ref, timeout, hash, generator/config version)
  maps/<map_id>.npz lưới M (int8, origin, resolution); map mục tiêu dùng chung một file
  summary.json      thống kê sinh (tỷ lệ hợp lệ, lý do loại, phân bố type × difficulty)
"""

import hashlib
import json
import os

import numpy as np

from . import grid as G

GENERATOR_VERSION = 'v3.0-2026-11-26'


def map_id_of(M):
    h = hashlib.sha1()
    h.update(np.ascontiguousarray(M.data).tobytes())
    h.update(json.dumps([M.origin, M.res]).encode())
    return h.hexdigest()[:16]


def save_split(path, scenarios, summary=None):
    os.makedirs(os.path.join(path, 'maps'), exist_ok=True)
    with open(os.path.join(path, 'manifest.jsonl'), 'w') as f:
        for sc in scenarios:
            M = sc['map']
            mid = map_id_of(M)
            mp = os.path.join(path, 'maps', mid + '.npz')
            if not os.path.exists(mp):
                np.savez_compressed(mp, data=M.data, origin=np.array(M.origin), resolution=M.res)
            row = {k: v for k, v in sc.items() if k != 'map'}
            row['map_id'] = mid
            row['generator_version'] = GENERATOR_VERSION
            f.write(json.dumps(row, ensure_ascii=False) + '\n')
    if summary is not None:
        with open(os.path.join(path, 'summary.json'), 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)


_MAPS = {}


def load_map(path, mid):
    key = (path, mid)
    if key not in _MAPS:
        z = np.load(os.path.join(path, 'maps', mid + '.npz'))
        _MAPS[key] = G.Grid(z['data'], tuple(z['origin']), float(z['resolution']))
    return _MAPS[key]


def load_split(path, with_maps=True):
    out = []
    with open(os.path.join(path, 'manifest.jsonl')) as f:
        for line in f:
            sc = json.loads(line)
            if with_maps:
                sc['map'] = load_map(path, sc['map_id'])
            out.append(sc)
    return out


def write_map_yaml(M, out_dir, name):
    """M -> pgm + yaml (map_server / Gazebo map_to_world). Trinary: 0 -> 254 (free), 100 -> 0, -1 -> 205."""
    from PIL import Image
    os.makedirs(out_dir, exist_ok=True)
    img = np.full(M.data.shape, 205, np.uint8)
    img[M.data == 0] = 254
    img[M.data >= 50] = 0
    Image.fromarray(img[::-1]).save(os.path.join(out_dir, name + '.pgm'))
    with open(os.path.join(out_dir, name + '.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {M.res}\n'
                f'origin: [{M.origin[0]}, {M.origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return os.path.join(out_dir, name + '.yaml')
