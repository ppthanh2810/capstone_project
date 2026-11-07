#!/usr/bin/env python3
"""Cắt ngang world Gazebo (SDF) -> bản đồ tham chiếu 2D (map_server yaml + pgm) ở toạ độ world.

CHỈ để lập tuyến cho route_drive.py (A* trên ô trắng) và so map SLAM với world; KHÔNG phải map định vị.
Lấy mọi <collision> (box, cylinder, mesh .obj / .stl / .dae), ghép pose model * link * collision (SDF 1.6,
pose tương đối cha, không xét relative_to), cắt các tam giác bởi mặt phẳng z = --heights (mặc định 0.1 ..
0.6 m: cao robot + laptop 0.60 m) -> các đoạn thẳng -> vẽ vào lưới. Ô có đoạn cắt = đen, còn lại trắng
(bên trong vật đặc vẫn trắng: route_drive nới --inflate nên không đi xuyên tường mỏng).
Bỏ qua mặt phẳng sàn (<plane>). Mesh model://X -> tìm trong --resource; https://fuel... -> ~/.ignition/fuel.

Dùng: world_to_ref_map.py <world.sdf> -o <map_noext> [--resource DIR ...] [--res 0.05]
"""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

FUEL_CACHE = os.path.expanduser('~/.ignition/fuel')


# ---------- pose ----------
def pose_matrix(text):
    v = [float(x) for x in (text or '').split()] + [0.0] * 6
    x, y, z, r, p, w = v[:6]
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    m = np.eye(4)
    m[:3, :3] = [[cw * cp, cw * sp * sr - sw * cr, cw * sp * cr + sw * sr],
                 [sw * cp, sw * sp * sr + cw * cr, sw * sp * cr - cw * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def child_pose(elem):
    p = elem.find('pose')
    return pose_matrix(p.text if p is not None else None)


# ---------- mesh loaders -> (N, 3, 3) triangles ----------
def load_obj(path):
    verts, tris = [], []
    with open(path, errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split('/')[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1]))
    v = np.asarray(verts)
    return v[np.asarray(tris)] if tris else np.zeros((0, 3, 3))


def load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = int.from_bytes(data[80:84], 'little')
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
            return rec['v'].astype(np.float64)
    nums = re.findall(r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data.decode(errors='ignore'))
    v = np.asarray(nums, dtype=np.float64)
    return v[: len(v) // 3 * 3].reshape(-1, 3, 3)


def load_dae(path):
    """Collada tối giản: mọi <triangles>/<polylist> của mọi geometry, bỏ qua node transform (đủ cho map tham chiếu)."""
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0].strip('{')} if root.tag.startswith('{') else {'c': ''}

    def q(t):
        return f'c:{t}' if ns['c'] else t

    unit = root.find(q('asset') + '/' + q('unit'), ns)
    scale = float(unit.get('meter', 1.0)) if unit is not None else 1.0
    up = root.find(q('asset') + '/' + q('up_axis'), ns)
    parts = []
    for mesh in root.iter(root.tag.replace('COLLADA', 'mesh')):
        sources = {s.get('id'): np.asarray(s.find(q('float_array'), ns).text.split(), dtype=np.float64)
                   for s in mesh.findall(q('source'), ns) if s.find(q('float_array'), ns) is not None}
        vert = mesh.find(q('vertices'), ns)
        pos = sources[vert.find(q('input'), ns).get('source').lstrip('#')].reshape(-1, 3)
        idx = []
        for prim in list(mesh.findall(q('triangles'), ns)) + list(mesh.findall(q('polylist'), ns)):
            inputs = prim.findall(q('input'), ns)
            stride = max(int(i.get('offset', 0)) for i in inputs) + 1
            voff = next(int(i.get('offset', 0)) for i in inputs if i.get('semantic') == 'VERTEX')
            p = np.asarray(prim.find(q('p'), ns).text.split(), dtype=np.int64).reshape(-1, stride)[:, voff]
            if prim.tag.endswith('polylist'):
                start = 0
                for c in np.asarray(prim.find(q('vcount'), ns).text.split(), dtype=np.int64):
                    poly = p[start:start + c]
                    idx += [(poly[0], poly[k], poly[k + 1]) for k in range(1, c - 1)]
                    start += c
            else:
                idx += [tuple(t) for t in p.reshape(-1, 3)]
        if idx:
            parts.append(pos[np.asarray(idx)] * scale)
    tris = np.concatenate(parts) if parts else np.zeros((0, 3, 3))
    if up is not None and up.text.strip() == 'Y_UP':
        tris = tris[..., [0, 2, 1]] * np.array([1, -1, 1])
    return tris


def box_tris(size):
    sx, sy, sz = (float(v) / 2 for v in size.split())
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return np.array([[c[a], c[b], c[d]] for a, b, cc, d in faces for (a, b, d) in ((a, b, cc), (a, cc, d))])


def cylinder_tris(radius, length, n=24):
    r, h = float(radius), float(length) / 2
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
    tris = []
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        p = [np.r_[a, -h], np.r_[b, -h], np.r_[b, h], np.r_[a, h]]
        tris += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return np.asarray(tris)


def resolve(uri, resources):
    if uri.startswith('model://'):
        rel = uri[len('model://'):]
        for d in resources:
            path = os.path.join(d, rel)
            if os.path.isfile(path):
                return path
    elif uri.startswith('https://fuel.gazebosim.org/1.0/'):
        owner, _, name, ver, *rest = uri[len('https://fuel.gazebosim.org/1.0/'):].split('/')
        base = os.path.join(FUEL_CACHE, 'fuel.gazebosim.org', owner.lower(), 'models', name.lower(), ver)
        rel = '/'.join(rest[1:] if rest and rest[0] == 'files' else rest)
        for path in (os.path.join(base, rel), os.path.join(base, 'files', rel)):
            if os.path.isfile(path):
                return path
    elif uri.startswith('file://') and os.path.isfile(uri[7:]):
        return uri[7:]
    elif os.path.isfile(uri):
        return uri
    return None


_cache = {}


def mesh_tris(geom, resources, missing):
    uri = geom.find('uri').text.strip()
    path = resolve(uri, resources)
    if path is None:
        missing.add(uri)
        return np.zeros((0, 3, 3))
    if path not in _cache:
        ext = path.rsplit('.', 1)[-1].lower()
        _cache[path] = {'obj': load_obj, 'stl': load_stl, 'dae': load_dae}[ext](path)
    tris = _cache[path]
    s = geom.find('scale')
    if s is not None:
        tris = tris * np.array([float(v) for v in s.text.split()])
    return tris


# ---------- cắt + vẽ ----------
def slice_segments(tris, z):
    """Đoạn giao của tam giác với mặt z (mảng (M, 2, 2))."""
    d = tris[..., 2] - z
    segs = []
    sign = d > 0
    cross = (sign.any(1)) & (~sign.all(1))
    for t, dd in zip(tris[cross], d[cross]):
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (dd[i] > 0) != (dd[j] > 0):
                f = dd[i] / (dd[i] - dd[j])
                pts.append(t[i, :2] + f * (t[j, :2] - t[i, :2]))
        if len(pts) == 2:
            segs.append(pts)
    return np.asarray(segs).reshape(-1, 2, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world')
    ap.add_argument('-o', '--output', required=True, help='đường dẫn không đuôi (ghi .yaml + .pgm)')
    ap.add_argument('--resource', action='append', default=[], help='thư mục chứa model (như IGN_GAZEBO_RESOURCE_PATH)')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--heights', default='0.1,0.2,0.35,0.5,0.6')
    ap.add_argument('--margin', type=float, default=1.0)
    args = ap.parse_args()
    heights = [float(h) for h in args.heights.split(',')]
    resources = args.resource + [p for p in os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '').split(':') if p]

    world = ET.parse(args.world).getroot().find('world')
    all_tris, missing = [], set()
    for model in world.findall('model'):
        mm = child_pose(model)
        for link in model.findall('link'):
            lm = mm @ child_pose(link)
            for col in link.findall('collision'):
                geom = col.find('geometry')
                g = geom[0] if len(geom) else None
                if g is None or g.tag == 'plane':
                    continue
                if g.tag == 'box':
                    tris = box_tris(g.find('size').text)
                elif g.tag == 'cylinder':
                    tris = cylinder_tris(g.find('radius').text, g.find('length').text)
                elif g.tag == 'mesh':
                    tris = mesh_tris(g, resources, missing)
                else:
                    print(f'Bỏ qua hình {g.tag} ({model.get("name")})', file=sys.stderr)
                    continue
                if len(tris):
                    m = lm @ child_pose(col)
                    all_tris.append(tris @ m[:3, :3].T + m[:3, 3])
    if missing:
        sys.exit('Không tìm thấy mesh: ' + ', '.join(sorted(missing)))
    tris = np.concatenate(all_tris)
    segs = np.concatenate([slice_segments(tris, z) for z in heights])
    lo = segs.reshape(-1, 2).min(0) - args.margin
    hi = segs.reshape(-1, 2).max(0) + args.margin
    w, h = (np.ceil((hi - lo) / args.res)).astype(int)
    occ = np.zeros((h, w), dtype=bool)
    for (p, q) in segs:
        n = int(np.ceil(np.linalg.norm(q - p) / (args.res * 0.5))) + 1
        pts = p + np.linspace(0, 1, n)[:, None] * (q - p)
        c = np.floor((pts - lo) / args.res).astype(int)
        occ[h - 1 - c[:, 1], c[:, 0]] = True
    img = np.where(occ, 0, 254).astype(np.uint8)
    Image.fromarray(img).save(args.output + '.pgm')
    with open(args.output + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(args.output)}.pgm\nmode: trinary\nresolution: {args.res}\n'
                f'origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{len(tris)} tam giác, {len(segs)} đoạn cắt -> {args.output}.yaml ({w} x {h} ô, '
          f'x {lo[0]:.2f}..{hi[0]:.2f}, y {lo[1]:.2f}..{hi[1]:.2f})')


if __name__ == '__main__':
    main()
