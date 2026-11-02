#!/usr/bin/env python3
"""Sinh world Gazebo từ bản đồ occupancy (map_server .yaml + .pgm) — S2.

Ô "occupied" (theo occupied_thresh / negate của yaml, giống map_server) được gộp thành hình chữ nhật
(tham lam theo hàng) rồi đùn lên thành tường cao `height` m, ghi thành MỘT mesh STL nhị phân (1 visual
+ 1 collision -> 1 draw call, thay vì vài nghìn hộp). Ô unknown / free không tạo gì.

Toạ độ world Gazebo = toạ độ frame `map` của bản đồ (dùng origin + resolution của yaml) -> pose AMCL so
thẳng được với /sim/ground_truth/odom. Chỉ hỗ trợ origin yaw = 0.

Kết quả để trong thư mục cache (mặc định ~/.ros/stage4_gazebo/worlds/<tên>_<hash>/), không nằm trong repo:
  world.sdf, walls.stl
Dùng:
  map_to_world.py <map.yaml> [--height 1.0] [--out DIR] [--force]
In ra đường dẫn world.sdf.
"""

import argparse
import hashlib
import math
import os
import struct
import sys

import numpy as np
import yaml
from PIL import Image

GENERATOR_VERSION = '1'  # đổi khi đổi định dạng đầu ra -> cache cũ bị bỏ qua


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_path)
    img = np.asarray(Image.open(image_path).convert('L'), dtype=np.float64)
    # Giống nav2 map_server: p = (255 - v)/255 (negate: v/255); occupied nếu p > occupied_thresh
    p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
    occ = p > float(meta['occupied_thresh'])
    origin = meta['origin']
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise RuntimeError('map_to_world: chưa hỗ trợ origin có yaw khác 0')
    return occ, float(meta['resolution']), float(origin[0]), float(origin[1]), image_path


def merge_rectangles(occ):
    """Gộp ô occupied thành hình chữ nhật (tham lam: kéo dài theo hàng rồi theo cột). Trả về [(r0, c0, r1, c1)]."""
    h, w = occ.shape
    used = np.zeros_like(occ, dtype=bool)
    rects = []
    for r in range(h):
        row = occ[r] & ~used[r]
        c = 0
        while c < w:
            if not row[c]:
                c += 1
                continue
            c1 = c
            while c1 + 1 < w and row[c1 + 1]:
                c1 += 1
            r1 = r
            while r1 + 1 < h and occ[r1 + 1, c:c1 + 1].all() and not used[r1 + 1, c:c1 + 1].any():
                r1 += 1
            used[r:r1 + 1, c:c1 + 1] = True
            rects.append((r, c, r1, c1))
            c = c1 + 1
    return rects


def box_triangles(x0, y0, x1, y1, z1):
    """Hộp [x0,x1]x[y0,y1]x[0,z1], bỏ mặt đáy (nằm trên sàn). Tam giác CCW nhìn từ ngoài."""
    a, b, c, d = (x0, y0), (x1, y0), (x1, y1), (x0, y1)
    tris = []

    def quad(p0, p1, p2, p3, n):
        tris.append((n, p0, p1, p2))
        tris.append((n, p0, p2, p3))

    top = [(*p, z1) for p in (a, b, c, d)]
    bot = [(*p, 0.0) for p in (a, b, c, d)]
    quad(top[0], top[1], top[2], top[3], (0, 0, 1))
    quad(bot[0], bot[1], top[1], top[0], (0, -1, 0))   # y = y0
    quad(bot[1], bot[2], top[2], top[1], (1, 0, 0))    # x = x1
    quad(bot[2], bot[3], top[3], top[2], (0, 1, 0))    # y = y1
    quad(bot[3], bot[0], top[0], top[3], (-1, 0, 0))   # x = x0
    return tris


def write_stl(path, rects, res, ox, oy, nrows, height):
    tris = []
    for r0, c0, r1, c1 in rects:
        # hàng 0 của ảnh là mép TRÊN bản đồ (y lớn nhất)
        x0 = ox + c0 * res
        x1 = ox + (c1 + 1) * res
        y0 = oy + (nrows - 1 - r1) * res
        y1 = oy + (nrows - r0) * res
        tris.extend(box_triangles(x0, y0, x1, y1, height))
    with open(path, 'wb') as f:
        f.write(b'stage4_gazebo map_to_world'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for n, p0, p1, p2 in tris:
            f.write(struct.pack('<12fH', *n, *p0, *p1, *p2, 0))
    return len(tris)


WORLD_TEMPLATE = """<?xml version="1.0"?>
<!-- SINH TỰ ĐỘNG bởi stage4_gazebo/scripts/map_to_world.py — không sửa tay.
     Nguồn: {yaml_path} ({n_occ} ô occupied -> {n_rect} hộp, tường cao {height} m).
     Toạ độ world = frame map của bản đồ. Phần còn lại giống worlds/empty_floor.sdf + system Sensors (gpu_lidar). -->
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- Cảm biến: IMU (S1), gpu_lidar (S2, cần render engine) -->
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="map_walls">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>file://{stl_path}</uri></mesh></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{stl_path}</uri></mesh></geometry>
          <material>
            <ambient>0.55 0.45 0.35 1</ambient>
            <diffuse>0.75 0.62 0.48 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def generate(yaml_path, height=1.0, out_dir=None, force=False):
    """Sinh (hoặc dùng lại cache) world cho bản đồ. Trả về đường dẫn world.sdf."""
    yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
    occ, res, ox, oy, image_path = load_map(yaml_path)

    h = hashlib.sha1()
    for p in (yaml_path, image_path):
        with open(p, 'rb') as f:
            h.update(f.read())
    h.update(f'{height:.4f}|{GENERATOR_VERSION}'.encode())
    tag = h.hexdigest()[:10]
    name = os.path.splitext(os.path.basename(yaml_path))[0]
    if out_dir is None:
        out_dir = os.path.join(os.path.expanduser('~'), '.ros', 'stage4_gazebo', 'worlds', f'{name}_{tag}')
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, 'world.sdf')
    stl_path = os.path.join(out_dir, 'walls.stl')
    if not force and os.path.isfile(world_path) and os.path.isfile(stl_path):
        return world_path

    rects = merge_rectangles(occ)
    n_tri = write_stl(stl_path, rects, res, ox, oy, occ.shape[0], height)
    world_name = 'map_' + ''.join(ch if ch.isalnum() else '_' for ch in name)
    with open(world_path, 'w') as f:
        f.write(WORLD_TEMPLATE.format(yaml_path=yaml_path, n_occ=int(occ.sum()), n_rect=len(rects),
                                      height=height, world_name=world_name, stl_path=stl_path))
    print(f'[map_to_world] {yaml_path}: {int(occ.sum())} ô occupied -> {len(rects)} hộp, {n_tri} tam giác '
          f'-> {world_path}', file=sys.stderr)
    return world_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--height', type=float, default=1.0, help='chiều cao tường (m)')
    ap.add_argument('--out', default=None, help='thư mục đầu ra (mặc định cache trong ~/.ros)')
    ap.add_argument('--force', action='store_true', help='sinh lại kể cả khi đã có cache')
    args = ap.parse_args()
    if not math.isfinite(args.height) or args.height <= 0.0:
        ap.error('--height phải > 0')
    print(generate(args.map_yaml, args.height, args.out, args.force))


if __name__ == '__main__':
    main()
