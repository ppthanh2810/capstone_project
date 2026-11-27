#!/usr/bin/env python3
"""Spawn / xoá vật cản thử nghiệm (models/test_obstacles) trong world Gazebo Fortress đang chạy.

Dùng được với MỌI world (house, warehouse, office, world sinh từ map như Classroom = maps/map.yaml, ...):
gọi thẳng service của system UserCommands (/world/<w>/create, /remove, /scene/info) qua CLI `ign service`,
không cần ROS graph. Vật cản là đồ MỚI, không nằm trong map tĩnh; không có gì tự spawn — chỉ khi gọi script.

  spawn_obstacle.py --model tall_small --x 1.0 --y 2.0 --yaw 0.0      # tên tự sinh test_obstacle_tall_small_<k>
  spawn_obstacle.py --model low_long --x 2 --y 0 --yaw-deg 90 --world aws_small_house
  spawn_obstacle.py --list                     # vật cản thử nghiệm đang có trong world
  spawn_obstacle.py --remove test_obstacle_tall_small_1
  spawn_obstacle.py --remove-all               # chỉ xoá model tên bắt đầu bằng test_obstacle_
  spawn_obstacle.py --models                   # 6 loại vật cản + kích thước

Chạy qua ROS: ros2 run stage4_gazebo spawn_obstacle.py ...
World: tự dò (/world/<w>/create); có nhiều world cùng IGN_PARTITION -> bắt buộc --world.
Chỉ xoá được model có tiền tố test_obstacle_ (cả bản chèn từ GUI), không bao giờ xoá robot / tường / đồ đạc.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

PREFIX = 'test_obstacle_'
LIB_DIR = 'test_obstacles'


def library_dir():
    """Thư mục models/test_obstacles: share của package (install) hoặc cạnh script (source)."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('stage4_gazebo'), 'models', LIB_DIR))
    except Exception:  # noqa: BLE001 - chưa source workspace / không có ament_index
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.join(here, '..', 'models', LIB_DIR))
    for c in candidates:
        if os.path.isdir(c):
            return os.path.realpath(c)
    sys.exit('Không tìm thấy thư mục models/test_obstacles (đã build + source workspace chưa?)')


def available_models(lib):
    """{tên: (đường dẫn model.sdf, (dài, rộng, cao))}"""
    out = {}
    for name in sorted(os.listdir(lib)):
        sdf = os.path.join(lib, name, 'model.sdf')
        if not (os.path.isfile(sdf) and os.path.isfile(os.path.join(lib, name, 'model.config'))):
            continue
        size = ET.parse(sdf).getroot().find('.//collision/geometry/box/size').text.split()
        out[name] = (sdf, tuple(float(v) for v in size))
    return out


def ign_service(service, reqtype, reptype, req, timeout_ms=3000):
    cmd = ['ign', 'service', '-s', service, '--reqtype', reqtype, '--reptype', reptype,
           '--timeout', str(timeout_ms), '--req', req]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 10)
    return r.stdout


def active_worlds():
    out = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True, timeout=15).stdout
    return sorted(set(re.findall(r'^/world/([^/]+)/create$', out, re.M)))


def pick_world(requested):
    worlds = active_worlds()
    partition = os.environ.get('IGN_PARTITION', '')
    where = f' (IGN_PARTITION="{partition}")' if partition else ''
    if requested:
        if requested not in worlds:
            sys.exit(f'World "{requested}" không chạy{where}. World đang chạy: {worlds or "không có"}')
        return requested
    if not worlds:
        sys.exit(f'Không thấy world Gazebo nào đang chạy{where}. Launch simulation trước '
                 '(cùng IGN_PARTITION với terminal này).')
    if len(worlds) > 1:
        sys.exit(f'Có nhiều world đang chạy: {worlds}. Chọn một bằng --world <tên>.')
    return worlds[0]


def model_names(world):
    """Tên các model cấp cao nhất trong world (scene/info)."""
    out = ign_service(f'/world/{world}/scene/info', 'ignition.msgs.Empty', 'ignition.msgs.Scene', '')
    return re.findall(r'^model \{\n  name: "([^"]+)"', out, re.M)


def test_obstacles(world):
    return [n for n in model_names(world) if n.startswith(PREFIX)]


def spawn(world, model, sdf, x, y, yaw, name=None):
    existing = set(model_names(world))
    if name is None:
        k = 1
        while f'{PREFIX}{model}_{k}' in existing:
            k += 1
        name = f'{PREFIX}{model}_{k}'
    elif not name.startswith(PREFIX):
        name = PREFIX + name
    if name in existing:
        sys.exit(f'Đã có entity tên "{name}" trong world {world}.')
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'sdf_filename: "{sdf}" name: "{name}" allow_renaming: false '
           f'pose {{position {{x: {x} y: {y} z: 0.0}} orientation {{x: 0 y: 0 z: {qz:.9f} w: {qw:.9f}}}}}')
    out = ign_service(f'/world/{world}/create', 'ignition.msgs.EntityFactory', 'ignition.msgs.Boolean', req)
    if 'data: true' not in out:
        sys.exit(f'Spawn thất bại ({out.strip() or "không phản hồi"}).')
    return name


def remove(world, name):
    out = ign_service(f'/world/{world}/remove', 'ignition.msgs.Entity', 'ignition.msgs.Boolean',
                      f'name: "{name}" type: MODEL')
    return 'data: true' in out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', help='tall_small | tall_wide | tall_long | low_small | low_wide | low_long')
    ap.add_argument('--x', type=float, default=0.0, help='m, frame world (= frame map của bản đồ sim)')
    ap.add_argument('--y', type=float, default=0.0, help='m')
    ap.add_argument('--yaw', type=float, default=None, help='rad (trục dài của hộp = trục x model)')
    ap.add_argument('--yaw-deg', type=float, default=None, help='độ (thay cho --yaw)')
    ap.add_argument('--name', default=None, help=f'tên entity (tự thêm tiền tố {PREFIX}); mặc định tự sinh')
    ap.add_argument('--world', default=None, help='tên world; mặc định tự dò (bắt buộc khi có nhiều world)')
    ap.add_argument('--list', action='store_true', help='liệt kê vật cản thử nghiệm trong world')
    ap.add_argument('--remove', metavar='NAME', nargs='+', help=f'xoá vật cản (tên phải có tiền tố {PREFIX})')
    ap.add_argument('--remove-all', action='store_true', help=f'xoá mọi model tên bắt đầu bằng {PREFIX}')
    ap.add_argument('--models', action='store_true', help='in 6 loại vật cản + kích thước rồi thoát')
    a = ap.parse_args()

    models = available_models(library_dir())
    if a.models:
        for n, (sdf, (l, w, h)) in models.items():
            print(f'{n:11s} {l:.2f} x {w:.2f} x {h:.2f} m  {sdf}')
        return
    actions = sum([bool(a.model), a.list, bool(a.remove), a.remove_all])
    if actions != 1:
        ap.error('chọn đúng một trong: --model, --list, --remove, --remove-all (hoặc --models)')
    world = pick_world(a.world)

    if a.list:
        names = test_obstacles(world)
        print(f'[{world}] {len(names)} vật cản thử nghiệm' + (':' if names else ''))
        for n in names:
            print('  ' + n)
    elif a.remove_all:
        names = test_obstacles(world)
        failed = [n for n in names if not remove(world, n)]
        print(f'[{world}] đã xoá {len(names) - len(failed)}/{len(names)} vật cản thử nghiệm')
        if failed:
            sys.exit(f'Không xoá được: {failed}')
    elif a.remove:
        present = set(model_names(world))
        bad = False
        for n in a.remove:
            if not n.startswith(PREFIX):
                print(f'Bỏ qua "{n}": chỉ xoá model có tiền tố {PREFIX}', file=sys.stderr)
                bad = True
            elif n not in present:
                print(f'Không có "{n}" trong world {world}', file=sys.stderr)
                bad = True
            elif remove(world, n):
                print(f'[{world}] đã xoá {n}')
            else:
                print(f'Xoá "{n}" thất bại', file=sys.stderr)
                bad = True
        sys.exit(1 if bad else 0)
    else:
        if a.model not in models:
            ap.error(f'--model phải là một trong {list(models)}')
        if a.yaw is not None and a.yaw_deg is not None:
            ap.error('chỉ dùng --yaw hoặc --yaw-deg')
        yaw = math.radians(a.yaw_deg) if a.yaw_deg is not None else (a.yaw or 0.0)
        sdf, (l, w, h) = models[a.model]
        name = spawn(world, a.model, sdf, a.x, a.y, yaw, a.name)
        print(f'[{world}] {name}: {a.model} {l:.2f} x {w:.2f} x {h:.2f} m tại '
              f'({a.x:+.3f}, {a.y:+.3f}) yaw {yaw:+.3f} rad')


if __name__ == '__main__':
    main()
