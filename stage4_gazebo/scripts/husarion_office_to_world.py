#!/usr/bin/env python3
"""Chuyển world văn phòng Husarion (husarion_gz_worlds, SDF 1.6) -> world Fortress của stage4.

Đọc  <dir>/worlds/husarion_office.sdf
Ghi  stage4_gazebo/worlds/husarion_office.sdf (file này được commit; chạy lại script khi đổi bản Husarion).

- Giữ nguyên mọi <model> (tường Surfaces, cửa, vách kính, cầu thang, đồ đạc, 3 ROSbot trang trí trên bàn):
  cùng pose, mesh, collision. Tên bỏ tiền tố 'OpenRobotics/' (dấu '/' lẫn vào tên topic / entity của Gazebo);
  trùng tên sau đó -> hậu tố _dupK.
  Ép <static>true</static> (vách kính Glass_wall_* là model động có khớp trượt -> cố định cho LiDAR / map ổn định).
- Bỏ <state> (snapshot của Gazebo, trùng pose), <gui> (camera + floor list của bản gốc), physics / plugin cũ.
- Đèn 'sun' giữ hướng / màu nhưng TẮT bóng, scene không bóng (bóng rất tốn GPU, sensor render chung GPU với
  gpu_lidar / depth camera).
- Physics / system / IMU / Sensors (ogre2) giống world nhà / kho.
- Mesh: `model://<Tên>/...` -> IGN_GAZEBO_RESOURCE_PATH cần <dir>/models (s2_slam.launch.py env:=office tự đặt);
  `https://fuel.gazebosim.org/...` -> cache ~/.ignition/fuel (scripts/fetch_husarion_office.sh tải sẵn).

Dùng: husarion_office_to_world.py [<dir>] [-o <world.sdf>]
"""

import argparse
import os
import xml.etree.ElementTree as ET

HEADER = """<?xml version="1.0"?>
<!-- World văn phòng Husarion cho Gazebo Fortress — SINH bởi stage4_gazebo/scripts/husarion_office_to_world.py
     từ {src} (husarion/husarion_gz_worlds, Apache-2.0). Model nằm ngoài repo (scripts/fetch_husarion_office.sh).
     Toạ độ giữ nguyên của Husarion (văn phòng ~15 x 12 m, x -1..14, y -11..1.5). Mọi model static.
     Đèn không bóng. Không sửa tay: chạy lại script. -->
<sdf version="1.8">
  <world name="husarion_office">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- Cảm biến: IMU (S1), gpu_lidar (S2), depth camera (S3) — cần render engine -->
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <grid>false</grid>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""

FOOTER = """  </world>
</sdf>
"""


def clean_name(name):
    return name.split('/', 1)[1] if name.startswith('OpenRobotics/') else name


def indent(elem, level):
    ET.indent(elem, space='  ', level=level)
    return '    ' + ET.tostring(elem, encoding='unicode').strip()


def main():
    here = os.path.dirname(os.path.realpath(__file__))
    default_dir = os.environ.get('HUSARION_WORLDS_DIR',
                                 os.path.expanduser('~/capstone_ws/third_party/husarion_gz_worlds'))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', nargs='?', default=default_dir)
    ap.add_argument('-o', '--output', default=os.path.join(here, '..', 'worlds', 'husarion_office.sdf'))
    args = ap.parse_args()

    src = os.path.join(args.dir, 'worlds', 'husarion_office.sdf')
    world = ET.parse(src).getroot().find('world')
    names = set()
    out = []
    for model in world.findall('model'):
        name = clean_name(model.get('name'))
        base, k = name, 1
        while name in names:   # 'OpenRobotics/Shelf_1' và 'Shelf_1' cùng có -> hậu tố _dupK như world nhà
            name = f'{base}_dup{k}'
            k += 1
        names.add(name)
        model.set('name', name)
        static = model.find('static')
        if static is None:
            static = ET.Element('static')
            model.insert(0, static)
        static.text = 'true'
        out.append(model)

    with open(args.output, 'w') as f:
        f.write(HEADER.format(src=os.path.relpath(src, args.dir)))
        f.write('\n    <!-- ===== Model Husarion: cùng pose với husarion_office.sdf gốc ===== -->\n')
        for model in out:
            f.write(indent(model, 2) + '\n')
        f.write(FOOTER)
    print(f'{len(out)} model -> {os.path.realpath(args.output)}')


if __name__ == '__main__':
    main()
