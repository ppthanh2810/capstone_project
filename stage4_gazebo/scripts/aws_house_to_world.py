#!/usr/bin/env python3
"""Chuyển world AWS RoboMaker Small House (Gazebo Classic, SDF 1.6) -> world Fortress của stage4.

Đọc  <aws>/worlds/small_house.world (ElementTree bỏ sẵn phần comment, vd. đèn 'sun')
Ghi  stage4_gazebo/worlds/aws_small_house.sdf (file này được commit; chạy lại script khi đổi bản AWS).

- Mỗi <model name=N><static/><include><uri>model://M</uri></include><pose>P</pose></model> của AWS
  -> <include><uri>model://M</uri><name>N</name><pose>P</pose><static>true</static></include>.
  Tên trùng (AWS đặt 2 model cùng tên AirconditionerA_01_001) -> thêm hậu tố _dupK (Fortress từ chối tên trùng).
- Sàn FloorB: chỉ lấy VISUAL mesh, collision là mặt phẳng z = 0 (ma sát 1.0 như empty_floor.sdf), giống kho.
- Đèn AWS giữ nguyên vị trí / màu / suy giảm nhưng TẮT bóng (point light có bóng rất tốn GPU, sensor render chung
  GPU với LiDAR/camera) + đèn hướng nhẹ không bóng vì đèn 'sun' của AWS bị comment.
- Physics / system / IMU / Sensors (gpu_lidar, depth camera) giống world kho.
- URI mesh của model nhà là `model://...` -> IGN_GAZEBO_RESOURCE_PATH chỉ cần <aws>/models
  (s2_slam.launch.py env:=house tự đặt).

Dùng: aws_house_to_world.py [<aws_dir>] [-o <world.sdf>]
"""

import argparse
import collections
import os
import xml.etree.ElementTree as ET

FLOOR = 'aws_robomaker_residential_FloorB_01'

HEADER = """<?xml version="1.0"?>
<!-- World nhà AWS RoboMaker Small House cho Gazebo Fortress — SINH bởi
     stage4_gazebo/scripts/aws_house_to_world.py từ {src} (nhánh ros2, MIT-0).
     Model nằm ngoài repo (scripts/fetch_aws_house.sh). Toạ độ giữ nguyên của AWS (nhà ~20 x 12 m quanh gốc).
     Sàn: visual FloorB + collision mặt phẳng. Đèn AWS không bóng. Không sửa tay: chạy lại script. -->
<sdf version="1.8">
  <world name="aws_small_house">
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
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="house_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <pose>{floor_pose}</pose>
          <geometry><mesh><uri>model://{floor}/meshes/aws_FloorB_01_visual.DAE</uri></mesh></geometry>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def light_xml(light):
    """<light> của AWS -> chuỗi SDF 1.8: bỏ thuộc tính frame của pose, tắt bóng."""
    out = [f'    <light name="{light.get("name")}" type="{light.get("type")}">']
    for child in light:
        if child.tag == 'cast_shadows':
            continue
        if child.tag == 'pose':
            out.append(f'      <pose>{" ".join(child.text.split())}</pose>')
        elif len(child):
            inner = ''.join(f'<{c.tag}>{c.text.strip()}</{c.tag}>' for c in child)
            out.append(f'      <{child.tag}>{inner}</{child.tag}>')
        else:
            out.append(f'      <{child.tag}>{(child.text or "").strip()}</{child.tag}>')
    out.append('      <cast_shadows>false</cast_shadows>')
    out.append('    </light>')
    return '\n'.join(out)


def main():
    here = os.path.dirname(os.path.realpath(__file__))
    default_aws = os.environ.get('AWS_HOUSE_DIR', os.path.expanduser('~/capstone_ws/third_party/aws_small_house'))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('aws_dir', nargs='?', default=default_aws)
    ap.add_argument('-o', '--output', default=os.path.join(here, '..', 'worlds', 'aws_small_house.sdf'))
    args = ap.parse_args()

    src = os.path.join(args.aws_dir, 'worlds', 'small_house.world')
    world = ET.parse(src).getroot().find('world')
    entries = []
    floor_pose = '0 0 0 0 0 0'
    seen = collections.Counter()
    for model in world.findall('model'):
        uri = model.find('include/uri').text.strip()
        pose_el = model.find('pose')
        if pose_el is None:
            pose_el = model.find('include/pose')
        pose = ' '.join((pose_el.text if pose_el is not None else '0 0 0 0 0 0').split())
        if uri == f'model://{FLOOR}':
            floor_pose = pose
            continue
        name = model.get('name')
        seen[name] += 1
        if seen[name] > 1:
            name = f'{name}_dup{seen[name] - 1}'
        entries.append((name, uri, pose))
    lights = world.findall('light')

    with open(args.output, 'w') as f:
        f.write(HEADER.format(src=os.path.relpath(src, args.aws_dir), floor=FLOOR, floor_pose=floor_pose))
        f.write('\n    <!-- ===== Đèn AWS (không bóng) ===== -->\n')
        for light in lights:
            f.write(light_xml(light) + '\n')
        f.write('\n    <!-- ===== Model AWS: cùng tên + pose với small_house.world ===== -->\n')
        for name, uri, pose in entries:
            f.write(f'    <include>\n      <uri>{uri}</uri>\n      <name>{name}</name>\n'
                    f'      <pose>{pose}</pose>\n      <static>true</static>\n    </include>\n')
        f.write(FOOTER)
    print(f'{len(entries)} model + sàn + {len(lights)} đèn -> {os.path.realpath(args.output)}')


if __name__ == '__main__':
    main()
