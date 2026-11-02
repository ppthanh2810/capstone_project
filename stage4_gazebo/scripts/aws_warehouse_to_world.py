#!/usr/bin/env python3
"""Chuyển world AWS RoboMaker Small Warehouse (Gazebo Classic, SDF 1.6) -> world Fortress của stage4.

Đọc  <aws>/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world (bỏ phần comment = mái nhà)
Ghi  stage4_gazebo/worlds/aws_small_warehouse.sdf (file này được commit; chạy lại script khi đổi bản AWS).

- Mỗi <model name=N><include><uri>model://M</uri></include><pose>P</pose></model> của AWS
  -> <include><uri>model://M</uri><name>N</name><pose>P</pose></include> (cùng tên, cùng pose).
- Sàn GroundB: chỉ lấy VISUAL mesh, collision là mặt phẳng z = 0 (ma sát 1.0 như empty_floor.sdf) —
  collision mesh sàn làm bánh nảy / trượt li ti -> sai odometry không có trên xe thật.
- Physics / system / IMU / Sensors (gpu_lidar) giống world sinh từ bản đồ; giữ đèn trần AWS + ambient.
- Model AWS dùng URI mesh `file://models/...`: IGN_GAZEBO_RESOURCE_PATH phải chứa cả <aws> và <aws>/models
  (s2_slam.launch.py env:=warehouse tự đặt).

Dùng: aws_warehouse_to_world.py [<aws_dir>] [-o <world.sdf>]
"""

import argparse
import os
import xml.etree.ElementTree as ET

FLOOR = 'aws_robomaker_warehouse_GroundB_01'

HEADER = """<?xml version="1.0"?>
<!-- World kho AWS RoboMaker Small Warehouse (không mái) cho Gazebo Fortress — SINH bởi
     stage4_gazebo/scripts/aws_warehouse_to_world.py từ {src} (nhánh ros2, MIT-0).
     Model nằm ngoài repo (scripts/fetch_aws_warehouse.sh). Toạ độ giữ nguyên của AWS (kho ~14 x 21 m quanh gốc).
     Sàn: visual GroundB + collision mặt phẳng. Không sửa tay: chạy lại script. -->
<sdf version="1.8">
  <world name="aws_small_warehouse">
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
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <grid>false</grid>
    </scene>

    <!-- Đèn trần của AWS (point, cao 9 m) + đèn hướng nhẹ không bóng để kho không tối -->
    <light name="Warehouse_CeilingLight_003" type="point">
      <pose>0 0 9 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>80</range>
        <constant>0.3</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>true</cast_shadows>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="warehouse_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <!-- Mesh sàn GroundB: mặt trên ở z = 0.124 m (đơn vị cm trong DAE). AWS đặt z = -0.090 -> mặt sàn +3.4 cm
             so với các model khác. Ở đây hạ visual -0.125 để mặt sàn trùng collision z = 0 (x/y/yaw giữ của AWS:
             {floor_pose}). -->
        <visual name="visual">
          <pose>{floor_xy} -0.125 0 0 {floor_yaw}</pose>
          <geometry><mesh><uri>model://{floor}/meshes/{floor}_visual.DAE</uri></mesh></geometry>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def main():
    here = os.path.dirname(os.path.realpath(__file__))
    default_aws = os.environ.get('AWS_WAREHOUSE_DIR',
                                 os.path.expanduser('~/capstone_ws/third_party/aws_small_warehouse'))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('aws_dir', nargs='?', default=default_aws)
    ap.add_argument('-o', '--output', default=os.path.join(here, '..', 'worlds', 'aws_small_warehouse.sdf'))
    args = ap.parse_args()

    src = os.path.join(args.aws_dir, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    world = ET.parse(src).getroot().find('world')   # ElementTree bỏ comment (mái nhà) sẵn
    entries = []
    floor_pose = '0 0 0 0 0 0'
    for model in world.findall('model'):
        uri = model.find('include/uri').text.strip()
        pose_el = model.find('pose')
        if pose_el is None:
            pose_el = model.find('include/pose')
        pose = ' '.join((pose_el.text if pose_el is not None else '0 0 0 0 0 0').split())
        name = model.get('name')
        if uri == f'model://{FLOOR}':
            floor_pose = pose
            continue
        entries.append((name, uri, pose))

    with open(args.output, 'w') as f:
        fp = floor_pose.split()
        f.write(HEADER.format(src=os.path.relpath(src, args.aws_dir), floor=FLOOR, floor_pose=floor_pose,
                              floor_xy=f'{fp[0]} {fp[1]}', floor_yaw=fp[5]))
        f.write('\n    <!-- ===== Model AWS: cùng tên + pose với no_roof_small_warehouse.world ===== -->\n')
        for name, uri, pose in entries:
            f.write(f'    <include>\n      <uri>{uri}</uri>\n      <name>{name}</name>\n'
                    f'      <pose>{pose}</pose>\n    </include>\n')
        f.write(FOOTER)
    print(f'{len(entries)} model + sàn -> {os.path.realpath(args.output)}')


if __name__ == '__main__':
    main()
