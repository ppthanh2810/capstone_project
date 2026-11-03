#!/usr/bin/env bash
# Tải model AWS RoboMaker Small House (nhánh ros2, MIT-0) cho world house của stage4_gazebo.
# Đặt NGOÀI repo (183 MB mesh + texture): mặc định ~/capstone_ws/third_party/aws_small_house
# (s2_slam.launch.py env:=house đọc từ đây, hoặc từ biến AWS_HOUSE_DIR).
# Ghim commit để world (worlds/aws_small_house.sdf) luôn khớp pose / tên model.
set -euo pipefail

REPO=https://github.com/aws-robotics/aws-robomaker-small-house-world.git
COMMIT=ff9631ca6d1db9c1ba656498151464b5ab74aafe   # ros2 @ 2021-08-20 "Remove Tutlebot env variable export step (#45)"
DEST=${AWS_HOUSE_DIR:-$HOME/capstone_ws/third_party/aws_small_house}

# Vá 2 model có quán tính không hợp lệ (Gazebo classic bỏ qua, Fortress từ chối nạp CẢ world:
# "A link named link has invalid inertia"). Cả hai đều static -> giá trị không ảnh hưởng mô phỏng.
#   Dumbbell_01: iyy 0 -> 0.1 (bất đẳng thức tam giác) | ShoeRack_01: thiếu izz -> 0.04
patch_inertia() {
  python3 - "$1/models" <<'PY'
import re, sys
root = sys.argv[1]
fixes = {'aws_robomaker_residential_Dumbbell_01': (r'<iyy>\s*0(\.0*)?\s*</iyy>', '<iyy>0.1</iyy>'),
         'aws_robomaker_residential_ShoeRack_01': (r'(<iyy>[^<]*</iyy>)(?![\s\S]*<izz>)', r'\1<izz>0.04</izz>')}
for model, (pat, rep) in fixes.items():
    path = f'{root}/{model}/model.sdf'
    s = open(path, newline='').read()
    t = re.sub(pat, rep, s, count=1)
    if t != s:
        open(path, 'w', newline='').write(t)
        print(f'Đã vá quán tính: {model}')
PY
}

# Repo AWS là package ament (cần gazebo_ros của Gazebo classic, không cài): colcon build ở ~/capstone_ws
# quét cả third_party/ -> COLCON_IGNORE để chỉ dùng models/, không build nó.
if [ -d "$DEST/models" ]; then
  touch "$DEST/COLCON_IGNORE"
  patch_inertia "$DEST"
  echo "Đã có: $DEST ($(git -C "$DEST" rev-parse --short HEAD 2>/dev/null || echo '?'))"
  exit 0
fi
mkdir -p "$(dirname "$DEST")"
git clone --depth 1 -b ros2 "$REPO" "$DEST"
touch "$DEST/COLCON_IGNORE"
patch_inertia "$DEST"
HEAD=$(git -C "$DEST" rev-parse HEAD)
if [ "$HEAD" != "$COMMIT" ]; then
  echo "Cảnh báo: nhánh ros2 đang ở $HEAD, khác commit đã kiểm ($COMMIT)" >&2
fi
echo "Xong: $DEST"
