#!/usr/bin/env bash
# Tải model AWS RoboMaker Small Warehouse (nhánh ros2, MIT-0) cho world warehouse của stage4_gazebo.
# Đặt NGOÀI repo (24 MB mesh + texture): mặc định ~/capstone_ws/third_party/aws_small_warehouse
# (s2_slam.launch.py env:=warehouse đọc từ đây, hoặc từ biến AWS_WAREHOUSE_DIR).
# Ghim commit để world (worlds/aws_small_warehouse.sdf) luôn khớp pose / tên model.
set -euo pipefail

REPO=https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
COMMIT=ee0af733315e78432408c3cd98d378ecee5f767c   # ros2 @ 2021 "Fix launch files for using gazebo_ros (#20)"
DEST=${AWS_WAREHOUSE_DIR:-$HOME/capstone_ws/third_party/aws_small_warehouse}

if [ -d "$DEST/models" ]; then
  echo "Đã có: $DEST ($(git -C "$DEST" rev-parse --short HEAD 2>/dev/null || echo '?'))"
  exit 0
fi
mkdir -p "$(dirname "$DEST")"
git clone --depth 1 -b ros2 "$REPO" "$DEST"
HEAD=$(git -C "$DEST" rev-parse HEAD)
if [ "$HEAD" != "$COMMIT" ]; then
  echo "Cảnh báo: nhánh ros2 đang ở $HEAD, khác commit đã kiểm ($COMMIT)" >&2
fi
echo "Xong: $DEST"
