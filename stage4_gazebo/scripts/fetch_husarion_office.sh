#!/usr/bin/env bash
# Tải world văn phòng Husarion (husarion/husarion_gz_worlds, Apache-2.0) cho env:=office của stage4_gazebo.
# Đặt NGOÀI repo: mặc định ~/capstone_ws/third_party/husarion_gz_worlds (hoặc biến HUSARION_WORLDS_DIR).
# Ghim commit để worlds/husarion_office.sdf (sinh bởi husarion_office_to_world.py) khớp tên / pose model.
# Mesh đồ đạc OpenRobotics nằm trên Gazebo Fuel (URI https://fuel...): tải sẵn vào cache ~/.ignition/fuel
# để Gazebo chạy được khi không có mạng (lần đầu cần mạng).
set -euo pipefail

REPO=https://github.com/husarion/husarion_gz_worlds.git
COMMIT=87271ef83672d9c9c96cb014ac64d746cacfd2c4   # main @ 2026-06-22 "0.1.0"
DEST=${HUSARION_WORLDS_DIR:-$HOME/capstone_ws/third_party/husarion_gz_worlds}

if [ ! -d "$DEST/models" ]; then
  mkdir -p "$(dirname "$DEST")"
  git clone -q "$REPO" "$DEST"
  git -C "$DEST" checkout -q "$COMMIT"
fi
# Repo Husarion là package ament (env-hook đặt GZ_SIM_RESOURCE_PATH cho Harmonic) -> không build, chỉ dùng
# models/ + worlds/.
touch "$DEST/COLCON_IGNORE"
HEAD=$(git -C "$DEST" rev-parse HEAD 2>/dev/null || echo '?')
if [ "$HEAD" != "$COMMIT" ]; then
  echo "Cảnh báo: $DEST đang ở $HEAD, khác commit đã kiểm ($COMMIT)" >&2
fi

# Model Fuel mà husarion_office.sdf trỏ tới (owner/name/version), tải 1 lần vào cache
grep -oE 'https://fuel\.gazebosim\.org/1\.0/[^/]+/models/[^/]+/[0-9]+' "$DEST/worlds/husarion_office.sdf" \
  | sort -u | while read -r url; do
  rel=${url#https://fuel.gazebosim.org/1.0/}           # openrobotics/models/adjtable/3
  if [ -d "$HOME/.ignition/fuel/fuel.gazebosim.org/$rel" ]; then
    continue
  fi
  echo "Tải Fuel: $rel"
  ign fuel download -u "$url" >/dev/null
done
# Map tham chiếu cắt từ world (chỉ cho route_drive.py --preset office lập tuyến, KHÔNG dùng để định vị)
HERE=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")    # --symlink-install: về thư mục scripts/ nguồn
WORLD="$HERE/../worlds/husarion_office.sdf"
[ -f "$WORLD" ] || WORLD="$HERE/../../share/stage4_gazebo/worlds/husarion_office.sdf"   # cài thường (lib/stage4_gazebo)
mkdir -p "$DEST/maps"
python3 "$HERE/world_to_ref_map.py" "$WORLD" -o "$DEST/maps/office_ref" --resource "$DEST/models"
echo "Xong: $DEST (${HEAD:0:7}), Fuel cache: ~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models"
