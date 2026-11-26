#!/usr/bin/env bash
# Chạy regression Navigation trên Gazebo headless, cách ly mạng (CHỈ SIM, không chạm phần cứng).
#   stage4_gazebo/test/run_regression.sh <out_dir> <label> [--repeat 3] [--only R1,R2] [--no-require-localized]
#        [-- launch_arg:=value ...]
# Ví dụ:  run_regression.sh ~/bench/reg planner --repeat 3 --no-require-localized
#         run_regression.sh ~/bench/reg full --only R2,R4 -- monitor:=true
# (không set -u: setup.bash của ROS dùng biến chưa khai báo)
OUT=${1:?out_dir}; LABEL=${2:?label}; shift 2
REG_ARGS=(); LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--" ]; then shift; LAUNCH_ARGS=("$@"); break; fi
  REG_ARGS+=("$1"); shift
done
mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88} ROS_LOCALHOST_ONLY=1 IGN_PARTITION=${IGN_PARTITION:-reg$ROS_DOMAIN_ID}
LOG="$OUT/${LABEL}_launch.log"
setsid ros2 launch stage4_gazebo s3_navigation.launch.py gui:=false rviz:=false joy:=false \
  x:=0 y:=0 yaw:=-1.5708 "${LAUNCH_ARGS[@]}" >"$LOG" 2>&1 &
LPID=$!
cleanup() {
  kill -INT -"$LPID" 2>/dev/null; sleep 5; kill -KILL -"$LPID" 2>/dev/null
  pkill -f "ign gazebo.*$IGN_PARTITION" 2>/dev/null
}
trap cleanup EXIT
sleep 25
python3 "$(ros2 pkg prefix stage4_gazebo)/lib/stage4_gazebo/s3_regression.py" --out "$OUT/${LABEL}.json" "${REG_ARGS[@]}" 2>&1 | tee "$OUT/${LABEL}.txt"
exit "${PIPESTATUS[0]}"
