#!/usr/bin/env bash
# Ghi lại MỘT lần chạy robot thật (stage3 navigation) để phân tích sau bằng analyze_run.py.
# CHỈ subscribe (ros2 bag record) + đọc tham số/log: KHÔNG mở cổng phần cứng, KHÔNG publish lệnh.
# Chạy trong terminal riêng SAU KHI navigation.launch.py đã lên; Ctrl+C để dừng ghi.
#
#   stage3_navigation/scripts/record_run.sh <nhãn> [--no-cloud] [--note "ghi chú"]
#     <nhãn>      vd. cam_box_1m, long_route, wheels_off
#     --no-cloud  không ghi point cloud camera (~5-7 MB/s); mặc định CÓ ghi (để kiểm tra camera)
#     --note      mô tả: vật cản ở đâu, đích ở đâu, điều kiện ánh sáng...
#
# Kết quả: ~/robot_runs/<YYYYmmdd_HHMMSS>_<nhãn>/{bag/, meta.md, params/, logs/} và 1 dòng trong ~/robot_runs/RUN_LOG.md
set -u

LABEL="${1:-run}"; shift || true
CLOUD=1; NOTE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-cloud) CLOUD=0 ;;
    --note) NOTE="${2:-}"; shift ;;
    *) echo "tham số lạ: $1" >&2; exit 2 ;;
  esac
  shift
done

ROOT="${ROBOT_RUNS_DIR:-$HOME/robot_runs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="$ROOT/${STAMP}_${LABEL}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$DIR/params" "$DIR/logs"
START_EPOCH="$(date +%s)"

TOPICS=(
  # điều khiển / vận tốc
  /cmd_vel_out /cmd_vel_nav /cmd_vel_dwb /cmd_vel_joy /wheel_feedback
  /odometry/filtered /wheel_odom /imu_odom /imu/data
  # định vị
  /tf /tf_static /map /amcl_pose /initialpose /initialpose_guess /scan_match/status
  /localization/status /particle_cloud
  # cảm biến
  /scan_filtered /scan
  /camera/camera/depth/camera_info
  # costmap / kế hoạch / điều khiển
  /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
  /local_costmap/fusion_layer/status /global_path /plan /local_plan /speed_limit
  /CustomDWB/state /CustomDWB/trajectories
  /goal_pose /goal_path_bridge/follow_state
  /rosout
)
[ "$CLOUD" = 1 ] && TOPICS+=(/camera/camera/depth/color/points)

# ---- meta trước khi chạy ----
{
  echo "# Lần chạy ${STAMP}_${LABEL}"
  echo
  echo "- Bắt đầu: $(date '+%F %T')"
  echo "- Ghi chú: ${NOTE:-(không)}"
  echo "- Ghi cloud camera: $([ "$CLOUD" = 1 ] && echo có || echo không)"
  echo "- Git: $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null) ($(git -C "$REPO" status --short 2>/dev/null | wc -l) file chưa commit)"
  echo
  echo "## Node đang chạy"
  echo '```'
  timeout 5 ros2 node list 2>&1
  echo '```'
  echo "## Thiết bị serial"
  echo '```'
  ls -l /dev/serial/by-id/ 2>&1
  echo '```'
} > "$DIR/meta.md"

git -C "$REPO" diff --stat > "$DIR/params/git_diff_stat.txt" 2>/dev/null
cp "$REPO/stage3_navigation/config/"*.yaml "$DIR/params/" 2>/dev/null
# Tham số đang chạy thật (có thể khác file nếu đã ros2 param set)
for n in /controller_server /local_costmap/local_costmap /goal_path_bridge /localization_monitor /amcl; do
  f="$DIR/params/$(echo "$n" | tr '/' '_' | sed 's/^_//').yaml"
  timeout 10 ros2 param dump "$n" > "$f" 2>/dev/null || rm -f "$f"
done

echo "[record_run] ghi vào $DIR (Ctrl+C để dừng)"
# Ctrl+C tới cả nhóm tiến trình: bag dừng sạch, script bỏ qua SIGINT để chạy phần "sau khi dừng".
trap ':' INT
ros2 bag record -o "$DIR/bag" "${TOPICS[@]}"
trap - INT

# ---- sau khi dừng ----
END_EPOCH="$(date +%s)"
find "$HOME/.ros/log" -maxdepth 1 -type f -newermt "@$START_EPOCH" \
  \( -name 'goal_path_bridge_*' -o -name 'controller_server_*' -o -name 'localization_monitor_*' \
     -o -name 'amcl_*' -o -name 'planner_server_*' -o -name 'camera*' -o -name 'realsense*' \) \
  -exec cp {} "$DIR/logs/" \; 2>/dev/null
# launch.log của lần launch mới nhất (có log zlac8015d_move, ekf, twist_mux in ra màn hình)
LAST_LAUNCH="$(ls -td "$HOME"/.ros/log/20*/ 2>/dev/null | head -1)"
[ -n "$LAST_LAUNCH" ] && cp "$LAST_LAUNCH/launch.log" "$DIR/logs/launch.log" 2>/dev/null
{
  echo
  echo "- Kết thúc: $(date '+%F %T') ($((END_EPOCH - START_EPOCH)) s)"
  echo "- Bag: $(du -sh "$DIR/bag" 2>/dev/null | cut -f1)"
} >> "$DIR/meta.md"

LOG="$ROOT/RUN_LOG.md"
[ -f "$LOG" ] || printf '# Nhật ký chạy robot\n\n| Lần chạy | Ghi chú |\n|---|---|\n' > "$LOG"
echo "| \`${STAMP}_${LABEL}\` | ${NOTE:-} |" >> "$LOG"
echo "[record_run] xong. Phân tích: python3 $REPO/stage3_navigation/scripts/analyze_run.py $DIR"
