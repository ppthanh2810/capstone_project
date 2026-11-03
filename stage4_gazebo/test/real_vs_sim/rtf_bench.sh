#!/bin/bash
# Chạy 4 cấu hình S3 (world kho), xe lái tuyến warehouse, đo RTF 600 s mỗi cấu hình.
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash; source ~/capstone_ws/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=rtf88
SCR=$(cd "$(dirname "$0")/../../scripts" && pwd)
for cfg in "false false" "false true" "true false" "true true"; do
  set -- $cfg; gui=$1; cam=$2; tag=gui${gui}_cam${cam}
  ros2 launch stage4_gazebo s3_navigation.launch.py gui:=$gui rviz:=$gui camera:=$cam joy:=false init_pose:=true > launch_$tag.log 2>&1 &
  LP=$!; sleep 40
  python3 $SCR/route_drive.py --preset warehouse > drive_$tag.log 2>&1 &
  DP=$!
  python3 rtf_meter.py 600 rtf_$tag.json > meter_$tag.log 2>&1
  kill -INT $DP 2>/dev/null; sleep 1; kill -INT $LP; sleep 10
  pkill -f "ign gazebo" 2>/dev/null; pkill -f rviz2 2>/dev/null; sleep 3
  echo "$tag $(cat meter_$tag.log | tail -1)"
done
