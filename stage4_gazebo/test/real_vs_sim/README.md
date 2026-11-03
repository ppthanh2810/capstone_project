# So sánh sim với xe thật (bài quay tại chỗ, bag `~/rot_debug` 2026-10-14)

```bash
python3 bag_to_json.py ~/rot_debug real.json          # bag thật -> json
# sim: ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
ros2 launch stage4_gazebo s1_odom.launch.py gui:=false rviz:=false joy:=false accel_time_ms:=200 decel_time_ms:=200
python3 replay_cmd.py real.json sim.json               # phát lại /cmd_vel_out của bag lên /cmd_vel_nav (sim time)
python3 analyze_rotation.py real.json; python3 analyze_rotation.py sim.json
```
Kết quả: `docs/PROGRESS.md`, mục "So sánh sim với xe thật".
