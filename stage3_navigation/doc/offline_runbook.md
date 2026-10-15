# Runbook — kiểm thử offline CustomDWB và các bước phần cứng (cho người vận hành)

> **Phần 1–3 không chạm phần cứng.** Phần 4 là thử trên robot: **chỉ người vận hành chạy, sau khi xác nhận**,
> bánh kê khỏi mặt đất, tay ở E-stop. `navigation_enabled:=false` **không** tắt phần cứng khi `stage2:=true`
> (LiDAR, IMU, Modbus motor, PS4 vẫn chạy).

## 1. Build + test (không cần ROS graph, trừ plugin test chạy trong domain riêng)

```bash
source /opt/ros/humble/setup.bash
cd ~/capstone_ws
colcon build --symlink-install --packages-select stage3_navigation
source install/setup.bash
colcon test --packages-select stage3_navigation && colcon test-result --verbose --test-result-base build/stage3_navigation
```
Gồm: `test_dwa_core` (44 test: dynamic window, phanh, va chạm, bám đường, đích, dao động, tham số, 17 kịch bản kín
vòng × 3 seed), `test_dwb_controller_plugin` (12 test qua `nav2_core::Controller` thật; CMake đặt
`ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`), lint (uncrustify, flake8, pep257, xmllint, lint_cmake; cppcheck 2.7 bị
ament tự bỏ qua). `test_test_astar_planning.py.xunit.xml` là kết quả cũ còn sót (test không còn trong repo).

## 2. Benchmark lõi kín vòng (không ROS graph, vài giây)

```bash
ros2 run stage3_navigation dwa_closed_loop_sim --out /tmp/dwa_nominal --runs 5 --robot nominal
ros2 run stage3_navigation dwa_closed_loop_sim --out /tmp/dwa_sluggish --runs 5 --robot sluggish
# thử tham số: --set path_distance_scale=2.0 --set control_latency=0.2 ; vết từng chu kỳ: --trace
```
Mã thoát ≠ 0 nếu có lần chạy va chạm / vượt giới hạn / lệnh không hữu hạn / không thành công khi kỳ vọng thành công.
Mô hình: `test/sim_world.hpp` (costmap tổng hợp thấy toàn bộ vật cản, robot có trễ + giới hạn gia tốc riêng).

## 3. Fixture ROS offline (controller_server thật, không phần cứng)

### 3a. Benchmark tự động (CustomDWB vs Nav2 DWB Standard vs Nav2 DWB LimitedAccel)
```bash
cd ~/capstone_ws/capstone_project/stage3_navigation/test/offline
python3 run_benchmark.py --out ~/capstone_ws/stage3_benchmark_runs/<tên> --runs 5 --workers 4
```
Mỗi lần chạy tự đặt `ROS_DOMAIN_ID` riêng (150 + worker) và `ROS_LOCALHOST_ONLY=1`, launch
`navigation.launch.py stage2:=false rviz:=false navigation_enabled:=false auto_follow:=false`, kiểm graph (không có
`zlac8015d_move`, `twist_mux`, `joy*`, `sllidar`, `imu_odom`, `ekf_filter_node`, `amcl`, `slam_toolbox`;
`/cmd_vel_nav` không có publisher; `/cmd_vel_dwb` chỉ do `controller_server` publish) **trước** khi gửi FollowPath.
Kết quả: `runs.csv`, `summary.{json,md}`, `nav2_params_benchmark.yaml`, `raw/<scenario>__<profile>__<run>/`.

### 3b. Chạy tay (xem bằng RViz/CLI)
Terminal 1 (graph cách ly):
```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
source ~/capstone_ws/install/setup.bash
ros2 launch stage3_navigation navigation.launch.py stage2:=false rviz:=false navigation_enabled:=false \
  monitor:=false      # không có localization_monitor offline -> bridge không đòi /localization/status
```
Terminal 2 (cùng export):
```bash
python3 ~/capstone_ws/capstone_project/stage3_navigation/test/offline/offline_sim.py --ros-args \
  -p scenario:=obstacle_near_path \
  -p scenario_file:=$HOME/capstone_ws/capstone_project/stage3_navigation/test/offline/scenarios.json
```
Kiểm tra (cùng export):
```bash
ros2 node list --no-daemon                       # KHÔNG được có zlac8015d_move / twist_mux / joy / sllidar
ros2 topic info /cmd_vel_nav --no-daemon         # Publisher count: 0
ros2 topic info /cmd_vel_dwb -v --no-daemon      # chỉ controller_server
ros2 run tf2_ros tf2_echo map base_footprint     # TF có
ros2 param get /controller_server CustomDWB.max_vel_x
ros2 topic echo /CustomDWB/state                 # phase, compute_ms, rej_*, cost_*
ros2 action list                                 # /follow_path
```
Gửi path: `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose:
{position: {x: 3.0}, orientation: {w: 1.0}}}"` (A* qua `planner_server`, cần `/map` từ `offline_sim.py`) rồi
`ros2 service call /goal_path_bridge/follow std_srvs/srv/Trigger`. Đây là topic goal, không phải topic vận tốc;
chỉ chạy trong graph cách ly ở trên.

### 3c. Replanning (goal_path_bridge, vật cản chưa có trong bản đồ)
```bash
cd ~/capstone_ws/capstone_project/stage3_navigation/test/offline
python3 run_replan_test.py --out ~/capstone_ws/stage3_replan_runs/<tên>            # 5 kịch bản, replan bật
python3 run_replan_test.py --out <dir> --replan false --scenario replan_detour      # đối chứng hành vi cũ
```
Domain 161 + `ROS_LOCALHOST_ONLY=1`, `auto_follow:=true monitor:=false`, kiểm graph trước khi gửi `/goal_pose`.
Kết quả: `<dir>/summary.json`, `<dir>/<kịch bản>/{launch.log,sim.csv,client.json}`. Thiết kế + kết quả: `replanning.md`.

## 4. Rollout phần cứng — NGƯỜI VẬN HÀNH thực hiện, từng bước cần xác nhận

Điều kiện trước: phần 1–3 PASS; đọc `requirements_validation.md` (mục NOT_VERIFIED).

1. **Đo trước khi chạy controller** (teleop, bánh kê / sàn trống, E-stop):
   - Gia tốc/giảm tốc thật (ghi `/odometry/filtered` khi bước lệnh 0 → 0.15 → 0): điền `acc_lim_x`, `decel_lim_x`,
     `acc_lim_theta`, `decel_lim_theta` (nhỏ hơn hoặc bằng giá trị đo).
   - Trễ lệnh → chuyển động (thời điểm lệnh trên `/cmd_vel_out` so với thay đổi `/wheel_feedback`): điền
     `control_latency` (lấy phân vị cao, không lấy trung bình).
   - Quãng dừng ở 0.15 m/s: so với `v(τ+dt) + v²/(2·decel) + stopping_margin`.
   - Footprint thật (laptop, cáp, giá đỡ) so với `robot_radius 0.20 + padding 0.10`; tâm quay so với tâm thân.
2. **Motor cách ly** (robot bật, `navigation_enabled:=false`):
   ```bash
   ros2 launch stage3_navigation navigation.launch.py     # mặc định: CustomDWB, /cmd_vel_dwb (không tới motor)
   # RViz: 2D Pose Estimate -> 2D Goal Pose; rồi:
   ros2 service call /goal_path_bridge/follow std_srvs/srv/Trigger
   ros2 topic echo /cmd_vel_dwb ; ros2 topic echo /CustomDWB/state
   ros2 service call /goal_path_bridge/cancel std_srvs/srv/Trigger
   ```
   Kiểm tra: local costmap với `/scan_filtered` thật (nửa sau NaN), `compute_ms` p99 < 80 ms trên máy này,
   `missed_deadline`/`over_budget` ≈ 0, lệnh không vượt giới hạn, không có lệnh lùi.
3. **Bánh kê khỏi mặt đất + E-stop**, `navigation_enabled:=true` (chỉ khi người dùng xác nhận).
4. **Chạy trên sàn tốc độ thấp** (vd. `ros2 param set /controller_server CustomDWB.max_vel_x 0.08`), khu vực trống,
   rồi với vật cản tĩnh; dừng bằng `/goal_path_bridge/cancel`, Ctrl+C hoặc E-stop cứng. PS4 chỉ ghi đè tạm thời.

Rollback: mã trước = commit `e6561b4` (`git checkout e6561b4 -- stage3_navigation`); hoặc giữ mã mới và dùng
`local_planner:=nav2_dwb` (Nav2 DWB không đổi).
