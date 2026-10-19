# `stage3_navigation`: lập đường A\* + DWB tự né vật cản

Package này giúp robot **tự đi tới một đích** chọn trên RViz, theo hai lớp:

| Lớp | Làm gì | Biết gì |
|---|---|---|
| **A\*** (toàn cục) | Tìm **một** đường từ robot tới đích, **một lần** cho mỗi goal | Chỉ bản đồ tĩnh (`/map`) |
| **DWB** (cục bộ) | Mỗi 0.1 s chọn vận tốc (v, ω) để bám đường A\* và **tự né vật cản** | Vật cản LiDAR đang thấy (local costmap 2 × 2 m) |

- **Không có replanning.** Đường A\* không đổi trong cả lần bám. Vật cản mới chỉ do DWB xử lý (mục 9).
- Bộ điều khiển mặc định là **CustomDWB** (`stage3_navigation::DwbController`, tự viết). Nav2 DWB vẫn giữ để so sánh.
- Tính năng replan (commit `0d553eb`, 2026-10-15) đã được gỡ ngày 2026-10-19. Code và tham số stage 3 = bản trước đó
  (`a7c59a1`), cộng cổng serial mới.

> ⚠️ **Mọi launch ở đây đều include Stage 2**, nên bật LiDAR, IMU, **motor** và tay PS4 (trừ `stage2:=false`).
> `navigation_enabled:=false` (mặc định) chỉ ngắt **lệnh của DWB** khỏi motor; tay PS4 vẫn lái được.
> `navigation_enabled:=true` (DWB lái motor thật): kê bánh / giữ E-stop.

Tài liệu sâu hơn trong `doc/`:
- `audit.md`: các lỗi đã sửa ở CustomDWB.
- `offline_runbook.md`: cách chạy test và benchmark.
- `requirements_validation.md`: kiểm chứng yêu cầu.
- `sources.md`: nguồn tham khảo.
- `p2_proposals.md`: đề xuất chưa làm.

---

## Mục lục
1. Các file chính
2. Kiến trúc và luồng dữ liệu
3. `planning.launch.py`
4. `navigation.launch.py`
5. Các trường hợp chạy
6. `nav2_params.yaml`: tham số
7. `goal_path_bridge`: phân tích chi tiết
8. CustomDWB: phân tích chi tiết
9. DWB tự né vật cản
10. Các tình huống và cách hệ thống phản ứng
11. Kết quả trên robot
12. Theo dõi và debug
13. Test và công cụ
14. Giới hạn đã biết

---

## 1. Các file chính

| File | Vai trò |
|---|---|
| `launch/planning.launch.py` | Stage 2 + A\* + RViz (chỉ lập đường, **không** có DWB) |
| `launch/navigation.launch.py` | `planning.launch.py` + DWB + local costmap |
| `config/nav2_params.yaml` | Mọi tham số: planner, 2 costmap, bridge, controller, DWB |
| `src/goal_path_bridge.cpp` | Node điều phối: goal → A\* → FollowPath, giám sát |
| `src/dwa_core.cpp` | Thuật toán CustomDWB (không phụ thuộc ROS) |
| `src/dwb_controller.cpp` | Vỏ ROS của CustomDWB (plugin Nav2) |
| `include/stage3_navigation/dwa_core.hpp` | Tham số mặc định `DwbParams`, khai báo thuật toán |
| `include/stage3_navigation/dwb_controller.hpp` | Khai báo plugin |
| `dwb_controller_plugin.xml` | Đăng ký plugin với pluginlib |
| `rviz/planning.rviz` | RViz duy nhất của stage 3 |
| `scripts/analyze_cmd_lag.py` | Đo độ trễ lệnh → vận tốc thật từ bag |
| `test/` | gtest, mô phỏng kín vòng, benchmark offline |
| `doc/` | Tài liệu thiết kế, audit, benchmark |

---

## 2. Kiến trúc và luồng dữ liệu

### Sơ đồ tổng

```text
                       (Stage 2: AMCL hoặc slam_toolbox)        (Stage 1: EKF)
                          /map, TF map → odom              TF odom → base_footprint, /odometry/filtered
                                   │                                     │
RViz 2D Goal Pose ──/goal_pose──> goal_path_bridge ──action /compute_path_to_pose──> planner_server (A*)
                                   │   ▲                                               │ global_costmap
                                   │   └────────────── đường A* ───────────────────────┘ (static + inflation)
                                   │
                                   ├──> /global_path (RViz)
                                   │
            (chỉ khi ~/follow hoặc auto_follow)
                                   │
                                   └──action /follow_path──> controller_server (10 Hz)
                                                              │  plugin CustomDWB hoặc FollowPath (Nav2 DWB)
                                                              │  local_costmap ← /scan_filtered (né vật cản)
                                                              ▼
                                          "cmd_vel" ──> /cmd_vel_dwb  (navigation_enabled:=false, không ai nghe)
                                                   └──> /cmd_vel_nav  (navigation_enabled:=true)
                                                              │
                                                    twist_mux (ưu tiên 10; PS4 = 100)
                                                              │
                                                       /cmd_vel_out → zlac8015d_move → motor

/localization/status (monitor) ──> goal_path_bridge   (cho phép / hủy bám đường)
/global_costmap/costmap        ──> goal_path_bridge   (giải thích khi A* thất bại)
```

### Node của stage 3

| Node | Có trong | Loại |
|---|---|---|
| `planner_server` (+ `global_costmap`) | planning, navigation | lifecycle |
| `lifecycle_manager_planning` | planning, navigation | – |
| `goal_path_bridge` | planning (A\* only) hoặc navigation (có FollowPath) | node thường |
| `controller_server` (+ `local_costmap`) | navigation | lifecycle |
| `lifecycle_manager_navigation` | navigation | – |
| `rviz2_planning` | cả hai (`rviz:=true`) | – |

Stage 3 **không phát TF**, không có BT navigator, không gọi `NavigateToPose`. Mọi thứ do `goal_path_bridge` điều phối.

### Topic, action, service chính

| Tên | Loại | Từ → tới |
|---|---|---|
| `/goal_pose` | topic `PoseStamped` | RViz → bridge |
| `/compute_path_to_pose` | action | bridge → planner_server |
| `/global_path` | topic `Path` (transient_local) | bridge → RViz |
| `/plan` | topic `Path` | planner_server (tự phát) |
| `/follow_path` | action | bridge → controller_server |
| `/goal_path_bridge/follow` | service `Trigger` | người dùng → bridge (bắt đầu bám) |
| `/goal_path_bridge/cancel` | service `Trigger` | người dùng → bridge (dừng) |
| `/goal_path_bridge/follow_state` | topic `String` (transient_local) | bridge → người dùng |
| `/cmd_vel_dwb` hoặc `/cmd_vel_nav` | topic `Twist` | controller_server → (không ai) / twist_mux |
| `/local_plan` | topic `Path` | plugin đang chạy → RViz |
| `/CustomDWB/state` | topic `DiagnosticStatus` | CustomDWB → debug |
| `/CustomDWB/trajectories` | topic `MarkerArray` | CustomDWB → RViz |
| `/CustomDWB/local_path` | topic `Path` | CustomDWB → RViz |

### Lifecycle (thứ tự bật)
- `lifecycle_manager_planning` bật `planner_server`. `lifecycle_manager_navigation` bật `controller_server`.
  Cả hai `autostart: true`.
- Costmap chỉ khởi động xong khi có TF tới `base_footprint`:
  - global costmap cần `map → base_footprint`. Ở chế độ localization, TF này **chỉ có sau khi đặt 2D Pose Estimate**
    (AMCL mới phát `map → odom`);
  - vì thế `bond_timeout: 0.0` (tắt bond), để lifecycle manager không coi việc chờ TF là node chết.
- Chưa đặt pose thì gửi goal sẽ bị báo `planner_server action not available`.

---

## 3. `launch/planning.launch.py`: chỉ lập đường A\*

**Vai trò:** chạy **một** bringup Stage 2 + `planner_server` + `global_costmap` + `goal_path_bridge` (A\* only) + RViz.
Không có lệnh vận tốc nào.

```bash
ros2 launch stage3_navigation planning.launch.py                              # localization (mặc định)
ros2 launch stage3_navigation planning.launch.py mode:=mapping allow_unknown:=true
```

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `mode` | `localization` | `localization` (AMCL) hoặc `mapping` (slam_toolbox); khác → lỗi |
| `serial_port` | `/dev/ttyUSB2` | Cổng LiDAR |
| `imu_port` | `/dev/ttyUSB1` | Cổng IMU |
| `map` | `~/capstone_ws/capstone_project/maps/map.yaml` | Bản đồ (localization) |
| `csm` | **`true`** | 2D Pose Estimate qua CSM (localization) |
| `monitor` | **`true`** | Chạy `localization_monitor` (localization) |
| `params_file` | `config/nav2_params.yaml` | File tham số |
| `allow_unknown` | `false` | A\* được đi qua vùng unknown |
| `stage2` | `true` | `false`: không chạy Stage 2 (test offline) |
| `goal_path_bridge` | `true` | `navigation.launch.py` đặt `false` (chạy bridge riêng) |
| `rviz` | `true` | Mở `planning.rviz` |
| `log_level` | `info` | Mức log |

Cách hoạt động:
- Hai `include_stage2(...)` có điều kiện: chỉ đúng **một** cái chạy theo `mode`. Nhờ vậy chỉ có một nguồn
  `map → odom`, một driver motor, một EKF.
- Stage 2 bị đặt `rviz:=false`: chỉ có một RViz.
- `csm` và `monitor` chỉ có tác dụng khi `mode:=localization`. Ở stage 3 chúng **mặc định bật**; ở stage 2 mặc
  định tắt.
- `allow_unknown` ghi đè `GridBased.allow_unknown` của planner.
- RViz: topic `/initialpose` (công cụ 2D Pose Estimate) được remap:
  - `mode:=localization csm:=true` → `/initialpose_guess` (qua CSM);
  - còn lại → `/initialpose` (thẳng vào AMCL; ở mapping không ai dùng).
- Bridge chạy với `auto_follow: false` (ở đây không có controller).

---

## 4. `launch/navigation.launch.py`: A\* + DWB

**Vai trò:** include `planning.launch.py` (với `goal_path_bridge:=false`), rồi thêm `controller_server`,
`lifecycle_manager_navigation` và một `goal_path_bridge` có FollowPath.

```bash
ros2 launch stage3_navigation navigation.launch.py                          # CustomDWB, cách ly motor
ros2 launch stage3_navigation navigation.launch.py local_planner:=nav2_dwb  # Nav2 DWB để so sánh
ros2 launch stage3_navigation navigation.launch.py navigation_enabled:=true # DWB lái motor thật
```

Có đủ đối số chung của `planning.launch.py` (truyền nguyên xuống), cộng thêm:

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `local_planner` | `custom` | `custom` = CustomDWB; `nav2_dwb` = Nav2 DWB; khác → lỗi |
| `navigation_enabled` | `false` | `true`: lệnh DWB → `/cmd_vel_nav` → motor |
| `auto_follow` | `false` | `true`: A\* xong là bám đường ngay |

Những gì launch tự đặt:

| Đặt gì | Giá trị |
|---|---|
| Remap `cmd_vel` của controller_server | `/cmd_vel_nav` nếu `navigation_enabled`, ngược lại `/cmd_vel_dwb` |
| Bridge `controller_id` | `CustomDWB` (custom) / `FollowPath` (nav2_dwb) |
| Bridge `require_localized` | `true` chỉ khi `mode:=localization` **và** `monitor:=true` |
| Bridge `auto_follow` | = `auto_follow` |

- Khi khởi động, launch in một dòng `LogInfo` cho biết chế độ vận tốc (cách ly hay lái motor).
- `controller_server` luôn nạp **cả hai** plugin. `controller_id` trong goal FollowPath chọn plugin nào chạy.

---

## 5. Các trường hợp chạy

| # | Lệnh | Robot làm gì | Dùng khi |
|---|---|---|---|
| A | `planning.launch.py` | Chỉ vẽ đường A\* | Kiểm tra bản đồ, A\* |
| B | `planning.launch.py mode:=mapping allow_unknown:=true` | A\* trên bản đồ đang vẽ | Thử A\* khi chưa có bản đồ |
| C | `navigation.launch.py` | DWB tính lệnh nhưng **không tới motor** | Xem lệnh DWB an toàn |
| D | `navigation.launch.py navigation_enabled:=true` | DWB **lái motor** | Thử thật (kê bánh / E-stop) |
| E | `... local_planner:=nav2_dwb` | Như C/D với Nav2 DWB | So sánh |
| F | `... auto_follow:=true` | A\* xong là bám ngay | Bỏ bước gọi `follow` |
| G | `... monitor:=false` | Không đòi LOCALIZED | Thử DWB né vật cản (mục 9) |
| H | `... stage2:=false rviz:=false` | Không phần cứng | Test offline (nguồn dữ liệu ngoài) |

### Quy trình chuẩn (C hoặc D)
1. Chạy launch. Chờ log `Managed nodes are active` (có thể chỉ xuất hiện sau bước 2).
2. RViz **2D Pose Estimate**: click vị trí thật của robot. CSM chỉnh lại pose, AMCL bắt đầu phát TF.
3. Chờ `/localization/status` = `LOCALIZED`.
4. RViz **2D Goal Pose**: chọn đích. Thấy `/global_path` hiện lên.
5. Bắt đầu bám (bỏ qua bước này nếu `auto_follow:=true`):
   ```bash
   ros2 service call /goal_path_bridge/follow std_srvs/srv/Trigger
   ```
6. Theo dõi:
   ```bash
   ros2 topic echo /goal_path_bridge/follow_state
   ```
   Ở chế độ C, xem thêm `ros2 topic echo /cmd_vel_dwb`.
7. Dừng:
   ```bash
   ros2 service call /goal_path_bridge/cancel std_srvs/srv/Trigger
   ```
   Hoặc Ctrl+C, hoặc E-stop.

### Lưu ý cho chế độ C (cách ly)
Robot không chạy thật, nên vận tốc đo luôn khoảng 0. DWB vẫn ra lệnh, nhưng **sau 10 s progress checker báo
`Failed to make progress`** → `ABORTED`. **Đây là bình thường.** Chế độ này dùng để xem lệnh đầu tiên, pha xoay và
quỹ đạo trên RViz.

### Rollout trên robot (theo thứ tự)
1. Chế độ C: xem `/cmd_vel_dwb` và RViz.
2. Chế độ D với **bánh kê khỏi mặt đất**, tay ở E-stop.
3. Chế độ D trên sàn, đi chậm, khu vực trống.
4. Thử né vật cản: chế độ D + `monitor:=false`, có người giám sát (mục 9).

---

## 6. `config/nav2_params.yaml`: tham số

### 6.1 `planner_server` (A\*)

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `planner_plugins` | `["GridBased"]` | Một planner |
| `GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | NavFn của Nav2 |
| `use_astar` | `true` | A\* (không phải Dijkstra) |
| `allow_unknown` | `false` | Không đi qua vùng unknown |
| `tolerance` | `0.0` | Đích phải ở ô hợp lệ; không tự dời đích |
| `use_final_approach_orientation` | `false` | Hướng cuối = hướng Goal Pose |
| `expected_planner_frequency` | 1.0 | Chỉ để cảnh báo |

A\* chạy **chỉ khi có goal mới**. Không lập lại định kỳ, không lập lại khi gặp vật cản.

### 6.2 `global_costmap` (cho A\*)

| Tham số | Giá trị |
|---|---|
| `global_frame` / `robot_base_frame` | `map` / `base_footprint` |
| `robot_radius` | 0.20 m |
| `resolution` | 0.05 m |
| `rolling_window` | `false` (kích thước theo `/map`) |
| `update_frequency` / `publish_frequency` | 1.0 / 1.0 Hz |
| `transform_tolerance` | 0.5 s |
| `track_unknown_space` | `true` |
| `trinary_costmap` | `true` |
| `lethal_cost_threshold` / `unknown_cost_value` | 100 / 255 |
| `always_send_full_costmap` | `true` |
| `plugins` | `static_layer`, `inflation_layer` |

- **`static_layer`**: lấy `/map`, transient_local, không nhận `/map_updates`.
- **`inflation_layer`**: `inflation_radius 0.55`, `cost_scaling_factor 3.0`, không inflate vùng unknown.

Hệ quả:
- A\* **chỉ thấy bản đồ tĩnh**, không biết vật cản mới (người, thùng…).
- Ô cách tường < 0.20 m là inscribed (99), A\* coi như vật cản → A\* đi được khe **≥ 0.40 m**.
- Đường A\* ưu tiên đi giữa lối, cách xa tường (nhờ inflation tới 0.55 m).

### 6.3 `controller_server`

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `controller_frequency` | 10 Hz | Một chu kỳ điều khiển = 0.1 s |
| `odom_topic` | `/odometry/filtered` | Vận tốc đo |
| `min_x_velocity_threshold` | 0.001 | Vận tốc đo nhỏ hơn → 0 |
| `min_theta_velocity_threshold` | 0.001 | – |
| `min_y_velocity_threshold` | 0.5 | Robot vi sai: bỏ vy |
| `failure_tolerance` | 0.3 s | Plugin lỗi liên tục quá 0.3 s → abort |
| `controller_plugins` | `["FollowPath", "CustomDWB"]` | Nạp cả hai |

**Progress checker** (`SimpleProgressChecker`):
- `required_movement_radius` 0.25 m, `movement_time_allowance` 10 s.
- Không đi được 0.25 m trong 10 s → abort `Failed to make progress`.
- **Xoay tại chỗ không tính là đi.**

**Goal checker** (`SimpleGoalChecker`, tên `general_goal_checker`):
- `xy_goal_tolerance` 0.15 m, `yaw_goal_tolerance` 0.25 rad (~14°).
- `stateful: true`: lần đầu vào trong 0.15 m thì chốt vị trí, sau đó chỉ còn xét hướng.

### 6.4 `CustomDWB` (`stage3_navigation::DwbController`)

**Giới hạn động học** (giá trị khởi đầu, chưa đo đầy đủ):

| Tham số | Giá trị |
|---|---|
| `max_vel_x` | 0.15 m/s |
| `min_vel_x` | 0.0 (**không lùi**: phía sau không có LiDAR) |
| `max_vel_theta` | 0.5 rad/s |
| `min_in_place_vel_theta` | 0.1 rad/s |
| `acc_lim_x` / `decel_lim_x` | 0.5 / 0.5 m/s² |
| `acc_lim_theta` / `decel_lim_theta` | 1.0 / 1.0 rad/s² |

**Lấy mẫu và mô phỏng:**

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `vx_samples` × `vtheta_samples` | 20 × 21 | ≈ 420–440 ứng viên mỗi chu kỳ |
| `sim_time` | 1.7 s | Nhìn trước (0.15 m/s → ~0.26 m) |
| `sim_granularity` | 0.05 s | Bước mô phỏng tối đa |
| `control_latency` | 0.2 s | Trễ lệnh → bánh (đo từ bag 2026-10-14) |
| `stopping_margin` | 0.05 m | Khoảng trống thêm sau khi phanh |

**Vận tốc gốc của cửa sổ:**

| Tham số | Giá trị |
|---|---|
| `velocity_reference` | `command` |
| `max_feedback_gap_x` | 0.15 m/s |
| `max_feedback_gap_theta` | 0.5 rad/s |

**Bám đường:**

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `local_plan_length` | 1.5 m | Đoạn đường phía trước được xét |
| `carrot_distance` | 0.6 m | Điểm dẫn hướng ("cà rốt") |
| `closest_search_distance` | 1.0 m | Cửa sổ tìm tiến trình |
| `max_path_deviation` | 0.8 m | Lệch đường hơn → dừng |

**Pha xoay và đích:**

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `rotate_to_heading_angle` | 0.8 rad | Lệch hơn → xoay tại chỗ (ra khi < 0.4) |
| `rotate_gain` | 1.5 | ω = gain × sai số góc |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.15 / 0.25 | Dự phòng (thật lấy từ goal checker) |
| `goal_tolerance_fraction` | 0.9 | Dùng 0.9 × dung sai bên trong |
| `goal_latch_release_margin` | 0.05 m | Trôi ra ngoài xy + 0.05 → tiếp cận lại |

**Trọng số chi phí** (nhỏ hơn = tốt hơn):

| Tham số | Giá trị |
|---|---|
| `path_distance_scale` | 4.0 |
| `carrot_distance_scale` | 2.0 |
| `heading_scale` | 1.0 |
| `obstacle_scale` | 0.5 |
| `speed_scale` | 0.3 |
| `oscillation_scale` | 1.0 |

**Chống dao động:** reset sau `oscillation_reset_dist` 0.05 m, `oscillation_reset_angle` 0.2 rad, hoặc
`oscillation_reset_time` 2.0 s.

**An toàn dữ liệu:**

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `transform_tolerance` | 0.3 s | Thời gian **chờ** TF |
| `max_tf_age` | 1.0 s | Tuổi tối đa TF `map → odom` |
| `max_pose_age` | 0.5 s | Tuổi tối đa TF `odom → base_footprint` |
| `odom_timeout` | 0.5 s | `/odometry/filtered` im lặng hơn → dừng |
| `collision_check` | `auto` | Footprint tròn → chỉ xét ô tâm |
| `unknown_is_obstacle` | `true` | Không có tác dụng (local costmap không theo dõi unknown) |

**Debug:** `publish_trajectories: true`, `trajectory_marker_stride: 4`, `debug_publish_rate: 5.0` Hz.

Cả 45 tham số được kiểm tra khi configure và khi `ros2 param set`:
- phải hữu hạn và nằm trong miền giá trị;
- phải thỏa quan hệ với nhau (ví dụ `carrot_distance ≤ local_plan_length`);
- tải ≤ 100 000 pose mô phỏng mỗi chu kỳ.

Sai một cái thì cả lô bị từ chối, giữ giá trị cũ:

```bash
ros2 param set /controller_server CustomDWB.max_vel_x 0.12
```

### 6.5 `FollowPath` (Nav2 DWB, để so sánh)
- Cùng giới hạn: `max_vel_x` 0.15, `max_vel_theta` 0.5, `acc_lim_x` 0.5, `decel_lim_x` −0.5 (Nav2 dùng số âm),
  `acc_lim_theta` 1.0, `sim_time` 1.7, `vx_samples` 20, `vtheta_samples` 20.
- Critics mặc định Humble: `RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist`.
- Trọng số: `PathAlign` 32, `GoalAlign` 24, `PathDist` 32, `GoalDist` 24, `RotateToGoal` 32, `BaseObstacle` 0.02.
- Khác CustomDWB: không có watchdog odometry, không mô phỏng trễ và đuôi phanh. Benchmark offline: khi odometry
  ngừng, Nav2 DWB vẫn chạy tiếp tới đích.

### 6.6 `local_costmap` (cho DWB)

| Tham số | Giá trị |
|---|---|
| `global_frame` | `odom` |
| Kích thước | 2 × 2 m, rolling window quanh robot |
| `resolution` | 0.05 m |
| `robot_radius` + `footprint_padding` | 0.20 + 0.10 m (nội tiếp **0.30 m**) |
| `update_frequency` / `publish_frequency` | 5.0 / 2.0 Hz |
| `transform_tolerance` | 0.3 s |
| `track_unknown_space` | `false` |
| `plugins` | `obstacle_layer`, `inflation_layer` |
| obstacle: nguồn | `/scan_filtered`, đánh dấu 2.5 m / xóa 3.0 m, `inf_is_valid: false` |
| obstacle: `expected_update_rate` | 0.5 s (scan cũ hơn → costmap "không current") |
| inflation | `inflation_radius 0.45`, `cost_scaling_factor 3.0` |

Lý do chọn:
- **2 × 2 m (không phải 3 × 3):** Nav2 DWB lấy đích cục bộ = điểm cuối đường A\* còn trong costmap. Với 3 × 3 m, ở
  đầu tường đích cục bộ nằm sau tường → robot đứng yên → abort (tái hiện offline).
- **Padding 0.10 m:** không có padding thì khi test offline DWB lái sát tới chạm vật cản chưa có trong bản đồ.
- **Đánh đổi:** DWB không qua được khe < ~0.6 m, trong khi A\* vẫn lập đường qua khe ≥ 0.4 m.
- Nửa sau scan là NaN, mà `track_unknown_space: false` → vùng phía sau được coi là **trống**. Bảo vệ thực sự là
  **không cho lùi**.

### 6.7 `goal_path_bridge`

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `global_frame` / `robot_base_frame` | `map` / `base_footprint` | – |
| `planner_id` | `GridBased` | – |
| `tf_timeout` | 0.5 s | TF cũ hơn: cảnh báo (A\*) / từ chối (FollowPath) |
| `controller_id` | `FollowPath` (launch ghi đè `CustomDWB`) | Plugin DWB |
| `goal_checker_id` | `general_goal_checker` | – |
| `auto_follow` | `false` | – |
| `max_path_age` | 30 s | Đường cũ hơn → từ chối |
| `max_start_distance` | 0.5 m | Robot cách đường xa hơn → từ chối |
| `follow_timeout` | 120 s | Quá thời gian → hủy |
| `require_localized` | `true` (launch tính lại) | Đòi LOCALIZED |
| `localization_status_timeout` | 2.0 s | Status cũ hơn → coi như mất |
| `localization_uncertain_grace` | 2.0 s | WARN ngắn hơn được bỏ qua |

Các tham số FollowPath **được đọc lại mỗi lần dùng**, nên `ros2 param set /goal_path_bridge ...` có tác dụng ngay.

---

## 7. `goal_path_bridge`: phân tích chi tiết

### 7.1 Nhận goal (`onGoal`)
1. `frame_id` rỗng thì gán `map`.
2. **Hủy FollowPath đang chạy** (goal mới luôn dừng robot trước) và xóa đường cũ.
3. Chưa nhận `/map` → lỗi `no map on /map`.
4. Không có TF `map → base_footprint` → lỗi (gợi ý đặt 2D Pose Estimate). TF cũ hơn 0.5 s chỉ **cảnh báo**.
5. `planner_server` chưa sẵn sàng → lỗi.
6. Gửi `ComputePathToPose` (`use_start: false`: điểm đầu = pose hiện tại của robot).

Kết quả A\*:
- Thuộc goal cũ (đã có goal mới) → bỏ qua.
- **Thành công** → publish `/global_path`, lưu đường + thời điểm. Nếu `auto_follow` thì bắt đầu bám ngay.
- **Thất bại** → publish đường rỗng (xóa RViz) + log lý do, đọc từ global costmap:

| Thông báo | Nghĩa |
|---|---|
| `Goal is in an obstacle (lethal)` | Đích nằm trên vật cản |
| `Goal is closer than the robot radius ... (inscribed)` | Đích quá sát tường (< 0.20 m) |
| `Goal is in unknown space` | Đích ở vùng chưa quét |
| `Goal is outside the map` | Đích ngoài bản đồ |
| `Both cells are free: goal is probably enclosed ...` | Không có đường nối (bị bao kín, hoặc chỉ qua unknown) |

### 7.2 Bắt đầu bám đường (`startFollow`)
Bridge **từ chối** (state `REFUSED: ...`) theo thứ tự:
1. Đang có FollowPath chạy (gọi `cancel` trước).
2. Chưa có đường A\* hợp lệ.
3. Đường cũ hơn 30 s.
4. Không có TF, hoặc TF cũ hơn 0.5 s.
5. Robot cách đường > 0.5 m (ví dụ đã lái PS4 đi chỗ khác).
6. `require_localized` mà status không có / cũ hơn 2 s / **khác OK**. Lúc bắt đầu, WARN cũng bị từ chối.
7. `/follow_path` chưa sẵn sàng.

Qua hết thì gửi **đúng một** goal FollowPath với đường A\* đã lưu. Đường này không đổi tới khi kết thúc.

### 7.3 Trạng thái trên `/goal_path_bridge/follow_state`

| Trạng thái | Nghĩa |
|---|---|
| `IDLE` | Vừa khởi động |
| `FOLLOW_REQUESTED` | Đã gửi FollowPath, chờ server nhận |
| `FOLLOWING` | DWB đang bám |
| `REFUSED: <lý do>` | `follow` bị từ chối |
| `REJECTED` | controller_server không nhận goal |
| `SUCCEEDED` | Tới đích |
| `CANCELED` / `CANCELED: <lý do>` | Bị hủy (người dùng, goal mới, timeout, localization, tắt node) |
| `ABORTED` | controller_server bỏ cuộc (DWB không có quỹ đạo, không tiến, TF/costmap lỗi) |

### 7.4 Giám sát (timer 200 ms, `watchFollow`, chỉ khi đang bám)
1. **Quá 120 s** từ lúc gửi FollowPath → hủy.
2. **Localization** (khi `require_localized`):
   - `LOST` hoặc status cũ > 2 s → hủy **ngay**;
   - `UNCERTAIN` / `INITIALIZING` liên tục ≤ 2 s → bỏ qua (chỉ log);
   - WARN > 2 s → hủy.

### 7.5 Kết quả FollowPath

| Kết quả | Bridge xử lý |
|---|---|
| SUCCEEDED | `SUCCEEDED` |
| CANCELED | `CANCELED` |
| ABORTED | `ABORTED`. **Không thử lại.** Robot dừng (controller_server phát 0); gửi goal mới để đi tiếp |

### 7.6 Tắt node
Bridge tự bắt Ctrl+C / SIGTERM, gửi cancel FollowPath, chờ 0.5 s cho yêu cầu kịp đi, rồi mới thoát. controller_server
khi bị hủy sẽ phát lệnh 0.

---

## 8. CustomDWB: phân tích chi tiết

### 8.1 Khi nhận đường (`setPlan`)
- Xóa trạng thái theo đường: chốt đích, pha xoay, chiều xoay, chống dao động.
- `frame_id` rỗng, hoặc hướng đích không hợp lệ → lỗi → goal bị abort.
- Gộp các điểm trùng liên tiếp, tính độ dài cung tích lũy, đặt tiến trình = 0.
- Chỉ dùng hướng của **điểm cuối** (NavFn không đặt hướng cho các điểm giữa).

### 8.2 Một chu kỳ (`computeVelocityCommands`, 10 Hz)

**(a) Kiểm tra đầu vào.** Lỗi nào cũng → lệnh 0 + báo lỗi:
- không có đường;
- pose hoặc vận tốc không hữu hạn;
- frame pose khác `odom`;
- TF `odom → base_footprint` cũ hơn 0.5 s;
- `/odometry/filtered` chưa có hoặc im lặng > 0.5 s.

Hai chu kỳ cách nhau > 0.25 s (bị ngắt quãng) → quên lệnh trước, lấy vận tốc đo làm gốc.

**(b) Một TF `map → odom` duy nhất cho cả chu kỳ.**
- Chờ tối đa 0.3 s; tuổi > 1.0 s → lỗi.
- AMCL đóng dấu TF vào **tương lai** (+1 s), nên tuổi âm là bình thường.
- Dùng chung TF này cho đường cục bộ và đích, nên hai thứ luôn khớp nhau.

**(c) Bám tiến trình dọc đường (`PathTracker`).**
- Chỉ tìm hình chiếu trong 1.0 m phía trước tiến trình hiện tại. Lấy **cực tiểu đầu tiên**, nên với đường chữ U /
  tự cắt, robot không nhảy sang nhánh kia.
- **Tiến trình không bao giờ giảm.**
- Lệch đường > 0.8 m → lỗi.
- Đường cục bộ = 1.5 m tiếp theo, chuyển sang `odom`, **cắt ở mép local costmap** (thực tế ~1–1.4 m).

**(d) Dung sai đích:** lấy từ goal checker (0.15 m / 0.25 rad).

**(e) Chọn pha**, theo thứ tự ưu tiên:

```text
             d < 0.135 m                                lệch hướng > 0.8 rad
FOLLOW_PATH ───────────> ROTATE_TO_GOAL     FOLLOW_PATH ─────────────────> ROTATE_TO_PATH
            <───────────                                <─────────────────
             d > 0.20 m, hoặc đúng hướng mà d > 0.135     lệch hướng < 0.4 rad
```

Trước khi chọn pha:
- Ô dưới tâm robot ≥ 253 (cách vật cản < 0.30 m), unknown, hoặc ngoài costmap → `BLOCKED`, lỗi.
- **Robot đứng quá sát vật cản thì không ra lệnh nào, kể cả xoay.** Không lùi được nên không tự thoát.

**Vận tốc gốc** (`velocity_reference: command`):
- Gốc = lệnh trước, kẹp trong ±0.15 m/s và ±0.5 rad/s quanh vận tốc đo.
- Lý do: phản hồi trễ ~0.2–0.7 s. Nếu tính từ vận tốc đo thì gia tốc thực rất nhỏ, gây quay lố ~50° (robot
  2026-10-14).
- Kẹp quanh vận tốc đo để gốc không trôi xa thực tế khi PS4 chiếm quyền hoặc bánh bị kẹt.

**Pha 1: `ROTATE_TO_GOAL`** (ưu tiên cao nhất)
- Vào khi khoảng cách tới đích < 0.135 m (0.9 × 0.15).
- Lệch hướng > 0.225 rad → xoay tại chỗ về hướng đích.
- Đúng hướng và d ≤ 0.135 → giảm (v, ω) về 0, chờ goal checker báo xong.
- Đúng hướng nhưng 0.135 < d ≤ 0.20, hoặc trôi ra > 0.20 m → quay lại tiếp cận.

**Pha 2: `ROTATE_TO_PATH`**
- "Cà rốt" = điểm cách đầu đường cục bộ 0.6 m.
- Hướng tới cà rốt lệch > 0.8 rad → xoay tại chỗ, tới khi < 0.4 rad.
- Ví dụ: đích ở sau lưng → xoay ~180° trước rồi mới đi.

**Cách xoay** (chung cho hai pha xoay):
- Gần ±π: **giữ chiều đã chọn**, để nhiễu không làm robot lật trái/phải.
- `còn lại = |sai số| − phần đã quay do trễ (0.2 s + 0.05 s)`.
- `|ω| = min(1.5 × còn lại, √(2 × 1.0 × còn lại), 0.5)`, tối thiểu 0.1.
- Thực tế: chạy 0.5 rad/s tới khi còn ~0.33 rad thì chậm dần. ω đổi tối đa 0.1 rad/s mỗi chu kỳ.
- v giảm dần về 0 theo `decel_lim_x`.
- Lệnh xoay cũng được **mô phỏng và kiểm tra va chạm** như mọi quỹ đạo khác.

**Pha 3: `FOLLOW_PATH`** (DWA)
1. **Cửa sổ vận tốc:** v ∈ [gốc ± 0.05] ∩ [0, 0.15]; ω ∈ [gốc ± 0.1] ∩ [−0.5, 0.5].
   - Ví dụ: gốc (0.10, 0) → v ∈ [0.05, 0.15], ω ∈ [−0.1, 0.1].
   - Vận tốc đang vượt giới hạn (vừa lái PS4 nhanh) → lệnh = giới hạn gần nhất, cờ `overspeed`.
2. **Lấy mẫu:** 20 × 21 giá trị, luôn thêm v = 0 và ω = 0.
3. **Mô phỏng** mỗi ứng viên, bắt đầu từ vận tốc **đo**:
   - 0 → 0.2 s: lệnh trước vẫn đang tác dụng (trễ);
   - 0.2 → 0.3 s: chuyển sang ứng viên;
   - 0.3 → 1.7 s: giữ ứng viên;
   - **đuôi phanh**: từ 0.3 s phanh về 0, cộng thêm 0.05 m phía trước;
   - bước ≤ 0.05 s và quét ≤ nửa ô (0.025 m), để không lọt ô nào.
4. **Loại cứng:** pose nào (kể cả pose đầu và đuôi phanh) chạm ô ≥ 253, unknown, hoặc ra ngoài costmap → loại.
   Đây là điều kiện "chỉ chọn vận tốc phanh kịp" (Fox 1997).
5. **Chấm điểm mềm** (ứng viên hợp lệ):

| Thành phần | Đo gì | Trọng số |
|---|---|---|
| path | Cuối quỹ đạo cách đường bao xa (m) | 4.0 |
| carrot | Cuối quỹ đạo cách cà rốt bao xa (m) | 2.0 |
| heading | Lệch hướng tới cà rốt ở cuối quỹ đạo, /π | 1.0 |
| obstacle | Cost lớn nhất dọc cung 0.255 m cố định, /252 | 0.5 |
| speed | (0.15 − v) / 0.15 | 0.3 |
| oscillation | 1 nếu đổi chiều xoay ngay sau lần đổi trước | 1.0 |

   Cung obstacle có **độ dài cố định**, không phụ thuộc tốc độ. Nếu đo theo thời gian, ứng viên chậm "thấy" ít vùng
   gần vật cản hơn nên luôn rẻ hơn, và robot bò dần về 0 ở mép vùng đó (lỗi B1 trong `doc/audit.md`).
6. Chọn ứng viên rẻ nhất. Không còn ứng viên nào → `BLOCKED` → lỗi.
7. Có ứng viên nhưng **không cái nào có v > 0** (vật cản ngay trước) → v = 0, cảnh báo `holding position`.

**(f) Hiển thị:** `/local_plan`, `/CustomDWB/local_path`, marker quỹ đạo, `/CustomDWB/state` (5 Hz, và ngay khi đổi
pha hoặc lỗi). Log `phase X -> Y` mỗi khi đổi pha.

### 8.3 controller_server làm gì với lỗi của plugin
- Plugin lỗi → controller_server phát lệnh 0. Lỗi kéo dài > 0.3 s (~3 chu kỳ) → abort FollowPath.
- Progress checker báo lỗi → abort ngay.
- Bị hủy / abort / deactivate → phát lệnh 0.

---

## 9. DWB tự né vật cản

A\* không biết vật cản mới, nên **việc né vật cản hoàn toàn do CustomDWB làm**, dựa trên local costmap (LiDAR,
2 × 2 m quanh robot, chỉ phía trước ±90°).

### Cơ chế
1. **Loại cứng:** quỹ đạo nào (kể cả đuôi phanh) chạm vùng cách vật cản < 0.30 m thì bị loại. Robot không bao giờ
   chọn vận tốc mà không kịp phanh.
2. **Chấm điểm mềm:**
   - quỹ đạo đi gần vùng inflation (≤ 0.45 m quanh vật cản) bị phạt (`obstacle_scale` 0.5);
   - quỹ đạo lệch khỏi đường A\* cũng bị phạt (`path_distance_scale` 4.0, `carrot_distance_scale` 2.0).
3. Hai lực kéo này cân bằng nhau, cho ra hành vi bên dưới.

### Né được và không né được

| Trường hợp | Hành vi |
|---|---|
| Vật cản **cạnh đường**, còn đủ lối (tâm robot cách vật ≥ 0.30 m) | Lệch nhẹ để giữ khoảng cách, rồi quay về đường. Offline: 5/5 thành công, khoảng hở thân ≥ 0.16 m |
| Vật cản **chắn hẳn đường** | Dừng cách ~0.1 m (`holding position`). Vật cản đi khỏi trong 10 s → đi tiếp. Không → `Failed to make progress` → `ABORTED` |
| Vật cản **di chuyển cắt ngang** | Dừng hoặc giảm tốc khi nó vào vùng nguy hiểm, đi tiếp khi nó qua (chưa thử trên robot) |
| Vật cản **phía sau / bên hông ngoài ±90°** | Không thấy (nửa sau scan bị lọc). Không ảnh hưởng vì robot không lùi |

DWB **không đi vòng được** quanh vật cản chắn đường:
- nó chỉ nhìn trước khoảng 0.26 m (0.15 m/s × 1.7 s);
- nó luôn bị kéo về đường A\*.

Sau `ABORTED`: dọn vật cản rồi gửi lại 2D Goal Pose. A\* chỉ biết bản đồ tĩnh nên sẽ cho lại đường cũ.

### Cách thử né vật cản trên robot
Vật cản lớn ngay trước robot làm `localization_monitor` tụt điểm khớp scan–bản đồ (điểm trên vật cản không khớp
tường nào) → báo UNCERTAIN → bridge **hủy sau 2 s, trước khi DWB kịp xử lý**. Đã xảy ra nhiều lần trên robot (mục 11).
Vì vậy khi thử né vật cản:

```bash
ros2 launch stage3_navigation navigation.launch.py navigation_enabled:=true monitor:=false auto_follow:=true
```

`monitor:=false` tắt luôn việc tự hủy khi mất định vị thật, nên **phải có người giám sát và E-stop**.

---

## 10. Các tình huống và cách hệ thống phản ứng

| Tình huống | Diễn biến |
|---|---|
| **Bình thường, đích phía trước** | FOLLOW_PATH, v tăng 0.05 m/s mỗi chu kỳ tới 0.15. Gần đích: ROTATE_TO_GOAL → đúng hướng → dừng → `SUCCEEDED` |
| **Đích ở sau lưng** | ROTATE_TO_PATH ~180° (~6 s ở 0.5 rad/s) → FOLLOW_PATH. Xoay > 10 s → `Failed to make progress` |
| **Chế độ cách ly (C)** | Robot không chạy → sau 10 s `Failed to make progress` → `ABORTED`. Bình thường |
| **Vật cản cạnh đường** | Lệch nhẹ né rồi về đường (mục 9) |
| **Vật cản chắn hẳn đường** | Dừng, chờ 10 s → `ABORTED` nếu không thông. Không đi vòng |
| **Vật cản lớn ngay trước (gần)** | Monitor báo UNCERTAIN → bridge **hủy vì localization** sau 2 s. Thử với `monitor:=false` |
| **Robot đứng sát tường / vật cản (< 0.30 m)** | `robot pose is in collision` → lệnh 0 → sau 0.3 s `ABORTED`. Cần lái PS4 ra |
| **Khe hẹp 0.4–0.6 m** | A\* đi qua được, DWB không (nội tiếp 0.30 m) → dừng trước khe → `ABORTED` |
| **Giữ deadman PS4** | twist_mux lấy lệnh PS4. DWB vẫn tính (gốc kẹp quanh vận tốc đo). Lệch > 0.8 m → abort. Thả tay: 0.5 s sau DWB lái tiếp |
| **Gửi goal mới khi đang chạy** | Hủy FollowPath → lệnh 0 → A\* mới → phải gọi `follow` lại (trừ `auto_follow`) |
| **EKF / odom chết** | > 0.5 s: CustomDWB lỗi → lệnh 0 → sau 0.3 s `ABORTED` |
| **AMCL ngừng** | TF `map → odom` cũ quá ~2 s → lỗi. Monitor stale/LOST → bridge hủy |
| **LiDAR đứng** | Local costmap không current → controller_server **treo** (không phát gì, không nhận cancel). Robot dừng nhờ timeout twist_mux 0.5 s / watchdog driver |
| **Localization chập chờn khi quay / đi** | WARN ≤ 2 s bỏ qua; > 2 s hủy. Sau hủy, robot đứng → AMCL không cập nhật → monitor có thể kẹt UNCERTAIN → mọi `follow` bị từ chối. Gỡ: đặt lại 2D Pose Estimate hoặc nhích bằng PS4 |
| **Vượt quá đích > 0.20 m** | Tiến trình kẹt ở cuối đường → cà rốt = đích phía sau → ROTATE_TO_PATH quay lại → tiếp cận lại |
| **Bị đẩy lệch đường** | ≤ 0.8 m: tự kéo về. > 0.8 m: `ABORTED` → gửi goal mới |
| **Đường chữ U / tự cắt** | Không nhảy nhánh nhờ cửa sổ tìm 1 m + cực tiểu đầu tiên |
| **Đổi tham số khi chạy** | Kiểm tra cả lô; sai → giữ cũ; đúng → áp dụng từ chu kỳ sau |

---

## 11. Kết quả trên robot

Lấy từ log `~/.ros/log/goal_path_bridge_*.log` (không có bag, trừ `~/rot_debug`).

| Ngày | Chạy gì | Kết quả |
|---|---|---|
| 2026-10-13 | `planning.launch.py` | A\* chạy trên `maps/map.yaml` (4.04 m, 7.63 m), hiện trên RViz |
| 2026-10-14 14:58 | `navigation_enabled:=true` | Quay 135° bị lố ~50° → `Failed to make progress`. Đã sửa (`velocity_reference: command`, driver 200 ms) |
| 2026-10-14 15:26 | sau khi sửa (bag `~/rot_debug`) | Không còn quay lố; trễ lệnh → odom ~0.2 s; 2 đích tới nơi; 6/8 lần bị hủy do UNCERTAIN ngắn → thêm grace 2 s |
| 2026-10-14 15:40 | grace 2 s, `control_latency` 0.2 | 2 đích ngắn (1.37 m, 1.28 m) tới nơi. Đích dài 8.82 m: đi ~3.5 m rồi bị hủy (UNCERTAIN > 2 s) |
| 2026-10-16 18:30 | có replan | Đích 1.41 m tới nơi; đích 3.41 m bị hủy (UNCERTAIN); 2 lần abort |
| 2026-10-17 15:58 | có replan, vật cản trên đường | Xoay 180° → đi ~0.3 m → bị hủy (UNCERTAIN, consistency 0.248). Không phải DWB dừng |
| 2026-10-19 08:35–08:48 | có replan | Đích 1.26 m tới nơi; các đích 2.17 m và 5.60 m bị hủy (UNCERTAIN > 2 s); 1 lần bị từ chối lúc bắt đầu |

**Kết luận:**
- Đích ngắn (~1.3–1.4 m, có xoay tại chỗ) **thường tới nơi**.
- Nguyên nhân dừng giữa đường **chủ yếu là bridge hủy vì monitor báo UNCERTAIN**, không phải DWB.
- Hành vi né vật cản của DWB **chưa được thử trọn vẹn trên robot**, vì các lần có vật cản đều bị hủy vì localization
  trước.
- Các lần chạy 2026-10-14 lúc 15:26 và 15:40 dùng **gần như đúng code stage 3 hiện tại** (`a7c59a1`, cổng cũ). Các lần từ
  2026-10-16 tới 2026-10-19 dùng bản **có replan**. Từ khi gỡ replan, chưa chạy lại trên robot.

---

## 12. Theo dõi và debug

```bash
ros2 topic echo /goal_path_bridge/follow_state            # trạng thái bám đường
ros2 topic echo /CustomDWB/state --field message          # pha + lỗi của DWB
ros2 topic echo /CustomDWB/state                          # đủ: v/w lệnh, đo, gốc, cost, số reject
ros2 topic echo /localization/status --field message
ros2 topic echo /cmd_vel_dwb                              # chế độ cách ly
grep -E "path found|FollowPath|canceled" ~/.ros/log/goal_path_bridge_*.log   # log lần chạy cũ
```

Các trường hữu ích trong `/CustomDWB/state`:

| Trường | Ý nghĩa |
|---|---|
| `phase` | IDLE / FOLLOW_PATH / ROTATE_TO_PATH / ROTATE_TO_GOAL / BLOCKED |
| `v_cmd`, `w_cmd` | Lệnh vừa phát |
| `v_meas`, `w_meas` | Vận tốc đo |
| `v_ref`, `w_ref` | Gốc của cửa sổ |
| `dist_to_goal`, `progress`, `deviation` | Tiến độ bám đường |
| `n_valid`, `n_forward_valid` | Số ứng viên hợp lệ / hợp lệ mà có tiến |
| `rej_collision`, `rej_braking`, `rej_off_costmap` | Lý do loại |
| `compute_ms`, `over_budget`, `missed_deadline` | Tải tính toán |

Trong RViz (`planning.rviz`):
- **Bật sẵn:** `/map`, `/global_costmap/costmap`, `/scan_filtered`, `/amcl_pose`, `/global_path`,
  `/local_costmap/costmap`, `/local_plan`, `/CustomDWB/trajectories`.
- **Tắt sẵn:** `/marker` (Nav2 DWB), `/CustomDWB/local_path`.
- Màu marker: xanh lá = hợp lệ, đỏ = bị loại, xanh dương = được chọn, cam = đuôi phanh, cầu vàng = cà rốt.

Ghi bag cho lần thử né vật cản (để biết DWB dừng hay bridge hủy):

```bash
ros2 bag record -o ~/obstacle_run /cmd_vel_out /cmd_vel_nav /odometry/filtered /scan_filtered \
  /local_costmap/costmap /CustomDWB/state /goal_path_bridge/follow_state /localization/status /amcl_pose
python3 scripts/analyze_cmd_lag.py ~/obstacle_run --axis v
```

---

## 13. Test và công cụ

| Thành phần | Làm gì |
|---|---|
| `test/test_dwa_core.cpp` | gtest lõi: cửa sổ, phanh, va chạm, bám đường, đích, dao động, tham số + kịch bản kín vòng |
| `test/test_dwb_controller_plugin.cpp` | gtest plugin qua `nav2_core::Controller` (domain 87, localhost) |
| `test/dwa_closed_loop_sim.cpp` + `sim_world.*` | Benchmark kín vòng (`--robot nominal/sluggish/laggy`) |
| `test/offline/offline_sim.py`, `run_benchmark.py`, `scenarios.json` | Benchmark trong `controller_server` thật, so với Nav2 DWB |
| `scripts/analyze_cmd_lag.py` | Bag → độ trễ, gia tốc/phanh thực, quay lố |

```bash
cd ~/capstone_ws
colcon build --symlink-install --packages-select stage3_navigation
colcon test --packages-select stage3_navigation && colcon test-result --verbose
```

Kết quả sau khi gỡ replan (2026-10-19): **96 kết quả, 0 lỗi**, 10 bỏ qua (cppcheck, ament tự bỏ). Cách chạy benchmark
và test offline: `doc/offline_runbook.md`.

---

## 14. Giới hạn đã biết

- **DWB chỉ né được vật cản không chắn hẳn đường.** Vật cản chắn đường → dừng → abort sau 10 s. Không đi vòng.
- **A\* không biết vật cản mới:** global costmap chỉ có bản đồ tĩnh.
- **Vật cản chưa có trong bản đồ làm monitor tưởng mất định vị** → bridge hủy trước khi DWB xử lý. Đề xuất sửa
  (chưa làm): chỉ tính điểm khớp trên các điểm scan gần bản đồ.
- **Đường dài bị hủy giữa chừng:** điểm khớp scan–bản đồ tụt dần khi đi xa (đích 8.82 m bị hủy sau ~3.5 m).
  Nguyên nhân chưa rõ, cần bag.
- **Không có hành vi phục hồi:** không lùi, không xoay thoát khi đứng sát vật cản.
- **Global vs local khác nhau:** A\* (nội tiếp 0.20 m) đi được khe mà DWB (0.30 m) không qua được.
- **controller_server treo khi scan đứng** (giới hạn của Nav2 Humble), xem `doc/p2_proposals.md` #2.
- **Feedback `distance_to_goal`** mà bridge log không phản ánh tiến độ. Xem `progress` / `dist_to_goal` trong
  `/CustomDWB/state`.
- **Kiểm tra va chạm chỉ ở ô tâm:** đúng khi footprint tròn và tâm xoay trùng tâm thân (giả định, chưa kiểm chứng).
- **`/local_plan` có hai publisher** (hai plugin); chỉ plugin đang chạy publish.
- **Chưa đo trên robot:** giới hạn vận tốc/gia tốc, quãng phanh, trọng số. `control_latency` 0.2 s mới đo từ một bag.
- **Không có PS4 lock / E-stop phần mềm:** PS4 chỉ ghi đè tạm thời. Dừng thật: `cancel`, Ctrl+C, E-stop phần cứng.
