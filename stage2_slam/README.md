# `stage2_slam`: LiDAR, vẽ bản đồ (SLAM) và định vị (AMCL)

Package này giúp robot **biết mình đang ở đâu trên bản đồ**. Nó có hai chế độ, chạy một trong hai:

| Chế độ | Launch | Làm gì | Ai phát TF `map → odom` |
|---|---|---|---|
| Vẽ bản đồ | `mapping.launch.py` | Lái robot đi, slam_toolbox vẽ `/map` | slam_toolbox |
| Định vị | `localization.launch.py` | Nạp bản đồ có sẵn, AMCL tìm vị trí robot | AMCL |

> **Không bao giờ chạy hai chế độ cùng lúc**: cả hai đều phát TF `map → odom`.
> ⚠️ Cả hai launch đều include stage0 và stage1, nên **bật motor, LiDAR và IMU**.

---

## 1. Các file chính

| File | Vai trò |
|---|---|
| `launch/scan.launch.py` | LiDAR + URDF + xoay + lọc scan → `/scan_filtered` |
| `launch/mapping.launch.py` | Chế độ vẽ bản đồ |
| `launch/localization.launch.py` | Chế độ định vị |
| `src/scan_rotate.cpp` | Xoay scan 180° cho đúng hướng robot |
| `config/angular_filter.yaml` | Bỏ nửa sau của scan |
| `config/amcl_params.yaml` | Tham số AMCL + map_server |
| `src/scan_match_initializer*.cpp` | CSM: chỉnh lại vị trí ban đầu trước khi gửi AMCL |
| `config/scan_match_initializer.yaml` | Tham số CSM |
| `src/localization_monitor_node.cpp`, `src/localization_health.cpp` | Monitor: đánh giá định vị tốt hay xấu |
| `config/localization_monitor.yaml` | Ngưỡng của monitor |
| `rviz/scan.rviz`, `slam.rviz`, `localization.rviz` | Cấu hình RViz cho từng launch |
| `config/slam_mapping.yaml` | **Không launch nào dùng** (bản nháp) |
| `readme.txt` | Ghi chú cũ |

Bản đồ đang dùng: `capstone_project/maps/map.yaml` + `map.pgm`.

---

## 2. Luồng dữ liệu

### Phần chung: scan (`scan.launch.py`)

```text
RPLIDAR C1 ──/dev/ttyUSB2──> sllidar_node ──/scan_raw──> scan_rotate ──/scan──> laser_filters ──/scan_filtered
                            (frame laser_raw)          (+π, laser_frame)       (bỏ nửa sau)
URDF ──> robot_state_publisher ──> TF base_footprint → base_link → laser_frame, imu_link, ...
```

### Chế độ vẽ bản đồ (`mapping.launch.py`)

```text
/scan_filtered ─────────────────────┐
TF odom → base_footprint (EKF) ─────┼──> slam_toolbox ──> /map + TF map → odom
                                    │
stage0 (PS4 + motor) + stage1 (EKF) ┘   lái robot bằng tay để vẽ
```

### Chế độ định vị (`localization.launch.py`)

```text
maps/map.yaml ──> map_server ──/map──┐
/scan_filtered ──────────────────────┼──> AMCL ──> /amcl_pose, /particle_cloud, TF map → odom
TF odom → base_footprint (EKF) ──────┤
                                     │
RViz "2D Pose Estimate" ─────────────┘  (vị trí ban đầu)

Tùy chọn csm:=true:
RViz 2D Pose Estimate ──/initialpose_guess──> scan_match_initializer ──/initialpose──> AMCL

Tùy chọn monitor:=true:
/map, /scan_filtered, /odometry/filtered, /amcl_pose, TF ──> localization_monitor ──> /localization/status
```

---

## 3. `launch/scan.launch.py`

**Vai trò:** chuẩn bị scan sạch cho mọi node phía sau. Chỉ mở LiDAR, **không bật motor**.

```bash
ros2 launch stage2_slam scan.launch.py
```

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | `/dev/ttyUSB2` | Cổng LiDAR |
| `rviz` | `true` | Mở RViz |
| `rviz_config` | `rviz/scan.rviz` | File RViz (mapping/localization tự đổi) |

Node chạy:
- `robot_state_publisher` (URDF, `publish_default_joint_states: false`);
- `sllidar_node` (`frame_id: laser_raw`, topic → `/scan_raw`);
- `scan_rotate` (`angle_offset: π`, `frame_id: laser_frame`);
- `scan_to_scan_filter_chain` (`angular_filter.yaml`);
- RViz.

### `scan_rotate.cpp`
- Driver đặt góc 0 ở **phía sau** robot. Node này cộng π vào `angle_min` và `angle_max`.
- Mảng `ranges` giữ nguyên, nên không thêm sai số.
- Kết quả: `/scan` có góc trong [0, 2π], 0 = phía trước.

### `angular_filter.yaml`

| Tham số | Giá trị |
|---|---|
| Bộ lọc | `LaserScanAngularBoundsFilterInPlace` |
| `lower_angle` | 1.5716 rad (≈ 90°) |
| `upper_angle` | 4.7116 rad (≈ 270°) |
| `replace_with_nan` | `true` |

- Tia trong khoảng 90°–270° (nửa sau, bị laptop che) thành NaN.
- Còn lại phía trước ±90°.

Chi tiết LiDAR: `hardware/sllidar_ros2/README_CAPSTONE.md`.

---

## 4. `launch/mapping.launch.py`: vẽ bản đồ

**Vai trò:** chạy scan + stage0 (PS4, motor) + stage1 (EKF) + slam_toolbox. Lái robot bằng PS4 đi khắp khu vực để vẽ.

```bash
ros2 launch stage2_slam mapping.launch.py
```

| Đối số | Mặc định |
|---|---|
| `serial_port` | `/dev/ttyUSB2` |
| `imu_port` | `/dev/ttyUSB1` |
| `rviz` | `true` (`slam.rviz`) |

### Tham số slam_toolbox

Gốc lấy từ file mặc định của slam_toolbox (`mapper_params_online_async.yaml`), launch ghi đè các giá trị sau:

| Tham số | Giá trị | Lý do |
|---|---|---|
| `scan_topic` | `/scan_filtered` | Không dùng nửa sau |
| `base_frame` | `base_footprint` | Khớp EKF |
| `mode` | `mapping` | – |
| `min_laser_range` | 0.10 m | Bỏ điểm quá gần |
| `max_laser_range` | 16.0 m | Tầm của C1 (driver báo) |
| `map_update_interval` | 1.0 s | Mặc định 5 s, quá chậm |
| `minimum_travel_distance` | 0.2 m | Mặc định 0.5 m, quá thưa với robot đi chậm |
| `minimum_travel_heading` | 0.2 rad | Mặc định 0.5 rad |

Lưu bản đồ khi vẽ xong (một cách thường dùng, chạy khi `mapping.launch.py` còn đang chạy):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/capstone_ws/capstone_project/maps/map
```

Lệnh này **ghi đè** `maps/map.yaml` và `map.pgm` hiện có. Muốn giữ bản cũ thì đổi tên `-f`.

---

## 5. `launch/localization.launch.py`: định vị

**Vai trò:** chạy scan + stage0 + stage1 + map_server + AMCL (qua `nav2_bringup localization_launch.py`), có thể thêm
CSM và monitor.

```bash
ros2 launch stage2_slam localization.launch.py                       # chỉ AMCL
ros2 launch stage2_slam localization.launch.py csm:=true monitor:=true
```

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | `/dev/ttyUSB2` | Cổng LiDAR |
| `imu_port` | `/dev/ttyUSB1` | Cổng IMU |
| `map` | `~/capstone_ws/capstone_project/maps/map.yaml` | Bản đồ |
| `rviz` | `true` | RViz `localization.rviz` |
| `csm` | `false` | Bật CSM (mục 6) |
| `monitor` | `false` | Bật monitor (mục 7) |

- Stage 3 (`planning.launch.py`, `navigation.launch.py`) gọi launch này với `csm` và `monitor` **mặc định `true`**.
- Cách dùng: chạy launch → trong RViz bấm **2D Pose Estimate**, click vào vị trí thật của robot và kéo theo hướng
  robot → AMCL bắt đầu phát TF `map → odom`.

### Bản đồ `maps/map.yaml`

| Tham số | Giá trị |
|---|---|
| Độ phân giải | 0.05 m / ô |
| Kích thước | 1249 × 514 ô (≈ 62.5 × 25.7 m) |
| `origin` | (−13.3, −7.0, 0) |
| `mode` | `trinary` |
| `occupied_thresh` / `free_thresh` | 0.65 / 0.25 |

### `config/amcl_params.yaml`

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `robot_model_type` | `DifferentialMotionModel` | Robot vi sai |
| `laser_model_type` | `likelihood_field` | So scan với bản đồ theo khoảng cách tới tường |
| `scan_topic` | `/scan_filtered` | Chỉ nửa trước |
| `min_particles` / `max_particles` | 500 / 2000 | Số hạt |
| `update_min_d` | 0.05 m | Đi được bấy nhiêu mới cập nhật |
| `update_min_a` | 0.05 rad | Quay được bấy nhiêu mới cập nhật |
| `transform_tolerance` | 1.0 s | TF `map → odom` được đóng dấu trước 1 s |
| `base_frame_id` / `odom_frame_id` / `global_frame_id` | `base_footprint` / `odom` / `map` | – |

- Các tham số khác (nhiễu chuyển động, tầm laser…) dùng mặc định của Nav2.
- **Robot đứng yên thì AMCL không cập nhật** (do `update_min_d/a`).
- Lifecycle: `map_server` + `amcl` do `lifecycle_manager_localization` quản lý (`autostart: true`).

---

## 6. CSM: `scan_match_initializer` (chỉ khi `csm:=true`)

**Vai trò:** click 2D Pose Estimate bằng tay thường lệch vài chục cm và vài độ. CSM thử nhiều vị trí quanh điểm
click, chọn vị trí mà scan khớp bản đồ nhất, rồi mới gửi cho AMCL.

```text
RViz ──/initialpose_guess──> CSM ──/initialpose──> AMCL
                              ├──/scan_match/candidate (pose tìm được)
                              └──/scan_match/status    (DiagnosticStatus)
```

Cách làm:
1. Chờ đủ **3 scan** liên tiếp khi robot đứng yên.
2. Thử mọi vị trí trong **±0.5 m** (bước 0.025 m) và mọi góc trong **±20°** quanh điểm click.
3. Chấm điểm từng vị trí bằng mức khớp scan với bản đồ.
4. Kiểm tra kết quả và gửi đi.

| Kết quả | Khi nào | Gửi gì cho AMCL |
|---|---|---|
| `ACCEPTED` | Điểm ≥ 0.65, không có đỉnh cạnh tranh | Pose CSM |
| `AMBIGUOUS` | Có vị trí khác điểm gần bằng (≥ 90%) | Pose CSM, covariance nới rộng bao cả hai |
| Lỗi khác (`REJECTED`, `NO_SCAN`, `TIMEOUT`…) | – | Nguyên pose RViz |

- CSM **không bao giờ chặn** việc khởi tạo AMCL: lỗi thì gửi nguyên pose người dùng click.
- Mỗi lần click chỉ gửi tối đa một `/initialpose`, và chỉ khi AMCL đang `active`.
- **Không phát TF.**

### Tham số chính (`config/scan_match_initializer.yaml`)

| Nhóm | Tham số | Giá trị |
|---|---|---|
| search | `linear_window` | ±0.5 m |
| search | `angular_window` | ±0.349 rad (20°) |
| search | `linear_step` | 0.025 m |
| search | `time_budget` | 3.0 s |
| validation | `min_score` | 0.65 |
| validation | `ambiguity_ratio` | 0.90 |
| validation | `min_valid_points` | 60 |
| score_field | `sigma` | 0.05 m |
| preprocess | `min_range` / `max_range` | 0.10 / 12.0 m |
| session | `num_scans` | 3 |
| session | `timeout` | 5.0 s |
| session | `max_motion_linear` | 0.03 m (robot phải đứng yên) |
| fallback | `forward_guess` | `true` |
| fallback | `publish_ambiguous` | `true` |

Ngưỡng được chọn trên dữ liệu tổng hợp, **chưa hiệu chỉnh trên robot**. Lần chạy robot 2026-10-13: kết quả
`AMBIGUOUS` (điểm 0.938, chỉnh 0.42 m).

---

## 7. Monitor: `localization_monitor` (chỉ khi `monitor:=true`)

**Vai trò:** liên tục đánh giá "định vị có đáng tin không" và công bố lên `/localization/status` (2 Hz). Monitor
**chỉ báo**, không điều khiển robot. Stage 3 (`goal_path_bridge`) dùng trạng thái này để từ chối hoặc hủy điều hướng.

| Trạng thái | Mức | Nghĩa |
|---|---|---|
| `INITIALIZING` | WARN | Chưa có bản đồ / AMCL chưa active / chưa đặt pose / vừa đặt pose < 3 s |
| `LOCALIZED` | OK | Mọi kiểm tra đạt |
| `UNCERTAIN` | WARN | Ít nhất một kiểm tra ở mức cảnh báo |
| `LOST` | ERROR | Ít nhất một kiểm tra ở mức mất |

Các kiểm tra:

| Kiểm tra | Cảnh báo khi | Mất khi |
|---|---|---|
| Tuổi scan | > 0.5 s | > 2.0 s |
| Tuổi odometry | > 0.3 s | > 1.0 s |
| Tuổi TF `map → base` | > 0.5 s | > 2.0 s |
| Độ lệch chuẩn xy của AMCL | > 0.25 m | > 1.0 m |
| Độ lệch chuẩn yaw của AMCL | > 0.17 rad | > 0.5 rad |
| Độ khớp scan–bản đồ | < 0.45 | < 0.30 trong 5 lần liên tiếp |
| Nhảy pose (AMCL khác odom) | > 0.30 m hoặc > 0.26 rad | – |

### Độ khớp scan–bản đồ (`consistency`)
- Lấy tối đa 400 điểm scan (≤ 12 m), đặt lên bản đồ theo pose hiện tại.
- Mỗi điểm cho điểm `exp(−d² / 2σ²)` (σ = 0.05 m), với d là khoảng cách tới tường gần nhất. Lấy trung bình.
- Giá trị 1 = khớp hoàn hảo, 0 = không khớp.
- Kết quả thực tế trên robot: khoảng 0.45–0.55 khi định vị tốt.

> ⚠️ **Vật cản không có trong bản đồ cũng làm điểm này tụt**: điểm scan trên vật cản không gần tường nào, nên được 0
> điểm. Một vật lớn đứng gần robot có thể làm monitor báo UNCERTAIN/LOST dù AMCL vẫn đúng (robot 2026-10-17). Khi thử
> né vật cản ở stage 3, dùng `monitor:=false` và tự giám sát.

Ngưỡng là giá trị khởi đầu, chỉ `consistency_warn` đã chỉnh theo robot (0.55 → 0.45 ngày 2026-10-14).

---

## 8. Kiểm tra nhanh (chỉ đọc, khi launch đang chạy)

```bash
ros2 topic hz /scan_filtered                          # ≈ 10 Hz
ros2 run tf2_ros tf2_echo map odom                    # có sau khi đặt 2D Pose Estimate
ros2 topic echo /scan_match/status --once             # khi csm:=true
ros2 topic echo /localization/status --field message  # khi monitor:=true
```

Trong RViz:
- Điểm `/scan_filtered` phải nằm trùng tường của `/map`.
- `/particle_cloud` có thể không hiện do khác QoS (lỗi đã biết, chưa sửa).

## 9. Lưu ý / giới hạn

- Vị trí LiDAR trên robot lấy từ CAD, chưa hiệu chuẩn.
- Robot không thấy phía sau (nửa sau bị lọc).
- Sau khi bị hủy điều hướng vì định vị kém, robot đứng yên, AMCL không cập nhật, monitor có thể **kẹt ở UNCERTAIN**.
  Cách gỡ: đặt lại 2D Pose Estimate, hoặc nhích robot bằng PS4.
- `package.xml` chưa khai báo các package mà launch include (`slam_toolbox`, `nav2_bringup`, `laser_filters`, …).
- Repo không còn test cho CSM và monitor: sửa code hai phần này thì không có gì để kiểm tra lại.
