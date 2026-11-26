# stage4_gazebo — tiến độ

## Trạng thái

| Giai đoạn | Trạng thái |
|---|---|
| S0 điều khiển | Xong trên sim, chờ duyệt (2026-11-02) |
| S1 odometry | Xong trên sim, chờ duyệt (2026-11-02) |
| S2 LiDAR/SLAM | Xong trên sim, chờ duyệt (2026-11-02) |
| S2b world kho AWS + map | Xong trên sim, chờ duyệt (2026-11-02) |
| S3 điều hướng + camera | Xong trên sim, chờ duyệt (2026-11-02) |
| S2c/S3 world nhà AWS + map | Xong trên sim, chờ duyệt (2026-11-02) |

## S0 — Hạ tầng + điều khiển (2026-11-02)

### Môi trường
- Gazebo Fortress `ign gazebo` 6.18.0, `ros-humble-ros-gz` 0.244.26.
- Phiên Wayland (Qt tự chạy XWayland). GUI dùng driver `radeonsi` (GPU thật); `DRI_PRIME=1` chuyển sang RX 6550M. `glxinfo` chưa cài.
- `/home` còn 12 GB.

### Đã làm
- Package `stage4_gazebo` (ament_cmake):
  - `zlac_driver_model` (lib, không ROS)
  - `zlac_sim` (node)
  - URDF sim, world `empty_floor.sdf`
  - launch `sim_world` + `s0_control`
  - `rviz/s0.rviz`
  - gtest `test_zlac_driver_model` (10)
  - script `s0_check.py`
- **zlac_sim**: phần ROS chép từ `zlac8015d_move.cpp`:
  - động học, `lround` RPM, đảo dấu bánh phải
  - watchdog 100 ms / 0.5 s
  - feedback 20 Hz

  Phần firmware là `ZlacDriverModel`:
  - hàng đợi trễ
  - ramp từng bánh
  - kẹp ±3000
  - RPM 0.1, encoder 4096 tick, 0.336 m/vòng
- **Bánh**: 2 `JointController` (vận tốc) và `JointStatePublisher` (encoder đọc từ joint thật).
  - Không dùng `DiffDrive`, vì nó ramp chung cho cả xe và tự phát odom/TF.
  - Trục joint theo quy ước motor thật: bánh phải quay dương = lùi.
- **Ground truth**: `OdometryPublisher` → `/sim/ground_truth/odom` (TF của nó để ở `/sim/`, không bridge).
- PS4 thật: `joy_node` + `joy_teleop` + `twist_mux` chạy trực tiếp với YAML gốc stage0 + `use_sim_time`.
- `/joint_states` 20 Hz do `zlac_sim` phát (chỉ để vẽ bánh quay).
  - Bridge thẳng 1 kHz từ Gazebo sẽ làm `/tf` 1 kHz.

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`)

| Bài thử | Kết quả |
|---|---|
| Đứng yên 60 s | trôi 0.00 mm, yaw 0.000° |
| `/wheel_feedback` | 20.00 Hz (sim time) |
| Tiến 0.2 m/s, 3 s | dx +0.548 m, RPM +36/+36 |
| Lùi 0.2 m/s | dx −0.550 m |
| Quay ±0.4 rad/s | ±70.4°, L −12 / R +12 |
| Joy (−0.1) vs nav (+0.2) | lùi −0.275 m → joy thắng |
| Ngừng lệnh → dừng | 1.05–1.10 s (2 lần chạy) |
| Bước 0 → 0.3 m/s | 10→90 % 0.400 s, tới đích 0.500 s |
| Vận tốc ổn định | 54.0 rpm, 0.3025 m/s |
| TF cố định vs `robot.urdf` | lệch 0.000 mm / 0 rad |
| Nguồn `/tf`, `/tf_static` | chỉ `robot_state_publisher` |
| RTF | 1.0 |
| CPU | Gazebo ~41 %, zlac_sim ~12 % |

- Thời gian dừng = twist_mux timeout 0.5 + timer watchdog ≤ 0.1 + decel 0.5.
- `accel_time_ms:=200` qua launch arg được nhận đúng.

### Quyết định
- Ramp mặc định `time_to_target`: đổi lệnh nào cũng tới đích trong đúng `accel/decel_time`.
  - Lý do: comment trong driver cho thấy 500 ms → trễ ~0.7–0.8 s, 200 ms → ~0.2 s. Nếu ramp là độ dốc cố định (0→3000 rpm) thì đổi 500 → 200 ms gần như không có tác dụng.
  - Có `ramp_mode: rate` để thử cách hiểu còn lại.
- `command_delay` mặc định 0 (cấu hình lý tưởng).
- Gazebo được gọi thẳng `ign gazebo` (không qua `gz_sim.launch.py`).
  - Lý do: bản đó chạy `shell=True`, Ctrl+C bỏ lại tiến trình Gazebo.
  - Đóng cửa sổ Gazebo → tắt cả launch.

### Bằng chứng từ bag `~/rot_debug` (2026-10-14, accel 200 ms)
- `/cmd_vel_out` → RPM bắt đầu đổi sau 0.11–0.16 s.
- Dừng từ 15 rpm về 0 mất ~0.1–0.14 s, vọt lố ~1.6 rpm.
- Nhiễu RPM ổn định ±1 rpm.
- Chưa đủ để phân biệt hai cách hiểu ramp: lệnh DWB tăng từng bậc nhỏ.

### Sửa khả năng sử dụng `sim_world` (2026-11-02)
- Vấn đề người dùng gặp: chạy riêng `sim_world.launch.py` thì robot không lái được (nó không có driver/teleop); camera GUI ở xa (−6, 0, 6) nên robot Ø0.4 m trông rất nhỏ; có hộp thoại xác nhận khi thoát; một Gazebo server bị bỏ lại chạy ngầm sau khi tắt.
- Đã sửa:
  - Server (`-s`) và GUI (`-g`) là 2 tiến trình riêng: Ctrl+C hoặc đóng cửa sổ đều tắt sạch (đã thử cả hai).
  - `config/gui.config` (chép từ bản mặc định): camera gần robot, tắt quick start + hộp thoại thoát.
  - Tùy chọn camera bám robot `follow:=true` (mặc định tắt: chế độ bám giành quyền camera, không xoay quan sát bằng chuột được).
  - Chạy riêng `sim_world` sẽ in lời nhắc dùng `s0_control`.
  - Nếu còn Gazebo server cùng `IGN_PARTITION` thì launch dừng và in lệnh `kill`.
  - Bớt log: `robot_state_publisher`, `create`, bridge chuyển sang `output='log'`.
- GUI render bằng GPU (`radeonsi`, không phải llvmpipe). GUI ~100–150 % CPU, RTF 1.0.
- Sau khi sửa, `s0_check` vẫn 9/9 PASS.

### Mô hình xe đẹp hơn, bỏ laptop (2026-11-02, theo yêu cầu người dùng)
- Visual chép từ `robot.urdf`, dùng chung mesh STL của `robot_description`:
  - bánh `drive_wheel.stl`, nằm trong link bánh nên quay theo;
  - `motor_plate`, `lidar_plate`, `camera_mount_photo`, `camera_d435`, `rplidar`;
  - trụ, breadboard, IMU, caster theo kích thước thật.
- `sim_world` thêm share của `robot_description` vào `IGN_GAZEBO_RESOURCE_PATH` để Gazebo tìm `package://` mesh.
- Màu ghi trực tiếp trong từng visual (`xacro:color`). Lý do: trình chuyển URDF→SDF chỉ giữ màu tham chiếu theo tên cho visual đầu tiên của link, các visual sau thành đen. Gazebo còn nhân màu ×1.25 nên trông sáng hơn RViz.
- Bỏ laptop. Hệ quả cho S2: nửa sau LiDAR trong sim không bị che (laser_filters vẫn bỏ nửa sau).
- Khối lượng thân giữ 8.0 kg (TBD).
- Sau khi đổi: frame cố định vẫn lệch 0.000 mm; `s0_check` 9/9 PASS; RTF 1.0.
- Còn lại: mặt đế có vệt sọc mảnh do bóng đổ của đèn mặt trời (shadow acne), chỉ ảnh hưởng hình ảnh.

## S1 — Odometry (2026-11-02)

### Đã làm
- `urdf/sim_sensors.xacro`: sensor IMU Gazebo tại `imu_link`, 50 Hz, topic Gazebo `/sim/imu`.
  - Nhiễu gyro/accel là xacro arg, mặc định 0 (cấu hình lý tưởng).
  - World `empty_floor.sdf` nạp thêm system `Imu`.
- `imu_sim_odom` (node tên `imu_odom`, giống node thật):
  - `/sim/imu` → `/imu/data` (`imu_link`) + `/imu_odom` (`odom`→`base_footprint`, yaw + vyaw)
  - service `/imu_odom/zero_yaw`, tham số `zero_yaw_on_start` (mặc định false)
  - covariance, logic yaw, stamp `now()` chép từ `imu_odom.cpp`
  - Không có `/imu/mag` (không mô phỏng từ kế; stage1–3 không dùng).
- `s1_odom.launch.py`: S0 + `imu_sim_odom` + `zlac8015d_odom` + EKF.
  - `zlac8015d_odom` và EKF chạy nguyên bản, tham số giống `ekf.launch.py` / `ekf.yaml`, thêm `use_sim_time`.
- `config/s1_odom/bridge_s1.yaml` = S0 + IMU.
- `s0_control.launch.py` thêm arg `bridge_config`, `rviz_config` (để s1+ truyền xuống).
- `rviz/s1.rviz`: Fixed Frame `odom`, `/odometry/filtered` (+ `/wheel_odom`, tắt sẵn).
- `test/s1_check.py`.

### Sửa mô hình: collision bánh
- Lỗi phát hiện ở S1: quay tại chỗ thì `wheel_odom` thiếu 12 % (95° so với 108° thật).
- Nguyên nhân: collision trụ chạm sàn ở mép trong bánh.
  - Track hiệu dụng 2 × (0.17 − 0.0415/2) = 0.298 m thay vì 0.34.
- Sửa: collision bánh là **cầu** r 0.0535 (chạm ngay dưới tâm).
  - Sau khi sửa: `wheel_odom` quay sai ≤ 0.17°.
- Hệ quả cho số đo S0: quay ±0.4 rad/s 3 s trước đây ra 70.4° (xe quay nhanh hơn lệnh 14 %), nay ra 61.8° (đúng).
  - `s0_check` vẫn 9/9 PASS.

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`)

| Bài thử | Kết quả |
|---|---|
| `/odometry/filtered` | 30.00 Hz |
| `/imu_odom`, `/imu/data` | 50.00 Hz |
| `/wheel_odom` | 20.00 Hz |
| Đứng yên 10 s | EKF trôi 0.00 mm, 0.000° |
| Thẳng 2.05 m | sai EKF 1.6 mm (≤ 50) |
| Quay +94° | sai EKF 0.00° (≤ 3) |
| Quay −94° | sai EKF 0.00° |
| Nguồn `odom→base_footprint` | chỉ EKF, 30.00 Hz |
| `zero_yaw` | yaw 136° → 0.000° |
| RTF | 1.0 |
| CPU | Gazebo ~44 %, EKF ~6 %, imu_sim ~6 % |

- Bài quay dừng bằng lệnh 0 tường minh khi đạt góc, nên quay vượt ~4° do ramp decel 0.5 s (so sai số tại góc cuối).
- `zlac8015d_odom` luôn tạo `TransformBroadcaster` nên có tên trong publisher `/tf`, nhưng không gửi khi `publish_tf: false` (đã kiểm: cạnh `odom→base_footprint` đúng 30 Hz = chỉ EKF).

### Phát hiện: sai số EKF trên cung tròn (thuộc thuật toán stage1, không phải sim)
- Bài tham khảo (không thuộc tiêu chí): v 0.2 m/s, ω 0.3 rad/s, 8 s, quay 136°.
  - `wheel_odom` sai 2.0 mm; yaw EKF sai 0.00°; **vị trí EKF sai 72 mm**.
- Nguyên nhân: EKF tích phân `vx` của `/wheel_odom`. `vx` này qua lọc thông thấp α 0.2 @ 20 Hz (trễ ~0.2 s), còn yaw từ IMU thì không trễ.
  - Mỗi lần tăng/giảm tốc, quãng đường ≈ v·τ bị tính theo hướng khác: sai ≈ v·τ·2·sin(Δyaw/2) ≈ 0.04 × 1.85 ≈ 74 mm.
  - Đi thẳng không bị.
- Xe thật cũng có lỗi này (cùng code). Nó ảnh hưởng tới AMCL / CustomDWB khi chạy cung có dừng.
- Chưa sửa: muốn sửa phải đổi stage1 (vd. fuse pose x/y của wheel_odom hoặc giảm lọc), cần người dùng quyết định.

## S2 — LiDAR, SLAM, định vị (2026-11-02)

### Đã làm
- **World từ bản đồ thật** (`scripts/map_to_world.py`): `maps/map.yaml` → world SDF.
  - Ô occupied (ngưỡng `occupied_thresh` như map_server) gộp tham lam thành 4 200 hình chữ nhật (13 596 ô), đùn cao 1.0 m (`wall_height`, tạm) → **một** mesh STL nhị phân (42 000 tam giác, 2.1 MB) dùng cho cả visual + collision.
  - Toạ độ world = frame `map` → `/sim/ground_truth/odom` so thẳng với `/amcl_pose` / TF map.
  - Sinh lúc launch, cache trong `~/.ros/stage4_gazebo/worlds/<tên>_<hash>/` (0.3 s), không nằm trong repo.
  - Đã kiểm: hợp các hình chữ nhật = đúng mặt nạ occupied, không chồng.
  - World có system `Sensors` (ogre2) cho gpu_lidar; `empty_floor.sdf` không đổi.
- `urdf/sim_sensors.xacro`: `gpu_lidar` tại `lidar_link`, 720 tia, quét 0 → 2π, 10 Hz, 0.05–16 m, nhiễu xacro arg `lidar_noise` (0.01 m tạm).
- `lidar_sim_adapter` (C++): `/sim/scan` → `/scan_raw` đúng quy ước `sllidar_node` (đọc `publish_scan`):
  - angle −π…π, inc 2π/719, frame `laser_raw`, góc 0 ra SAU;
  - chỉ trừ π vào angle_min/max, giữ thứ tự tia (không nội suy);
  - ngoài [0.05, 16] / không hữu hạn → +inf; QoS reliable keep 10 như driver.
- `s2_slam.launch.py`: S1 + adapter + `scan_rotate` + `laser_filters` (nguyên bản, YAML stage2 + `use_sim_time`).
  - `mode:=localization` (mặc định): `nav2_bringup localization_launch.py` + `amcl_params.yaml`, CSM, monitor (`csm`/`monitor` mặc định true như stage3).
  - `mode:=mapping`: slam_toolbox, cùng override với `mapping.launch.py`.
  - Arg: `world_map` (map sinh world), `map` (map cho AMCL, rỗng = world_map), `world`, `wall_height`.
  - RViz dùng lại `stage2_slam/rviz/localization.rviz`; `/initialpose` → `/initialpose_guess` khi csm.
- `config/s2_slam/bridge_s2.yaml` = S1 + `/sim/scan`.
- `test/s2_check.py` (`--mode localization|mapping`, `--save`). Map sim lưu ở `stage4_gazebo/maps/sim_map.{yaml,pgm}`.

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`)

| Bài thử | Kết quả |
|---|---|
| `/scan_raw`, `/scan`, `/scan_filtered` | 10.00 Hz |
| `/scan_raw` | 720 tia, [−π, π], `laser_raw` |
| Nửa sau `/scan_filtered` | 360/360 NaN, trước 0 NaN |
| Scan vs ray-cast map | trung vị 9–11 mm |
| Giả thuyết xoay 180° / gương | 3.2 m / 3.2 m |
| Tia ngắn < 0.3 m (thân xe) | 0 |
| Hộp 0.297 m trước 0.8 m | không thấy |
| Hộp 0.497 m | thấy 0.687 m (mặt 0.70) |
| Định vị (đoán lệch 18 cm/5°) | LOCALIZED 4.5–5.1 s |
| CSM | ACCEPTED, score 0.84–0.88 |
| AMCL sau định vị | 32–34 mm, ≤ 0.3° |
| AMCL khi lái 1.55 m + 90° | ≤ 74 mm; cuối 49 mm |
| SLAM 360° + 1.55 m | 49–60 mm, 0.00° |
| Nguồn `map→odom` | chỉ slam_toolbox / AMCL |
| Định vị trên map sim đã lưu | LOCALIZED 4.5 s, 34 mm |
| Pause Gazebo 8 s | vẫn LOCALIZED |
| RTF | 1.0 |
| CPU | Gazebo ~56 %, AMCL ~6 % |

- `s2_check`: localization 8/8 (map thật và map sim), mapping 10/10. Sau khi thêm LiDAR, `s1_check` vẫn 10/10.
- Tia scan lệch > 5 cm (~10 %) nằm ở chỗ khoảng cách nhảy: mép góc, khe chéo giữa ô tường dày 1 ô, tầm xa (< 2 m: 97 % < 5 cm; 10–16 m: 69 %).
- Hộp cao đo ngắn 13 mm: lấy min của ±3° tia có nhiễu 0.01 m.

### Phát hiện
- **AMCL sai 4–12 cm** khi lái trên bản đồ khớp hoàn hảo (thử thêm 3 đoạn thẳng + 3 lần quay 60°: 38–120 mm, ≤ 1.6°).
  - `amcl_params.yaml` chỉ đặt particles + `update_min_*`, còn lại mặc định Nav2 (`sigma_hit` 0.2 m, `max_beams` 60).
  - Đây là giới hạn của cấu hình AMCL, không phải do map lệch; xe thật cũng chịu.
  - Chưa chỉnh (thuộc stage2).
- **Wall timer** của `localization_monitor` / `scan_match_initializer`: tuổi dữ liệu tính bằng `now()` (sim time). Pause 8 s → tuổi đứng yên, không LOST/UNCERTAIN giả. RTF < 1 chưa thử (cơ chế tương tự). Không cần sửa.
- **Xe kẹt tường → odometry hỏng**: một lần thử, script lái thẳng theo 1 tia ray-cast lọt qua khe nhỏ trên tường mảnh của bản đồ; xe Ø0.4 húc tường, bánh trượt.
  - Hệ quả: `wheel_odom` +2.7 m, EKF sai 2.7 m, SLAM sai 1.19 m / −8°.
  - Collision mesh giữ xe đúng; lỗi do script, đã sửa (`free_ahead` ray-cast 5 tia song song ±0.25 m).
  - Gợi ý cho xe thật: bản đồ có khe mà LiDAR nhìn xuyên được.
- `slam_toolbox` không có mặt sau (laser_filters bỏ), nên `sim_map` vẽ sau 360° + 1.55 m chỉ có 343×192 ô quanh điểm xuất phát.

### Quyết định
- Mesh STL đơn thay 4 200 hộp: 1 draw call cho GUI và gpu_lidar.
- Cache world trong `~/.ros`, không đưa mesh 2 MB vào repo.
- gpu_lidar quét 0 → 2π để adapter chỉ đổi angle_min/max, không xoay mảng.
- Stamp scan giữ của Gazebo (sim time lúc chụp; driver thật dùng lúc bắt đầu `grabScanData`).
- Không include launch stage2 (mở cổng thật); node + YAML gốc như S1.

## S2b — World kho AWS RoboMaker Small Warehouse + map chính (2026-11-02, theo yêu cầu người dùng)

### Đã làm
- Model thật từ `aws-robotics/aws-robomaker-small-warehouse-world` nhánh `ros2` (commit `ee0af73`, MIT-0, 24 MB).
  - `scripts/fetch_aws_warehouse.sh` clone vào `~/capstone_ws/third_party/aws_small_warehouse`, ngoài repo.
  - Không sửa model nào: Fortress tìm được mesh `file://models/...` khi `IGN_GAZEBO_RESOURCE_PATH` có cả `<aws>` và `<aws>/models`.
- `scripts/aws_warehouse_to_world.py` → `worlds/aws_small_warehouse.sdf` (commit file sinh ra).
  - Giữ đúng 24 model + tên + pose của `no_roof_small_warehouse.world`; mái nhà bỏ (comment trong bản AWS).
  - Sàn: visual GroundB, collision là mặt phẳng z = 0, ma sát 1.0. Mesh sàn đơn vị cm, mặt trên z = 0.124 m; AWS đặt −0.090 → mặt sàn +3.4 cm, nên hạ visual −0.125 để mặt sàn trùng collision.
  - Physics / IMU / Sensors (ogre2) giống world bản đồ; giữ đèn trần AWS + đèn hướng không bóng.
- `s2_slam.launch.py env:=warehouse` (mặc định vẫn `env:=map`, phần cũ không đổi).
  - Arg `aws_dir`; map định vị mặc định `stage4_gazebo/maps/warehouse.yaml`.
  - Chưa tải model / chưa có map → dừng và in lệnh cần chạy.
- `scripts/route_drive.py`: tự lái theo tuyến (chỉ sim).
  - Đọc pose thật để lái (như người cầm tay cầm), phát `/cmd_vel_nav`; thuật toán không thấy ground truth.
  - A* trên bản đồ tham chiếu (nới 0.40 m + phí gần vật cản), pure pursuit, quay 360° ở mỗi điểm.
  - Phát hiện kẹt; `--log-slam` in sai số SLAM so với ground truth; preset `warehouse`: 13 điểm, 81 m.
- **Map chính** `stage4_gazebo/maps/warehouse.{yaml,pgm}`: slam_toolbox (nguyên bản) vẽ trong sim.
  - Spawn (0,0,0) → frame map ≈ frame world. 288×420 ô @ 0.05 m, origin (−7.39, −10.5).
  - Khu kệ góc trên phải chưa đi vào (còn xám).
- `s2_check.py`:
  - `scan_vs_map` đổi tiêu chí chính sang tỷ lệ điểm cuối tia trúng ô occupied (±1 ô) ≥ 90 %; sai dọc tia chỉ in tham khảo. Lý do: chân kệ mảnh, tia lọt qua chỉ vì lệch vài cm.
  - `localized` không còn tính bản tin LOCALIZED cũ (trước khi monitor xử lý pose mới): phải bắt đầu sau lúc gửi pose và giữ ≥ 2 s.

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`)

| Bài thử | Kết quả |
|---|---|
| Nạp world kho | không lỗi mesh/texture |
| RTF | 1.0 |
| CPU Gazebo | ~59 % |
| Tuyến vẽ map | 81.0 m, 611 s sim, không kẹt |
| SLAM giữa tuyến (lớn nhất) | 478 mm, 1.2° (điểm 6) |
| SLAM cuối tuyến | 33 mm, −0.57° |
| EKF cuối tuyến | 201 mm |
| Scan trúng map kho | 98.6 % |
| Định vị kho | LOCALIZED 4.9 s |
| CSM / AMCL sau định vị | ACCEPTED 0.979 / 54 mm |
| AMCL khi lái | ≤ 62 mm |
| `s2_check` kho / map cũ | 8/8 / 8/8 |

### Phát hiện
- **Bản đồ AWS `maps/005` lệch world ~(0.10, 0.15) m** (scan trúng 25 % → 87 % sau khi dời). Đó là map SLAM của AWS, không phải ground truth. Chỉ dùng để vạch tuyến lái (đã nới 0.40 m).
- **SLAM drift khi đi dọc tường trái dài 17 m** (sai tăng 21 → 478 mm, yaw 1.2–1.8°), về lại vùng cũ thì loop closure kéo về 33 mm.
  - Lần chạy đầu bị dừng sớm ở góc trên trái (lỗi phát hiện kẹt, đã sửa), chưa kịp khép vòng → SLAM sai 593 mm.
  - Bài học cho xe thật: vẽ map phải đi khép vòng.
- **CSM REJECTED khi nhìn ra vùng trống** (robot ở (0.23, 1.38) quay −167°): 3 lần khớp lệch nhau 7.9 cm → *inconsistent matches*.
  - Pose đoán được chuyển nguyên cho AMCL; đứng yên thì AMCL không cập nhật → UNCERTAIN 40 s; lái đi thì hội tụ (9 mm).
  - Tái hiện đúng hạn chế "đứng yên sau cancel thì kẹt UNCERTAIN" đã ghi trong CLAUDE.md. Không sửa (stage2).
- Tường trái map kho có đoạn bị nhân đôi nhẹ (do drift trước loop closure).

### Quyết định
- Model AWS để ngoài repo + script tải ghim commit; world Fortress sinh bằng script, commit file kết quả.
- Không dùng collision mesh của sàn AWS (rủi ro bánh nảy / sai odometry).
- Map chính vẽ bằng chính stack stage2 trong sim, không dùng map AWS.

## S3 — Điều hướng + camera (2026-11-02)

### Đã làm
- `launch/s3_navigation.launch.py` (mặc định `env:=warehouse`):
  - Include `s2_slam.launch.py` với `rviz:=false` và `bridge_s3.yaml`.
  - Chạy trực tiếp các node của stage3 (**không** include `navigation.launch.py`, vì nó kéo stage2 thật vào):
    - `planner_server` (A*) + `lifecycle_manager_planning`
    - `controller_server` (CustomDWB / FollowPath) + `lifecycle_manager_navigation`
    - `goal_path_bridge`
  - `nav2_params.yaml` gốc qua `RewrittenYaml(use_sim_time: True)`. Cách này áp cho cả node con `global_costmap` / `local_costmap`.
  - RViz dùng `planning.rviz` của stage3, remap `/initialpose` → `/initialpose_guess` khi `csm`.
  - Mặc định khác xe thật:
    - `navigation_enabled:=true` (→ `/cmd_vel_nav` → twist_mux → zlac_sim)
    - `auto_follow:=true`
  - `require_localized` = localization và monitor (giống stage3).
- Camera D435:
  - xacro arg `camera` (mặc định false): sensor `depth_camera` tại `camera_link`, 424×240 @15 Hz, HFOV 87°, tầm 0.28–10 m, nhiễu 0.005 m (TBD).
  - Arg `camera` được truyền xuyên sim_world → s0 → s1 → s2 → s3.
  - s2 có thêm arg `bridge_config`.
- `cloud_sim_adapter` (C++): `/sim/camera/depth/points` → `/camera/camera/depth/color/points`.
  - Frame `camera_depth_optical_frame`, chỉ giữ x/y/z float32, unorganized, bỏ điểm không hữu hạn (giống realsense với `stream_filter 0`).
- `camera:=true` → `camera_layer:=auto` = fusion (`camera_fusion.yaml` gốc).
- `test/s3_check.py`: định vị, 3 goal, né vật, chắn kín, (`--camera`) trục point cloud, hộp thấp, né hộp thấp. Khoảng hở thân chấm theo ground truth.

### Kiểm chứng trục point cloud (yêu cầu §8 S3)
- Point cloud của `depth_camera` Fortress nằm trong frame **link** (X trước, Y trái, Z lên), **không** phải optical. Đo bằng hộp đặt cách camera 0.73 m, lệch trái 0.2–0.4 m: các điểm có x 0.713, y 0.18–0.41.
  - Chỉ đổi `frame_id` thì dữ liệu sẽ sai trục. Adapter xoay từng điểm: `x_opt = −y`, `y_opt = −z`, `z_opt = x`.
- Sau adapter, qua TF về `base_footprint`: hộp 0.2×0.2×0.3 đặt ở (1.0, 0.3) đo được x 0.883–1.098, y 0.179–0.407, z 0.03–0.309. Sàn có z trung bình −0.0004 m.

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`, kho AWS)

| Bài thử | Kết quả |
|---|---|
| `s3_check --camera` | 9/9 PASS |
| `s3_check` (LiDAR) | 6/6 PASS |
| CPU Gazebo (LiDAR) | ~56–65 % |
| Định vị | LOCALIZED 5.0 s, AMCL 30 mm |
| Goal 4.6 m (lối kệ) | SUCCEEDED, GT 0.130 m / −1.9° |
| Goal về (0, 0) | GT 0.099–0.120 m, ≤ 4.6° |
| Tuyến dài 10–12 m | SUCCEEDED, GT 0.10–0.17 m |
| Né hộp 0.3 m chắn đường | đi 5.74 / A* 5.51 m, hở 0.12 m |
| Tường chắn kín | dừng y −3.48, hở 0.09–0.18 m |
| Hộp 0.3 m (dưới LiDAR) | `/scan` 0 điểm, costmap 100 |
| Né hộp thấp, có camera | SUCCEEDED, hở 0.116 m |
| Hộp thấp, chỉ LiDAR | **đâm** (hở 0.000 m), kẹt |
| `/camera/.../points` | 14.1 Hz |
| RTF (camera bật) | 0.99–1.00 |

Sai số cuối theo ground truth = sai số goal checker (0.15 m, so với AMCL) + sai số AMCL. Ở góc trái kho, AMCL lệch khoảng 0.11 m vì tường map bị nhân đôi (S2b), nên s3_check chấm GT ≤ 0.20 m. Theo AMCL thì mọi goal đạt đều ≤ 0.15 m.

### Phát hiện
- **Kệ AWS ShelfD/E (x 2.77–6.69, 6 kệ) hở ở độ cao LiDAR 0.347 m.**
  - Map warehouse chỉ có chân kệ, nên A* coi gầm kệ là trống (vd. goal (3.6, −1) nằm trong kệ D1).
  - Collision của model kệ chỉ gồm 4 chân, nên robot chỉ có LiDAR "đi xuyên" tầng đáy kệ mà không va chạm. Ngoài đời thì đây là va chạm thật.
  - Có camera thì FusionLayer thấy tầng đáy (z 0.05–0.7 m): CustomDWB lệch > 1 m khỏi đường rồi ABORTED. Đây là hành vi đúng.
  - Lối đi thật giữa các kệ rộng khoảng 0.92 m (vd. y −0.80…0.14).
  - Bài học cho xe thật: map 2D từ LiDAR không có vật hở ở độ cao quét (kệ, bàn). Chỉ camera mới bảo vệ được, và A* vẫn lập đường xuyên qua chúng.
- **Chắn kín:**
  - Khi không có camera: dừng trước tường, trượt dọc khoảng 1 m, rồi ABORTED sau 22 s (`max_path_deviation`).
  - Khi có camera: lần chạy check bị bridge CANCELED vì UNCERTAIN > 2 s, trước lúc controller kịp abort.
  - Cả hai đều không chạm.
- **Sau khi gỡ vật,** FusionLayer giữ ô lethal thêm `decay_time` 10 s nếu camera đứng gần hơn `min_range` 0.3 m (không xóa được bằng tia). Goal gửi ngay sau đó ABORTED sau 4.8 s. Chờ 11 s thì OK. Hành vi này giống trên xe.
- `goal_path_bridge` dùng wall timer 200 ms nhưng mọi phép so thời gian đều theo `now()` (sim time), nên pause/RTF < 1 không gây hủy giả.

### Quyết định
- Không sửa stage3. Mọi khác biệt so với xe thật chỉ nằm ở tham số launch của s3.
- Goal test đặt ở lối đi thật (tính từ bbox mesh visual của kệ), không theo map 2D.
- Chưa đưa footprint kệ vào map (keepout). Hỏi người dùng trước.

## World nhà AWS RoboMaker Small House (2026-11-02, theo yêu cầu người dùng)

### Đã làm
- `scripts/fetch_aws_house.sh`: clone nhánh ros2 (ghim `ff9631c`, MIT-0, 183 MB) vào `~/capstone_ws/third_party/aws_small_house`.
  - Tạo `COLCON_IGNORE` để colcon không build package AWS (cần `gazebo_ros` classic).
  - Vá quán tính 2 model (xem Phát hiện).
- `scripts/aws_house_to_world.py` → `worlds/aws_small_house.sdf`:
  - 86 model giữ nguyên tên + pose, static.
  - Sàn FloorB: visual mesh + collision mặt phẳng.
  - 8 đèn AWS, tắt bóng, cộng đèn hướng phụ.
- `s2_slam` / `s3_navigation`:
  - `env:=house` (bảng `AWS_ENVS`).
  - `aws_dir` rỗng = thư mục mặc định của env.
  - Resource path chỉ cần `<aws>/models`.
- `route_drive.py` preset `house`: 16 điểm qua mọi phòng (điểm có khoảng hở lớn nhất trong mỗi ô 2 m của map AWS).
- `maps/house.yaml/.pgm`: vẽ bằng slam_toolbox, 2 vòng tuyến (vòng 2 `--no-spin`).

### Số đo (headless)

| Bài thử | Kết quả |
|---|---|
| Map AWS vs world | khớp 99.3 %, không lệch |
| RTF (camera bật) | 1.0 |
| SLAM vòng 1 (57.8 m) | lớn nhất 311 mm, cuối 44 mm / 2.6° |
| Map vòng 1 vs map AWS | 55 % (tốt nhất 76 %, xoay −1.25°) |
| SLAM vòng 2 | lớn nhất 107 mm, cuối 32 mm / 0.35° |
| Map vòng 2 vs map AWS | 91.9 %, phép biến đổi tốt nhất = 0 |
| `s2_check` nhà | 8/8 PASS (scan trúng 99.4 %) |
| Định vị | LOCALIZED 4.8 s, AMCL 48 mm |
| Goal S3 (camera) | 7/8 SUCCEEDED, GT 0.03–0.12 m |
| Tuyến dài nhất | 14.0 m, SUCCEEDED, 0.10 m |

### Phát hiện
- **2 model có quán tính sai làm Fortress không nạp được world** ("invalid inertia"): Dumbbell `iyy` 0, ShoeRack thiếu `izz`.
  - Script tải vá thành 0.1 / 0.04 (giữ CRLF, vá nhiều lần không sao). Cả hai đều static.
- Ảnh khung tranh tham chiếu `../../../../photos/*.jpg` không phân giải được trong Fortress. Chỉ ảnh hưởng hiển thị, chưa sửa.
- Một vòng tuyến chưa đủ: các phòng bên trái drift 0.3 m. Vòng 2 cho loop closure kéo về ≤ 0.11 m.
- **Máy tập (FitnessEquipment) gần (2.1–4.0, 2.2–3.4) gần như vô hình với LiDAR 0.347 m.**
  - A* lập đường sát nó. Camera thấy muộn, robot đã ở ô inscribed (cost 253), CustomDWB BLOCKED, không lùi được → ABORTED.
  - Cùng loại với kệ kho: vật LiDAR không thấy.

## World văn phòng Husarion (2026-11-03, theo yêu cầu người dùng)

### Đã làm
- `scripts/fetch_husarion_office.sh`: clone `husarion/husarion_gz_worlds` (ghim `87271ef`, Apache-2.0, 60 MB) vào `~/capstone_ws/third_party/husarion_gz_worlds`, tạo `COLCON_IGNORE`.
  - Tải sẵn 15 model OpenRobotics từ Gazebo Fuel vào `~/.ignition/fuel` (24 MB), để chạy được khi không có mạng.
  - Sinh map tham chiếu `<dir>/maps/office_ref.yaml` (chỉ dùng cho `route_drive.py`).
- `scripts/husarion_office_to_world.py` → `worlds/husarion_office.sdf`:
  - 99 model, giữ nguyên pose, tất cả static (kể cả vách kính trượt).
  - Bỏ tiền tố `OpenRobotics/` trong tên model; `Shelf_1` bị trùng → đổi thành `Shelf_1_dup1`.
  - Bỏ `<state>`, `<gui>`. Đèn sun không bóng. Plugin và Sensors (ogre2) giống world nhà.
- `scripts/world_to_ref_map.py` (dùng chung cho mọi world): cắt collision (box / cylinder / obj / stl / dae) ở z = 0.1–0.6 m, xuất ra map yaml + pgm theo toạ độ world.
- `s2_slam` / `s3_navigation`: `env:=office` (bảng `AWS_ENVS`, map `maps/office.yaml`), spawn mặc định (0, 0, 0) ở phòng trên trái.
- `route_drive.py`:
  - preset `office`: 21 điểm, 74 m.
  - Thêm khoá `inflate` theo từng preset; office dùng 0.30 vì ở 0.40 các cửa bị tách rời.
- `maps/office.yaml/.pgm`: vẽ bằng slam_toolbox, 2 vòng tuyến (vòng 2 `--no-spin`).

### Số đo (headless, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`)

| Bài thử | Kết quả |
|---|---|
| Nạp world (Fortress 6.18) | OK, 1 cảnh báo `.mtl` |
| SLAM vòng 1 (74.2 m) | lớn nhất 208 mm, cuối 48 mm |
| SLAM vòng 2 | lớn nhất 205 mm (phòng phải), cuối 54 mm |
| Map vs world | 92.6 % ô tường ≤ 0.10 m, trung vị 0.05 m |
| Định vị (`init_pose:=true`) | LOCALIZED 5.8 s |
| RTF S3 + camera | trung bình 0.95, thấp nhất 0.89; CPU 52 % |
| Goal S3 (camera, fusion) | 8/8 SUCCEEDED, GT 0.05–0.27 m |

### Phát hiện
- Ở độ cao LiDAR 0.347 m, ghế văn phòng và bàn chỉ hiện thành chân / trụ. Mặt bàn (~0.75 m) chỉ camera thấy (cùng loại với kệ kho, máy tập ở world nhà).
- Có 3 phòng (bếp trên phải, phòng dưới trái, phòng dưới phải) mà tuyến không vào được: cửa hẹp hoặc cửa đóng (model `Door`). Các phòng này chỉ được quét một phần qua cửa, phần còn lại là unknown, nên A* không lập đường vào đó (`allow_unknown: false`).
- Sai số GT tới 0.27 m ở khu bên phải = dung sai goal 0.15 m + map lệch ~0.2 m ở khu đó (SLAM vòng 2 lệch lớn nhất tại điểm (12.5, −5.5)).
- Bản Husarion viết cho Gazebo mới (`gz::sim::` + tên file `ignition-gazebo-*`), nhưng Fortress vẫn nạp được sau khi chuyển. Không dùng launch / env-hook của gói này.

## Thử inflation local 0.25 (2026-11-03, theo yêu cầu người dùng, đã hoàn tác)

- Yêu cầu ban đầu là 0.20, nhưng giá trị này nhỏ hơn bán kính inscribed 0.25 (robot 0.20 + padding 0.05). Khi đó ô cách vật 0.20–0.25 m có cost 0 và kiểm va chạm ô tâm của CustomDWB sẽ để thân xe chạm vật (Nav2 cũng cảnh báo). Vì vậy thử 0.25 = mức nhỏ nhất còn an toàn.
- `s3_check`, kho AWS, mỗi cấu hình 1 lần:

| Bài thử | 0.30 | 0.25 |
|---|---|---|
| Chỉ LiDAR | 6/6 | 6/6 |
| Hở thân khi né (LiDAR) | 0.12 m | 0.094 m |
| Có camera | 9/9 | 7/9 |
| Né hộp (camera) | OK | ABORTED, hở 0.036 m |
| Né hộp thấp (camera) | hở 0.116 m | hở 0.057 m |

- Nguyên nhân: không còn vùng cost mềm nên xe không bị đẩy ra xa vật. Camera xác nhận vật muộn (3 frame), nên khi ô vật xuất hiện tâm xe đã ở trong vùng inscribed (`robot pose is in collision (cell cost 253)`); không lùi được → ABORTED. Bài chắn kín chạy ngay sau đó xuất phát từ đúng chỗ kẹt nên cũng ABORTED. Không lần nào chạm vật.
- Quyết định: giữ 0.30. Lý do: inflation phải lớn hơn inscribed để còn vùng cost mềm, nhất là khi có camera.

## S-final — Benchmark RTF 10 phút (2026-11-03)

- Toàn stack S3 (`s3_navigation.launch.py`, kho AWS, localization + CSM + monitor + A* + CustomDWB), `init_pose:=true`, `joy:=false`. Khi `gui:=true` thì bật cả RViz.
- Xe tự lái tuyến `route_drive.py --preset warehouse` suốt thời gian đo (12–13/13 điểm, không kẹt), để LiDAR, AMCL, costmap và camera đều có tải.
- RTF = Δ`/clock` / Δ wall, cửa sổ 1 s, 600 mẫu. CPU = % toàn máy (12 luồng). Script: `test/real_vs_sim/rtf_bench.sh`, `rtf_meter.py`.

| Cấu hình | RTF TB | RTF min / p5 | CPU |
|---|---|---|---|
| headless, không camera | 0.999 | 0.981 / 0.996 | 27 % |
| headless, camera | 0.994 | 0.967 / 0.981 | 25 % |
| GUI + RViz, không camera | 1.000 | 0.977 / 1.000 | 23 % |
| GUI + RViz, camera | 0.992 | 0.940 / 0.976 | 46 % |

- Cả 4 cấu hình: 0 % số giây có RTF < 0.9. **Đạt tiêu chí ≥ 0.9.**
- Nặng nhất là GUI + camera: CPU gần gấp đôi, RTF thấp nhất 0.94.

## So sánh sim với xe thật (2026-11-03)

### Cách làm
- Dữ liệu thật duy nhất có sẵn: bag `~/rot_debug` (2026-10-14 15:26, CustomDWB, driver accel/decel 200 ms). Trong đó chỉ có **một lần quay tại chỗ**: ω 0.1 → 0.5 rad/s (ramp 0.4 s), giữ 0.5 rad/s ~3.9 s, rồi 0. Không có bài đi thẳng.
- Sim: `s1_odom` headless, world `empty_floor`, `accel/decel_time_ms:=200`, mọi tham số khác mặc định (**không chỉnh** theo xe thật), khối lượng 12 kg.
- Phát lại đúng 44 lệnh `/cmd_vel_out` của bag lên `/cmd_vel_nav` theo sim time. Hai bên dùng cùng một script phân tích.
- Script: `test/real_vs_sim/`.

### Kết quả

| Đại lượng | Xe thật | Sim |
|---|---|---|
| RPM khi giữ 0.5 rad/s (L / R) | −14.99 / +15.19 | −15.00 / +15.00 |
| ω EKF khi giữ | 0.524 rad/s | 0.494 rad/s |
| Tổng góc quay (IMU) | 116.7° | 115.3° |
| Tổng góc quay (EKF) | 116.7° | 115.3° |
| Góc từ encoder (track 0.34) | 114.5° | 115.2° |
| Quay thêm sau lệnh 0 | 4.8° | 3.1° |
| Lệnh → RPM ≥ 1 | 0.26 s | 0.08 s |
| Lệnh 0 → RPM < 1 | 0.26 s | 0.23 s |
| EKF trôi vị trí | 0 mm | 0 mm |

- Sai tổng góc sim so với thật: −1.45° (−1.2 %).
- Xe thật: encoder thấp hơn IMU 1.9 % → track hiệu dụng ≈ 0.334 m (hoặc sai bán kính bánh / thang IMU). Sim khớp tuyệt đối vì track 0.34 là giả định.
- Trễ lệnh của sim nhỏ hơn ~0.17 s vì `command_delay` = 0 (cấu hình lý tưởng). Đây là nguyên nhân chính khiến sim quay vượt ít hơn sau lệnh dừng.
- Một lần thử đặt `command_delay` 0.15 bằng `ros2 param set` lúc đang chạy: trễ chỉ tăng lên 0.11 s (tham số có vẻ không áp dụng đầy đủ khi đổi lúc chạy). Không dùng kết quả này.

### Kết luận
- Với cùng chuỗi lệnh, sim cho cùng RPM, cùng thời gian dừng và tổng góc lệch 1.2 %. Không chỉnh tham số nào. Như vậy sim đủ dùng để thử thuật toán stage1–3.
- Khác biệt còn lại (trễ lệnh, track hiệu dụng) đã biết nguyên nhân và nằm trong bảng TBD.
- Chưa có bài đi thẳng: cần ghi bag mới trên xe thật.

## Bảng TBD – cần đo

| Thông số | Giá trị tạm / lý do |
|---|---|
| Khối lượng xe + laptop | **12 kg (đã cân, 2026-11-03)**; sim: thân 11 + 2 bánh × 0.5 (chia là giả định) |
| Trọng tâm, quán tính thân | tâm, z +0.05 m; trụ đặc |
| Khối lượng bánh | 0.5 kg |
| Mô-men tối đa bánh | 20 N·m (joint effort) |
| Ma sát bánh / caster | 1.0 / 0.0 |
| Nghĩa `accel_time` ZLAC | `time_to_target` (xem trên) |
| Trễ lệnh, vòng tốc độ, nhiễu RPM | 0 (bag gợi ý 0.1 s, τ vài chục ms, ±1 rpm) |
| Vùng chết RPM thấp | chưa mô phỏng |
| Nhiễu / bias IMU (BNO055) | 0 (xacro arg `imu_gyro_noise`, `imu_accel_noise`) |
| Track hiệu dụng bánh cao su | 0.34 (collision cầu); xe thật chưa đo |
| Nhiễu LiDAR C1 | 0.01 m Gaussian (xacro arg `lidar_noise`), chưa đo |
| Tầm LiDAR | 0.05–16 m (driver); thực tế C1 chưa đo |
| Chiều cao tường world | 1.0 m (`wall_height`), tường thật cao hơn |
| Vật liệu / phản xạ, tia hụt | không mô phỏng (gpu_lidar luôn trả về) |
| Nhiễu độ sâu D435 | 0.005 m Gaussian (xacro arg `depth_noise`); không có lỗ dữ liệu |
| Tầm gần D435 | 0.28 m (clip near) |
| Chiều cao robot + laptop | **0.60 m (đã đo, 2026-11-03)**; FusionLayer `max_obstacle_height` 0.70 = 0.60 + lề 0.1 (giữ nguyên, chỉ sửa comment stage3) |

## Vấn đề còn lại
- Lúc đứng yên, encoder bánh phải có thể lệch 1 tick (8.2e-5 m), do lấy `floor` của góc joint xấp xỉ −0. Dưới deadband 0.0005 m của `zlac8015d_odom` nên không ảnh hưởng odom.
- Caster là cầu ma sát 0: không mô phỏng caster xoay / rung.
- Yaw IMU sim tuyệt đối theo world Gazebo (thay hướng từ trường NDOF); không mô phỏng nhiễu từ / trôi yaw.
- Chưa có bài thử trên xe thật để chỉnh ramp, trễ (S-final).
- Chưa thử với PS4 thật trong sim (chờ người dùng thử).
- S2 chưa thử với GUI Gazebo + RViz (chỉ headless): cần người dùng xem tường mesh, kho AWS (texture, ánh sáng) và RViz.
- Map kho chưa phủ khu kệ góc trên phải: thêm điểm vào preset `warehouse` hoặc lái tay rồi vẽ lại nếu S3 cần khu đó.
- Depth sim sạch hơn D435 thật (không lỗ, không bay điểm): chưa thêm trước khi kết luận về FusionLayer trên xe.
- S3 chưa thử với GUI + RViz (headless): người dùng cần thử 2D Pose Estimate → 2D Goal Pose.
- World giữ nguyên các khe trên tường mảnh của `maps/map.yaml`: LiDAR nhìn xuyên qua được nhưng xe Ø0.4 không lọt (cần lưu ý khi viết kịch bản S3).

## Thay đổi ngoài stage4
- Không có trong repo. Ngoài repo:
  - `~/capstone_ws/third_party/aws_small_warehouse` (clone AWS, MIT-0, có `COLCON_IGNORE`).
  - `~/capstone_ws/third_party/aws_small_house` (clone AWS, MIT-0, có `COLCON_IGNORE`, vá quán tính 2 model).
  - `~/capstone_ws/third_party/husarion_gz_worlds` (clone Husarion, Apache-2.0, có `COLCON_IGNORE`, map tham chiếu `maps/office_ref.*`).
  - `~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models/*` (15 model Fuel cho world văn phòng).
