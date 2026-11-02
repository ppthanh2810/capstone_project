# CLAUDE.md — capstone_project (bộ nhớ dự án cho Claude Code)

> File này đặt ở gốc repo (`~/capstone_ws/capstone_project/CLAUDE.md`). Claude Code đọc nó ở đầu mỗi phiên.
> Tiến độ chi tiết ghi ở `stage4_gazebo/docs/PROGRESS.md` (tạo ở S0), không ghi vào file này.

## 1. Cách làm việc với tôi

- Trả lời bằng **tiếng Việt**, thuật ngữ kỹ thuật giữ tiếng Anh. Comment trong code: tiếng Việt, giống phong cách repo.
- Làm **từng bước nhỏ, có kiểm chứng**: đọc code liên quan → nêu kế hoạch ngắn → sửa → build → chạy lệnh kiểm tra → báo kết quả bằng số liệu (Hz, sai số, RTF), không chỉ nói "đã xong".
- **Không bịa thông số.** Giá trị chưa đo (khối lượng, quán tính, ma sát, nhiễu, trễ) ghi `TBD – cần đo` kèm giá trị tạm và lý do; ghi vào bảng TBD trong `PROGRESS.md`.
- **Hỏi trước khi**: chạy `sudo`/`apt install`, sửa file trong `stage0_*`–`stage3_*` / `hardware/`, xóa file, đổi tên topic/frame, commit/push.
- **Không bao giờ** chạy launch của robot thật (`stage0`–`stage3`, `hardware/`) trên laptop này: chúng mở cổng USB và bật motor.
- Khi hết một giai đoạn: cập nhật `PROGRESS.md` (việc xong, số liệu đo, quyết định, vấn đề còn lại) và đề xuất commit message, chờ tôi duyệt.

## 2. Dự án

Robot tự hành vi sai ACE492 (ROS 2 **Humble**, C++/Python, colcon). Đọc `README.md` gốc và README từng stage khi cần chi tiết.

| Stage (robot thật) | Package | Nội dung |
|---|---|---|
| 0 | `stage0_ps4_control` | PS4/joy_teleop → twist_mux → `zlac8015d_move` (Modbus ZLAC8015D) |
| 1 | `stage1_odom` | `zlac8015d_odom` (/wheel_odom) + `imu_odom` (UART ESP32/BNO055) + EKF |
| 2 | `stage2_slam` | RPLIDAR C1 → scan_rotate → laser_filters; slam_toolbox / map_server+AMCL + CSM + localization_monitor |
| 3 | `stage3_navigation` | A* (NavFn) + goal_path_bridge + controller_server (**CustomDWB**) + local costmap (LiDAR + FusionLayer camera) |
| **4 (mới)** | **`stage4_gazebo`** | **Mô phỏng Gazebo thay lớp phần cứng, chạy lại thuật toán stage0–3** |

Mục tiêu xa: dùng mô phỏng để huấn luyện SAC chỉnh trọng số CustomDWB rồi chạy trên xe thật. **Phạm vi hiện tại chưa làm RL**, nhưng thiết kế không được chặn việc đó (xem §9).

## 3. Máy phát triển (đã đo)

- MSI Bravo 15 B7ED · Ubuntu 22.04.5 · ROS 2 Humble · Ryzen 5 7535HS (12 luồng) · RAM ~16 GB (rảnh ~8.5 GB).
- **Hai GPU AMD** (iGPU + RX 6550M), không NVIDIA/CUDA. Phải kiểm tra renderer thật bằng `glxinfo -B` và `DRI_PRIME=1 glxinfo -B`; nếu ra `llvmpipe` thì render bằng CPU → sensor GPU rất chậm.
- **/home chỉ còn ~12 GB**: rosbag ngắn (≤ 60 s, chỉ topic cần), dọn `log/` và `~/.ros/log` định kỳ, không lưu bag/mesh lớn trong repo.
- **Không** cài PyTorch/CUDA/ROCm trong phạm vi này.
- Gazebo: **Fortress + `ros-humble-ros-gz`** (cặp mặc định của Humble; CLI là `ign gazebo`, plugin tên `ignition-gazebo-*`, message `ignition.msgs.*`). Kiểm tra trạng thái hỗ trợ bằng `apt-cache policy ros-humble-ros-gz`. Phương án dự phòng nếu Fortress gây vướng: Harmonic qua `ros-humble-ros-gzharmonic` — chỉ đổi khi tôi đồng ý.

## 4. Nguyên tắc bất biến của stage4_gazebo

1. **Một package duy nhất `stage4_gazebo`** (ament_cmake, node C++ + launch Python). Bên trong chia theo giai đoạn `s0_control`, `s1_odom`, `s2_slam`, `s3_navigation` (§7). Không tạo thêm package.
2. **Chỉ thay ranh giới phần cứng, giữ nguyên thuật toán.** Node thuật toán của stage0–3 được chạy lại trực tiếp (`Node(package="stage1_odom", executable="zlac8015d_odom", ...)`) với YAML gốc của chúng + override sim. **Không include launch của stage0–3** vì các launch đó kéo theo driver phần cứng.
3. **Giữ nguyên hợp đồng giao diện** ở §5: tên topic, kiểu message, frame_id, đơn vị, dấu, tần số. Code stage1–3 phải chạy mà không biết mình đang ở sim.
4. **Thời gian**: Gazebo phát `/clock`; mọi node trong sim chạy `use_sim_time:=true`. YAML gốc hard-code `use_sim_time: false` ở ~10 file → ghi đè lúc launch bằng `nav2_common.launch.RewrittenYaml(param_rewrites={"use_sim_time": "True"})` hoặc dict đặt **sau** file YAML, kể cả node con `local_costmap/local_costmap`, `global_costmap/global_costmap` và lifecycle manager.
5. **Nguồn TF duy nhất**: robot_state_publisher (URDF tĩnh + joint bánh), EKF phát `odom → base_footprint`, slam_toolbox **hoặc** AMCL phát `map → odom`. Plugin Gazebo **không** được bridge TF/odom sang ROS.
6. **Ground truth chỉ để đánh giá**: pose thật của Gazebo phát ở `/sim/ground_truth/odom` (namespace `/sim/`), không bao giờ đưa vào EKF, costmap hay controller.
7. **Không cộng trùng hiệu ứng**: ramp/watchdog chỉ nằm ở `zlac_sim`; giới hạn gia tốc của plugin Gazebo đặt rộng hơn để không chồng ramp. Encoder lấy từ **chuyển động thật của joint bánh**, không chép lại lệnh.
8. **Sửa stage0–3 chỉ khi bắt buộc**, tối thiểu, mặc định giữ hành vi robot thật (ví dụ thêm tham số `use_sim_time`, đổi `create_wall_timer` → `create_timer` theo clock của node). Mỗi lần sửa ghi vào `PROGRESS.md` mục "Thay đổi ngoài stage4".

## 5. Hợp đồng giao diện phải giữ (trích từ repo)

| Thành phần thật | Thay trong sim bởi | Topic / kiểu | Frame | Tần số |
|---|---|---|---|---|
| `zlac8015d_move` (Modbus) | `zlac_sim` (s0) | nhận `/cmd_vel_out` (Twist); phát `/wheel_feedback` Float64MultiArray `[left_rpm, right_rpm, left_travelled, right_travelled]`, đã chuẩn hóa dấu (hai bánh tiến = dương), đơn vị giống driver thật (đọc `zlac8015d_odom.cpp` để xác nhận) | — | 20 Hz |
| PS4 + joy_teleop | giữ nguyên; nếu không có tay cầm: `teleop_twist_keyboard` remap → `/cmd_vel_joy` | `/cmd_vel_joy`, `/cmd_vel_nav` → twist_mux → `/cmd_vel_out` | — | timeout 0.5 s, prio 100 > 10 |
| `imu_odom` (UART) | `imu_sim_odom` (s1) | `/imu/data` (Imu), `/imu_odom` (Odometry: yaw + vyaw, yaw tương đối lúc khởi động), service `/imu_odom/zero_yaw` | `imu_link` | ~50 Hz |
| `zlac8015d_odom` | **giữ nguyên** | `/wheel_feedback` → `/wheel_odom` | | 20 Hz |
| EKF robot_localization | **giữ nguyên** `stage1_odom/config/ekf.yaml` | `/odometry/filtered`, TF `odom → base_footprint` | | 30 Hz |
| RPLIDAR C1 + sllidar_ros2 | sensor gpu_lidar + `lidar_sim_adapter` (s2) | `/scan_raw` LaserScan, **góc 0 hướng RA SAU robot**, frame `laser_raw` → `scan_rotate` (giữ nguyên) → `/scan` → laser_filters (giữ nguyên, ±90° trước) → `/scan_filtered` | `laser_raw` → `laser_frame` | ~10 Hz |
| RealSense D435 | sensor depth/rgbd + `cloud_sim_adapter` nếu cần (s3) | `/camera/camera/depth/color/points` PointCloud2 | `camera_depth_optical_frame` (Z trước, X phải, Y xuống) | 15 Hz, 424×240 |
| — | robot_state_publisher (xacro sim) | `/robot_description`, `/joint_states` (từ Gazebo) | cây TF giống hệt `robot.urdf` | |

Mặc định stage3 phát lệnh ra `/cmd_vel_dwb` (cách ly motor). Trong sim, cho phép `navigation_enabled:=true` → `/cmd_vel_nav`.

## 6. Thông số robot

Từ `hardware/robot_description/urdf/robot.urdf` (đơn vị m, rad, REP-103):
- Thân tròn Ø0.40; `base_footprint → base_link` z = 0.1135.
- Bánh chủ động r = 0.0535, rộng 0.0415, track 0.34, tâm trục (0, ±0.17, −0.06) theo base_link. Caster Ø0.075 tại x = ±0.068, z = −0.076.
- `imu_link` (0.11, 0, 0.1208); `camera_link` (0.17, 0, 0.1625) + 2 optical frame rpy (−π/2, 0, −π/2); `lidar_link` (0.15, 0, 0.23365) → `laser_frame` identity (LiDAR cao ~0.347 m so với sàn).
- URDF thật **không có** `<inertial>`, joint quay bánh, thẻ `<gazebo>` → stage4 tạo `urdf/robot_sim.urdf.xacro` riêng; **không sửa** `robot.urdf`.

Động lực học / giới hạn (từ code & config):
- Driver ZLAC: ramp `accel_time_ms`/`decel_time_ms` mặc định 500 ms trong code. Comment trong `nav2_params.yaml` nhắc cả 1000 ms và 200 ms → **mâu thuẫn, TBD**; dùng 500 ms làm tham số của `zlac_sim`. Watchdog 0.5 s → lệnh RPM 0.
- Trễ lệnh → `/odometry/filtered` ~0.2 s (đo 2026-10-14). CustomDWB: v ≤ 0.3 m/s, ω ≤ 1.0 rad/s, không lùi.
- PS4: tối đa 0.5 m/s, 0.6 rad/s.
- TBD – cần đo: khối lượng (kể cả laptop), trọng tâm, quán tính, ma sát bánh/caster, nhiễu IMU/LiDAR/depth, vùng chết RPM thấp, nghĩa chính xác của accel_time của ZLAC.

## 7. Cấu trúc package

```text
stage4_gazebo/
├── CMakeLists.txt, package.xml
├── docs/PROGRESS.md                 # tiến độ, số đo, bảng TBD, thay đổi ngoài stage4
├── urdf/
│   ├── robot_sim.urdf.xacro         # cùng frame/pose với robot.urdf + bánh có joint + inertial + caster
│   ├── sim_sensors.xacro            # lidar, imu, depth camera
│   └── sim_gazebo.xacro             # plugin: diff-drive/joint controller, joint_state, pose publisher
├── worlds/                          # empty_floor.sdf, room_test.sdf, obstacles_low.sdf ...
├── config/
│   ├── s0_control/  zlac_sim.yaml, bridge_s0.yaml
│   ├── s1_odom/     imu_sim.yaml, bridge_s1.yaml
│   ├── s2_slam/     bridge_s2.yaml
│   └── s3_navigation/ bridge_s3.yaml, nav_sim_overrides.yaml
├── src/
│   ├── s0_control/zlac_sim_node.cpp
│   ├── s1_odom/imu_sim_odom_node.cpp
│   ├── s2_slam/lidar_sim_adapter_node.cpp
│   ├── s3_navigation/cloud_sim_adapter_node.cpp   # chỉ khi cần sửa trục/frame
│   └── tools/ (ground_truth_eval, step_response_logger, scenario_spawner ...)
├── launch/
│   ├── sim_world.launch.py          # ign gazebo (gui/headless), spawn robot, /clock bridge, rsp
│   ├── s0_control.launch.py         # sim_world + zlac_sim + teleop + twist_mux
│   ├── s1_odom.launch.py            # s0 + imu_sim_odom + zlac8015d_odom + EKF
│   ├── s2_slam.launch.py            # s1 + lidar + scan_rotate + laser_filters + mode:=mapping|localization
│   └── s3_navigation.launch.py      # s2 + planner/controller/goal_path_bridge (+ camera:=true)
├── rviz/  s0..s3 .rviz
└── test/  kịch bản kiểm thử, script đo
```

Launch xếp chồng: s3 include s2 include s1 include s0. Tham số chung: `gui:=true|false`, `world:=...`, `camera:=false`, `rviz:=true`, `x/y/yaw` vị trí spawn.

## 8. Các giai đoạn công việc

Làm **theo thứ tự**, không sang giai đoạn sau khi tiêu chí giai đoạn trước chưa đạt. Mỗi lỗi phải truy được về đúng một tầng.

### S0 — Hạ tầng + điều khiển (tương ứng stage0)
- Kiểm tra môi trường: `ign gazebo --versions`, `glxinfo -B` / `DRI_PRIME=1 glxinfo -B`, `df -h`. Chạy được `ign gazebo -v 4 shapes.sdf`.
- Tạo package, `docs/PROGRESS.md`, xacro sim (thân, 2 bánh `continuous`, caster đơn giản hóa ma sát thấp, khối laptop phía sau che LiDAR, inertial tạm TBD), world sàn phẳng.
- Kiểm tra cây TF sim trùng `robot.urdf` (`tf2_echo` các cặp frame cố định, sai lệch ≤ 1 mm / 0.001 rad).
- `zlac_sim`: `/cmd_vel_out` → động học vi sai (r, b) → ramp → lệnh bánh trong Gazebo; watchdog 0.5 s; đọc joint bánh → `/wheel_feedback` 20 Hz theo sim time.
- Tiêu chí: robot đứng yên ổn định 60 s; tiến/lùi/quay đúng dấu; teleop chạy; PS4 (prio 100) thắng nav (prio 10); ngừng lệnh → bánh dừng sau watchdog + ramp; bước lệnh 0 → 0.3 m/s cho đường cong RPM có ramp đúng tham số; `/wheel_feedback` ≈ 20 Hz.

### S1 — Odometry (tương ứng stage1)
- Sensor IMU tại `imu_link` + bridge → `imu_sim_odom` tái tạo **đúng** đầu ra của `stage1_odom/src/imu_odom.cpp` (đọc file đó: frame, covariance, yaw tương đối, service zero_yaw).
- Chạy `zlac8015d_odom` và EKF gốc với `use_sim_time`.
- Tiêu chí (cấu hình lý tưởng, so với ground truth): đi thẳng 2 m sai ≤ 5 cm; quay 90° sai ≤ 3°; EKF 30 Hz; chỉ EKF phát `odom → base_footprint`.

### S2 — LiDAR, SLAM, định vị (tương ứng stage2)
- gpu_lidar tại `lidar_link`: 360°, ~10 Hz, tầm theo C1 (TBD xác nhận datasheet), nhiễu Gaussian tạm. `lidar_sim_adapter` chuyển sang quy ước `/scan_raw` (góc 0 ra sau, frame `laser_raw`) để `scan_rotate` + laser_filters chạy nguyên bản.
- Mapping bằng slam_toolbox, lưu map vào `stage4_gazebo/maps/` (không ghi đè `maps/map.yaml` thật); localization bằng map_server + AMCL + CSM + localization_monitor.
- Kiểm tra wall timer của `localization_monitor`, `scan_match_initializer`, `goal_path_bridge` khi RTF < 1 hoặc pause: nếu sinh LOST/UNCERTAIN giả, báo tôi trước khi sửa.
- Tiêu chí: hộp thấp hơn 0.347 m không có trong scan; nửa sau `/scan_filtered` là NaN; vẽ và lưu được map; khởi động lại định vị được trên map đã lưu, `/localization/status` = LOCALIZED.

### S3 — Điều hướng + camera (tương ứng stage3)
- Chạy planner_server (A*), goal_path_bridge, controller_server (CustomDWB) bằng `nav2_params.yaml` gốc + override sim; `navigation_enabled:=true`.
- Depth camera tại `camera_link`; xác minh point cloud nằm **thật sự** trong `camera_depth_optical_frame` (đặt vật ở vị trí biết trước, kiểm tra tọa độ trong RViz — đổi tên frame_id không sửa được dữ liệu sai trục). Bật FusionLayer với `camera_layer:=fusion`.
- Tiêu chí: tới goal trong 0.15 m / 0.25 rad; né vật nhỏ trên đường; vật chắn kín → dừng/abort (đúng hành vi hiện tại, A* không replan); vật thấp chỉ camera thấy xuất hiện trong local costmap; 0 va chạm trong bộ kịch bản nghiệm thu.

### S-final — Nghiệm thu & hiệu chỉnh với xe thật
- Benchmark RTF: mục tiêu trung bình ≥ 0.9 trong 10 phút ở world chuẩn (đo cả `gui:=false`, `camera:=false/true`).
- Cùng bài thử trên xe thật và sim (bước vận tốc, ngừng lệnh, chạy thẳng, quay 90°/360°, vật ở khoảng cách biết trước, cùng tuyến đường) → chỉnh ramp, ma sát, nhiễu; ghi kết quả vào `PROGRESS.md`.
- Toàn stack chạy được khi **không cắm** motor, LiDAR, ESP32, RealSense.

## 9. Để sẵn cho RL sau này (không triển khai bây giờ)

Không chặn các khả năng: chạy headless; seed cố định cho world/vật cản; service reset pose robot; `/sim/ground_truth/odom`; thay trọng số CustomDWB lúc chạy bằng `ros2 param set /controller_server CustomDWB.<tên>`; log metric theo episode. Observation của RL sau này chỉ được dùng tín hiệu có trên xe thật.

## 10. Lệnh thường dùng

```bash
source /opt/ros/humble/setup.bash
cd ~/capstone_ws
colcon build --symlink-install --packages-select stage4_gazebo
source install/setup.bash

ros2 launch stage4_gazebo s0_control.launch.py            # gui:=false để chạy headless
ros2 launch stage4_gazebo s2_slam.launch.py mode:=mapping
ros2 launch stage4_gazebo s3_navigation.launch.py camera:=true

ros2 topic hz /wheel_feedback /imu_odom /scan_filtered /odometry/filtered
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 run tf2_tools view_frames
ign topic -l                                                # topic phía Gazebo
```

## 11. Cạm bẫy đã biết

- `sllidar_ros2` phát góc 0 hướng **ra sau**; `scan_rotate` cộng π. Adapter sai quy ước → scan xoay 180° hoặc filter bỏ nhầm nửa trước.
- Sensor Gazebo đặt `frame_id` dạng `model/link/sensor`: phải ép về frame của dự án (trong SDF hoặc adapter).
- Plugin diff-drive của Gazebo tự phát odom/TF: không bridge chúng; nếu dùng diff-drive thì đặt giới hạn gia tốc rộng (ramp nằm ở `zlac_sim`). Cân nhắc điều khiển vận tốc từng joint bánh nếu cần ramp riêng từng bánh như ZLAC — ghi quyết định vào `PROGRESS.md`.
- `create_wall_timer` không theo sim time; `steady_clock` dùng để đo thời gian tính toán thì không sao.
- Không chạy `zlac8015d_move`, `move`, `m_to_m`, `imu_odom` (UART), `bno055_node`, `sllidar_ros2`, `realsense2_camera` trong sim.
- AMCL đóng dấu TF `map → odom` +1.0 s vào tương lai; CustomDWB đã tính (`max_tf_age`).
- Camera depth mô phỏng sạch hơn D435 thật: thêm nhiễu/lỗ dữ liệu trước khi kết luận về FusionLayer.