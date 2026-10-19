# `robot_description` — Thông số mô hình robot AMR (ROS 2 Humble)

Package `ament_cmake` chứa URDF, mesh STL, cấu hình RViz và `display.launch.py` cho robot vi sai thân tròn Ø400 mm
(2 bánh chủ động + 2 bánh caster). Mô hình được dựng từ `cad/robot_desciption.scad` (giữ nguyên lỗi chính tả, **không
đổi tên**). URDF đổi kích thước CAD từ **mm → m**, góc → **rad**.

- Quy ước REP-103: **+X trước, +Y trái, +Z lên**. Đơn vị SI (m, rad).
- Nguồn sự thật: `urdf/robot.urdf` (vị trí frame) và `cad/robot_desciption.scad` (hình học). Các con số dưới đây
  lấy từ hai file đó và từ code của các package khác (ghi rõ nguồn). **Tất cả là giá trị danh nghĩa theo CAD, chưa
  hiệu chuẩn** (xem mục 8).
- `readme.txt` là ghi chú cũ (bảng mm theo SCAD), vẫn khớp với README này.

---

## 1. Thông số tổng quát

| Thông số | Giá trị | Nguồn |
|---|---|---|
| Kiểu dẫn động | Vi sai (differential drive), 2 bánh chủ động + 2 caster (trước/sau) | URDF |
| Đường kính thân (đế) | **0.400 m** (bán kính 0.200 m) | URDF `cylinder radius 0.2` |
| Bán kính bánh chủ động | **0.0535 m** (Ø107 mm) | URDF, `wheel_radius` trong `zlac8015d_move.cpp`, `zlac8015d_odom.cpp` |
| Bề rộng lốp chủ động | 0.0415 m | URDF |
| Khoảng cách 2 bánh (track) | **0.340 m** (tâm bánh tại y = ±0.170) | URDF, `wheel_distance` trong `zlac8015d_move.cpp`, `zlac8015d_odom.cpp` |
| Mép ngoài bánh | y = ±0.19075 m (nằm trong vòng thân Ø400) | 0.170 + 0.0415/2 |
| Bánh caster | Ø75 mm, rộng 30 mm, tâm bánh (±0.068, 0, 0.0375) | URDF |
| Gá caster | x = ±0.120 m (bánh lệch 0.052 m về phía tâm ở tư thế danh nghĩa) | URDF / SCAD |
| Chiều cao đế dưới (mặt đáy) | 0.1135 m so với mặt đất | `base_footprint_joint` |
| Chiều cao tối đa (đỉnh LiDAR, theo collision) | ≈ 0.368 m | 0.34715 + 0.0418/2 |
| Mặt phẳng quét LiDAR (gốc `laser_frame`) | z = 0.34715 m | `lidar_joint` (chưa đo, xem mục 8) |
| Khối lượng / quán tính | **không có** trong URDF | – |

### Thông số dùng ở các package khác (phải khớp với mô hình)

| Thông số | Giá trị | File |
|---|---|---|
| `wheel_radius` / `wheel_distance` | 0.0535 / 0.34 | `stage0_ps4_control/src/zlac8015d_move.cpp`, `stage1_odom/src/zlac8015d_odom.cpp`, `stage1_odom/launch/wheel_odom_view.launch.py`, `stage1_odom/launch/ekf.launch.py` |
| `robot_radius` global costmap | 0.20 m | `stage3_navigation/config/nav2_params.yaml` |
| `robot_radius` + `footprint_padding` local costmap | 0.20 + 0.10 m (nội tiếp 0.30 m) | `stage3_navigation/config/nav2_params.yaml` |
| Khung odometry | `odom → base_footprint` | `stage1_odom/config/ekf.yaml` (`base_link_frame: base_footprint`) |

> Đổi bán kính bánh hoặc track thì phải sửa **đồng thời** URDF, SCAD, và tất cả các file trên.

### Động học (theo `zlac8015d_move.cpp`)

```
ω_trái  = (v − ω·L/2) / r          L = 0.34 m, r = 0.0535 m
ω_phải  = (v + ω·L/2) / r
RPM     = ω_bánh · 30/π            (bánh phải đảo dấu khi gửi lệnh và khi đọc phản hồi)
```

- RPM gửi cho ZLAC8015D được làm tròn về số nguyên, nên độ phân giải ≈ 2π/60 · 0.0535 ≈ **0.0056 m/s** mỗi bánh.
- Ví dụ: 0.15 m/s (max của DWB) ≈ 26.8 RPM; 0.2 m/s (max teleop hiện tại, `stage0_ps4_control/config/ps4.yaml`) ≈ 35.7 RPM.
- Xoay tại chỗ 0.5 rad/s → mỗi bánh 0.085 m/s ≈ 15.2 RPM.

---

## 2. Cây TF

```text
map  --[slam_toolbox | AMCL]-->  odom  --[ekf_filter_node]-->  base_footprint
                                                                    |
                                        base_footprint_joint (fixed, z +0.1135 m)
                                                                    |
                                                                base_link   (mang toàn bộ visual thân, motor, bánh, caster)
  +-- imu_link                     (imu_joint, fixed)
  +-- camera_link                  (camera_joint, fixed)
  |     +-- camera_color_optical_frame   (fixed, rpy −π/2, 0, −π/2)
  |     +-- camera_depth_optical_frame   (fixed, rpy −π/2, 0, −π/2)
  +-- lidar_link                   (lidar_joint, fixed)
        +-- laser_frame            (laser_joint, fixed, identity)
```

- 8 link, 7 joint, **tất cả `fixed`**. Bánh xe không có joint, nên chuyển động quay của bánh không có trong TF và không
  cần `/joint_states`.
- `map → odom` và `odom → base_footprint` **không** nằm trong URDF. Mỗi cạnh TF chỉ có **một** publisher:
  - `map → odom`: slam_toolbox (khi mapping) **hoặc** AMCL (khi localization), không bao giờ cả hai.
  - `odom → base_footprint`: EKF (`zlac8015d_odom` để `publish_tf: false`).
- Không bao giờ publish `odom → base_link`, vì `base_link` đã có cha là `base_footprint`.

---

## 3. Bảng joint (URDF, frame cha → con)

| Joint | Cha → Con | xyz (m) | rpy (rad) |
|---|---|---|---|
| `base_footprint_joint` | `base_footprint` → `base_link` | 0, 0, 0.1135 | 0, 0, 0 |
| `imu_joint` | `base_link` → `imu_link` | 0.110, 0, 0.1208 | 0, 0, 0 |
| `camera_joint` | `base_link` → `camera_link` | 0.170, 0, 0.1625 | 0, 0, 0 |
| `camera_color_optical_frame_joint` | `camera_link` → `camera_color_optical_frame` | 0, 0, 0 | −π/2, 0, −π/2 |
| `camera_depth_optical_frame_joint` | `camera_link` → `camera_depth_optical_frame` | 0, 0, 0 | −π/2, 0, −π/2 |
| `lidar_joint` | `base_link` → `lidar_link` | 0.150, 0, 0.23365 | 0, 0, 0 |
| `laser_joint` | `lidar_link` → `laser_frame` | 0, 0, 0 | 0, 0, 0 |

## 4. Vị trí frame so với `base_footprint` (mặt đất)

| Frame | xyz (m) | Hướng | Ghi chú |
|---|---|---|---|
| `base_footprint` | 0, 0, 0 | – | Hình chiếu tâm robot xuống mặt đất; tâm quay danh nghĩa (giữa 2 bánh) |
| `base_link` | 0, 0, 0.1135 | identity | Tâm mặt đáy của đế dưới |
| `imu_link` | 0.110, 0, 0.2343 | identity | Chỉ visual của board IMU xoay +90° quanh Z; **frame không xoay** |
| `camera_link` | 0.170, 0, 0.2760 | identity | Tâm mô hình RealSense D435 |
| `camera_*_optical_frame` | 0.170, 0, 0.2760 | rpy (−π/2, 0, −π/2) | Quy ước quang học REP-103 (z tới trước, x sang phải, y xuống) |
| `lidar_link` / `laser_frame` | 0.150, 0, 0.34715 | identity | `frame_id` của `/scan` (do `stage2_slam/scan_rotate` đặt) |

`frame_id` các message cảm biến phải khớp tên link: `imu_link`, `laser_frame`, `camera_color_optical_frame`,
`camera_depth_optical_frame`.

---

## 5. Hình học chi tiết (visual trong `base_link`)

Cột "z mặt đất" = z trong `base_link` + 0.1135.

| Chi tiết | Kích thước | Tâm trong `base_link` (m) | z mặt đất (m) | Hình dạng URDF |
|---|---|---|---|---|
| Đế dưới | Ø400 × 5 mm | (0, 0, 0.0025) | 0.1135 → 0.1185 | cylinder (+ collision) |
| 4 trụ nối 2 tầng | Ø12 × 100 mm | (±0.12, ±0.10, 0.055) | 0.1185 → 0.2185 | cylinder |
| Đế trên | Ø400 × 5 mm | (0, 0, 0.1075) | 0.2185 → 0.2235 | cylinder (+ collision) |
| Breadboard (mạch điện, mang IMU) | 170 × 63 × 10 mm, xoay 90° (cạnh dài theo Y) | (0.11, 0, 0.115) | 0.2235 → 0.2335 | box |
| Board IMU (BNO055) | 21 × 12 × 1.6 mm, xoay 90° | = `imu_link` | 0.2343 (tâm) | box (visual của `imu_link`) |
| Giá camera | 23 × 23 × 40 mm | đáy (0.17, 0, 0.11) | 0.2235 → 0.2635 | mesh `camera_mount_photo.stl` |
| RealSense D435 | 25 × 90 × 25 mm (STL 26.575 × 90 × 25.19) | = `camera_link` | 0.2635 → 0.2885 | mesh `camera_d435.stl` + collision box |
| 2 trụ đỡ tấm LiDAR | Ø12 × 100 mm | (0.17, ±0.06, 0.16) | 0.2235 → 0.3235 | cylinder |
| Tấm LiDAR | 90 × 170 × 3 mm, x từ 0.110 → 0.200 | z 0.2115 | ≈ 0.325 (tâm) | mesh `lidar_plate.stl` |
| RPLIDAR C1 | 55.6 × 55.6 × 41.8 mm | = `lidar_link` | collision 0.326 → 0.368 | mesh `rplidar.stl` + collision cylinder r 0.0278 |
| Motor trái / phải | 110 × 25 × 40 mm | (0, ±0.10, −0.02) | 0.0735 → 0.1135 | box |
| Tấm gá motor | 50 × 10 × 80 mm (lỗ trục ở z mặt đất 0.0535) | (0, ±0.1175, −0.04) | ≈ 0.0335 → 0.1135 | mesh `motor_plate.stl` |
| Bánh chủ động trái / phải | lốp Ø107 × 41.5 mm (STL cả hub 107 × 88.7 × 107) | (0, ±0.17, −0.06) | trục 0.0535, chạm đất 0 | mesh `drive_wheel.stl` + collision cylinder |
| Caster trước — tấm gá | 55 × 75 × 3 mm | (0.12, 0, −0.0015) | ≈ 0.112 | box |
| Caster trước — khớp xoay | Ø40 × 7 mm | (0.12, 0, −0.0085) | ≈ 0.105 | cylinder |
| Caster trước — càng | 50 × 35 × 48 mm | (0.108, 0, −0.0425) | ≈ 0.071 | box |
| Caster trước — bánh | Ø75 × 30 mm | (0.068, 0, −0.076) | trục 0.0375, chạm đất 0 | cylinder (+ collision) |
| Caster sau | đối xứng caster trước, xoay 180° quanh Z | x âm: −0.12 / −0.108 / −0.068 | như trên | như trên |

- Ở cấu hình CAD danh nghĩa, cả 4 bánh chạm mặt phẳng `base_footprint` (z = 0).
- Gầm thấp nhất (ngoài bánh): tấm gá motor ≈ 0.034 m, càng caster ≈ 0.047 m so với mặt đất (theo CAD).
- Mesh STL xuất từ OpenSCAD theo **mm**, URDF dùng `scale="0.001 0.001 0.001"`.

### Collision (đơn giản hóa)

Chỉ có: 2 đế (cylinder Ø400 × 5), 2 bánh chủ động (cylinder), 2 bánh caster (cylinder), camera (box 25 × 90 × 25),
LiDAR (cylinder r 0.0278 × 0.0418). Mô hình này **không** dùng để chứng minh khoảng hở. Nav2 không dùng collision
URDF; nó dùng `robot_radius` trong `nav2_params.yaml`.

### Màu (material)

`chassis_gray` (đế dưới), `silver` (đế trên, giá camera, tấm gá, camera), `gold` (trụ, khớp xoay caster), `steel_blue`
(tấm và trụ LiDAR), `white` (breadboard, motor), `dark_black` (bánh chủ động, LiDAR), `rubber` (bánh caster),
`imu_blue` (board IMU). Đây là màu gần đúng, không giữ màu từng chi tiết của OpenSCAD.

---

## 6. Cảm biến gắn trên robot

| Cảm biến | Frame | Vị trí (m, so với `base_footprint`) | Kết nối / topic | Ghi chú |
|---|---|---|---|---|
| RPLIDAR C1 | `laser_frame` | (0.150, 0, 0.34715) | `/dev/ttyUSB2`, 460800 baud → `/scan_raw` → `/scan` → `/scan_filtered` | Driver cho góc 0 = **phía sau**; `scan_rotate` cộng π vào dữ liệu (không sửa TF). `laser_filters` bỏ nửa sau (π−1.57, π+1.57) vì laptop che |
| BNO055 (qua ESP32-C3) | `imu_link` | (0.110, 0, 0.2343) | `/dev/ttyUSB1`, 115200 → `/imu/data`, `/imu_odom` | Frame không xoay so với `base_link` |
| RealSense D435 | `camera_link`, `camera_*_optical_frame` | (0.170, 0, 0.2760) | USB | Chưa có trong launch stage nào. Khi tích hợp, đặt `publish_tf:=false` để không trùng cạnh TF với URDF |

Motor driver ZLAC8015D: Modbus RTU qua `/dev/ttyUSB0` (không có frame). Thứ tự ttyUSB có thể đổi khi cắm lại USB, nên
kiểm tra bằng `ls -l /dev/serial/by-id/`.

---

## 7. Cài đặt và hiển thị

Build từ `~/capstone_ws`:

```bash
colcon build --packages-select robot_description --symlink-install
source install/setup.bash
ros2 launch robot_description display.launch.py
```

Tham số của `display.launch.py`:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `rviz` | `true` | Mở RViz (`rviz/robot_description.rviz`) |
| `publish_default_joint_states` | `true` | Chạy `joint_state_publisher`. Không cần thiết vì mọi joint đều fixed; **trên robot dùng `false`** (`stage2_slam/scan.launch.py` đã truyền `false`) |
| `use_sim_time` | `false` | Robot thật: `false` (dự án không có mô phỏng) |

Chỉ xem TF, không mở RViz (an toàn, không đụng phần cứng):

```bash
ros2 launch robot_description display.launch.py rviz:=false publish_default_joint_states:=false
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 run tf2_tools view_frames
check_urdf capstone_project/hardware/robot_description/urdf/robot.urdf
```

Chỉ chạy **một** `robot_state_publisher` cho mô hình này. Toàn bộ cây được publish trên `/tf_static`.

---

## 8. Giả định / giới hạn — cần hiệu chuẩn trước khi dùng cho điều hướng

- **Extrinsic lấy từ CAD, chưa hiệu chuẩn**: `imu_link`, `lidar_link`/`laser_frame`, các frame camera. Chưa đo chiều
  cao mặt quét LiDAR và sai lệch yaw còn lại.
- Frame quang học camera trùng tâm mô hình D435 (CAD không cho vị trí ống kính). Đây chỉ là giá trị giữ chỗ, không
  phải intrinsic hay extrinsic màu–độ sâu.
- `base_footprint` được giả định là tâm quay (giữa 2 bánh) và tâm thân tròn. Bộ kiểm tra va chạm của CustomDWB (chỉ
  xét ô tâm) dựa trên giả định này; **chưa kiểm chứng trên robot**.
- Bán kính bánh 0.0535 m và track 0.34 m là giá trị danh nghĩa, chưa hiệu chuẩn bằng phép đo quãng đường / quay
  tại chỗ (độ lún lốp, sai số chế tạo).
- Caster được vẽ đơn giản hóa; vị trí bánh và đường kính lốp giữ theo CAD. Hướng caster thực tế thay đổi khi chạy.
- Không có khối lượng, quán tính, truyền động hay `ros2_control`: package chỉ dùng cho TF và hiển thị RViz, **không**
  phải mô hình mô phỏng vật lý.
- Khi CAD thay đổi: sửa SCAD → xuất lại STL (mm) → cập nhật origin trong URDF → cập nhật bảng ở README này (và
  `readme.txt`). Không sửa STL bằng tay.
