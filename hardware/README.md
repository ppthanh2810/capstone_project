# `hardware/`: phần cứng của robot

Thư mục này chứa mọi thứ gắn với phần cứng thật: driver motor, đọc IMU, driver LiDAR và mô hình robot (URDF).
Thuật toán (odometry, SLAM, điều hướng) nằm ở các package `stage*` bên ngoài.

Chi tiết từng phần:
- `robot_description/README.md`: kích thước, frame, TF.
- `sllidar_ros2/README_CAPSTONE.md`: LiDAR.
- `imu_bno055/README.md`: IMU.

---

## 1. Luồng dữ liệu tổng quát

```text
                         ┌──────────── máy tính (ROS 2 Humble) ────────────┐
RPLIDAR C1 ──/dev/ttyUSB2──> sllidar_node ──/scan_raw──> stage2_slam (xoay, lọc) ──> /scan_filtered ──> SLAM / AMCL / costmap

BNO055 ─I2C─> ESP32-C3 ──/dev/ttyUSB0──> imu_odom (stage1_odom) ──/imu_odom──┐
                                                                             ├─> EKF ──> /odometry/filtered, TF odom→base_footprint
ZLAC8015D <──Modbus RS485 /dev/ttyUSB1──> zlac8015d_move (stage0) ──/wheel_feedback──> zlac8015d_odom ──/wheel_odom──┘
     ▲                                          ▲
     └── bánh trái / phải                        └── /cmd_vel_out (twist_mux: PS4 hoặc DWB)

URDF (robot_description) ──> robot_state_publisher ──> TF base_footprint → base_link → imu_link, laser_frame, camera_*
```

| Thiết bị | Cổng | Package trong `hardware/` | Node đọc/ghi |
|---|---|---|---|
| ZLAC8015D | `/dev/ttyUSB1` | `zlac8015d_hw` (thư viện) | `zlac8015d_move` (stage0) |
| ESP32 + BNO055 | `/dev/ttyUSB0` | `imu_bno055` (bản cũ) | `imu_odom` (stage1) |
| RPLIDAR C1 | `/dev/ttyUSB2` | `sllidar_ros2` | `sllidar_node` |

Thứ tự ttyUSB có thể đổi khi cắm lại USB. Kiểm tra bằng `ls -l /dev/serial/by-id/`:
- FTDI FT232R là motor;
- CP2102 là IMU;
- CP2102N là LiDAR.

---

## 2. `zlac8015d_hw`: thư viện driver motor

**Vai trò:** nói chuyện với bộ điều khiển động cơ ZLAC8015D qua Modbus RTU. Đây chỉ là **thư viện C++**, không có node.
Node dùng nó là `stage0_ps4_control/zlac8015d_move` (và `move` cũ).

| File | Vai trò |
|---|---|
| `include/zlac8015d_hw/zlac8015d.hpp` | Khai báo lớp `Controller`, địa chỉ thanh ghi, mã lỗi |
| `src/zlac8015d.cpp` | Đọc/ghi thanh ghi bằng libmodbus |
| `CMakeLists.txt`, `package.xml` | Build thư viện, cần `libmodbus-dev` |

Thông số chính:

| Thông số | Giá trị |
|---|---|
| Giao tiếp | Modbus RTU, 115200, 8N1 |
| Slave ID | 1 |
| Timeout phản hồi | 20 ms |
| Chế độ dùng | `3` = điều khiển vận tốc (RPM) |
| Encoder | 4096 xung / vòng |
| Quãng đường 1 vòng bánh | 0.336 m |
| Bán kính bánh | 0.0535 m |

Các hàm chính:
- `setMode`, `enableMotor`, `disableMotor`;
- `setAccelTime`, `setDecelTime`;
- `setRpm`, `getRpm`;
- `getWheelsTick`, `getWheelsTravelled`;
- `clearAlarm`.

Thanh ghi hay dùng:

| Thanh ghi | Địa chỉ |
|---|---|
| Lệnh điều khiển | `0x200E` |
| Chế độ | `0x200D` |
| RPM lệnh trái / phải | `0x2088` / `0x2089` |
| RPM phản hồi trái / phải | `0x20AB` / `0x20AC` |
| Vị trí encoder | `0x20A7`–`0x20AA` |
| Mã lỗi trái / phải | `0x20A5` / `0x20A6` |
| Thời gian tăng tốc trái / phải | `0x2080` / `0x2081` |
| Thời gian giảm tốc trái / phải | `0x2082` / `0x2083` |

Phía node `zlac8015d_move`:
- Tham số: `wheel_radius` 0.0535, `wheel_distance` 0.34, `accel_time_ms` và `decel_time_ms` (cùng **500 ms**; từng
  là 200 ms từ 2026-10-14 tới 2026-10-19, xem `stage0_ps4_control/README.md`).
- Cổng `/dev/ttyUSB1` viết cứng trong code.
- Bánh phải bị đảo dấu khi gửi lệnh và khi đọc phản hồi.
- Mất lệnh 0.5 s thì đặt RPM về 0.

> `0.336` m/vòng được viết cứng trong thư viện. Bán kính bánh ở node là tham số riêng. Đổi bánh xe thì phải sửa
> **cả hai** chỗ.

---

## 3. `imu_bno055`: IMU (bản cũ, robot không dùng node này)

**Vai trò:** chỗ lưu firmware mẫu cho ESP32 và một node ROS cũ. Robot thật đọc IMU bằng
**`stage1_odom/imu_odom`**, không dùng `bno055_node`.

| File | Vai trò |
|---|---|
| `src/esp32.c` | Firmware Arduino mẫu: đọc BNO055 qua I2C, gửi UART. **Khác firmware đang nạp** |
| `src/bno055_node.cpp` | Node cũ: serial → `/imu/data` |
| `include/imu_bno055/bno055.hpp` | Mở cổng serial, tách dòng dữ liệu |
| `launch/bno055.launch.py` | Chạy `bno055_node` |
| `config/bno055.yaml` | Có khối calibration đã lưu; phần lớn tham số không được đọc |

Thông số chính:

| Thông số | Giá trị |
|---|---|
| Chế độ BNO055 | NDOF (`0x0C`) |
| I2C | địa chỉ `0x29`, SDA 8, SCL 9, 100 kHz |
| UART | 115200 8N1 |
| Tần số | ≈ 50 Hz |
| Gói dữ liệu `imu_odom` cần | `IMU,millis,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz` |
| Frame | `imu_link` |

> Không chạy `bno055_node` cùng `imu_odom`: hai node mở cùng một cổng.

> ⚠️ Sau khi cắm lại ESP32, có lúc nó **không gửi dòng `IMU,…` nào** (robot 2026-10-20, 13:48 → ~14:16) mà không có
> lỗi nào trong log. Khi đó yaw của EKF đứng yên. Kiểm tra `ros2 topic hz /imu_odom` (≈ 50 Hz); nếu 0 thì nhấn RESET
> ESP32 hoặc cắm lại, rồi xem dữ liệu thô (`stage1_odom/README.md` mục 8).

---

## 4. `sllidar_ros2`: driver LiDAR (code của hãng Slamtec)

**Vai trò:** đọc RPLIDAR C1, publish `LaserScan`. **Không sửa** code trong package này; tham số được truyền từ
`stage2_slam/launch/scan.launch.py`.

| File / thư mục | Vai trò |
|---|---|
| `src/sllidar_node.cpp` | Node driver |
| `launch/sllidar_c1_launch.py` | Launch dự án dùng (được `scan.launch.py` include) |
| `launch/` (các file khác) | Cho dòng LiDAR khác, không dùng |
| `sdk/` | SDK của hãng |
| `scripts/` | Udev rule của hãng, **không dùng** (có thể nhận nhầm IMU) |
| `README_CAPSTONE.md` | Ghi chú riêng của dự án |

Thông số chính:

| Thông số | Giá trị |
|---|---|
| Baudrate | 460800 |
| Chế độ quét | Standard, 5 kHz, 10 Hz |
| Tầm đo | 0.05 – 16 m |
| Số tia | 720 (0.5°) |
| `frame_id` của driver | `laser_raw` (cố ý không có trong TF) |

Sau driver, `stage2_slam`:
- `scan_rotate` cộng π vào góc vì driver đặt góc 0 ở phía sau robot;
- `laser_filters` bỏ nửa sau vì laptop che LiDAR;
- kết quả là `/scan_filtered`, frame `laser_frame`.

---

## 5. `robot_description`: mô hình robot

**Vai trò:** URDF mô tả vị trí từng frame, để `robot_state_publisher` phát TF tĩnh, và để vẽ robot trong RViz.
Không có khối lượng, không dùng để mô phỏng.

| File / thư mục | Vai trò |
|---|---|
| `urdf/robot.urdf` | Link, joint, vị trí frame (nguồn sự thật cho TF) |
| `cad/robot_desciption.scad` | Bản CAD gốc (giữ nguyên lỗi chính tả trong tên) |
| `meshes/*.stl` | Hình 3D xuất từ CAD (mm, scale 0.001) |
| `launch/display.launch.py` | `robot_state_publisher` (+ RViz tùy chọn) |
| `rviz/robot_description.rviz` | Cấu hình RViz |
| `README.md`, `readme.txt` | Bảng thông số chi tiết |

Thông số chính:

| Thông số | Giá trị |
|---|---|
| Thân | Ø0.400 m, tròn |
| Bánh chủ động | r = 0.0535 m, track 0.340 m |
| Caster | Ø75 mm, trước và sau |
| `base_link` | cao 0.1135 m so với mặt đất |
| `imu_link` | (0.110, 0, 0.2343) m |
| `laser_frame` | (0.150, 0, 0.34715) m |
| `camera_link` | (0.170, 0, 0.2760) m |

- Mọi joint đều `fixed`.
- Trên robot chạy với `publish_default_joint_states:=false`.

---

## 6. Lưu ý chung

- **Mọi package ở đây chạm phần cứng thật** (trừ `robot_description`). Chạy node motor thì kê bánh khỏi mặt đất và
  để E-stop trong tầm tay.
- **Thông số bánh xe phải đồng bộ:**
  - `wheel_radius = 0.0535` và `wheel_distance = 0.34` ở URDF, `zlac8015d_move.cpp`, `zlac8015d_odom.cpp`, các launch
    của `stage1_odom`;
  - và `TRAVEL_IN_ONE_REV = 0.336` ở thư viện motor.
- Vị trí cảm biến lấy từ CAD, **chưa hiệu chuẩn**.
- RealSense D435: frame trong URDF (`camera_link`, `camera_{color,depth}_optical_frame`). Driver chạy qua
  `stage3_navigation/launch/camera.launch.py` (`navigation.launch.py camera:=true`, `publish_tf:=false`). Không có
  package riêng trong `hardware/`. Độ nghiêng camera chưa đo (CAD = nằm ngang).
