# `stage1_odom`: odometry bánh xe + IMU + EKF

Package này trả lời câu hỏi **"robot đã đi bao xa và đang quay về hướng nào"** tính từ lúc khởi động. Nó gộp
encoder bánh xe và IMU bằng bộ lọc EKF, rồi phát:
- topic `/odometry/filtered`;
- TF **`odom → base_footprint`**.

AMCL, slam_toolbox và bộ điều khiển DWB đều dùng hai đầu ra này.

| File | Vai trò |
|---|---|
| `src/zlac8015d_odom.cpp` | Encoder bánh xe → `/wheel_odom` |
| `src/imu_odom.cpp` | Đọc IMU (ESP32 + BNO055) → `/imu_odom` |
| `config/ekf.yaml` | Cấu hình EKF (`robot_localization`) |
| `launch/ekf.launch.py` | Chạy odometry đầy đủ: bánh xe + IMU + EKF (**launch chính**) |
| `launch/wheel_odom_view.launch.py` | Xem riêng odometry bánh xe, không EKF (để thử nghiệm) |
| `rviz/odom.rviz` | RViz, Fixed Frame `odom` |
| `readme.txt` | Ghi chú cũ, có sơ đồ |

---

## 1. Luồng dữ liệu

```text
stage0: zlac8015d_move ──/wheel_feedback (20 Hz)──> zlac8015d_odom ──/wheel_odom──┐  (vx, vy)
                                                                                 ├──> ekf_filter_node (30 Hz) ──> /odometry/filtered
ESP32 + BNO055 ──/dev/ttyUSB0 (≈50 Hz)──> imu_odom ──/imu_odom───────────────────┘  (yaw, vyaw)        └──> TF odom → base_footprint
                                              ├──> /imu/data
                                              └──> /imu/mag
```

Mỗi nguồn đóng góp một phần:

| Nguồn | EKF lấy | Lý do |
|---|---|---|
| Bánh xe | vận tốc `vx`, `vy` | Encoder đo vận tốc tiến tốt |
| IMU | góc `yaw`, tốc độ quay `vyaw` | Hướng từ encoder sai khi bánh trượt / xoay tại chỗ |

- `vy` luôn là 0: robot vi sai không đi ngang được. Đưa `vy = 0` vào EKF giúp chặn trôi ngang.
- Vị trí (x, y) của `/wheel_odom` **không** đưa vào EKF. EKF tự tích phân vận tốc kết hợp với yaw của IMU.

> **Chỉ một node được phát TF `odom → base_footprint`.** Với `ekf.launch.py`, đó là EKF (`zlac8015d_odom` để
> `publish_tf: false`). Với `wheel_odom_view.launch.py`, đó là `zlac8015d_odom`. **Không chạy hai launch này cùng lúc.**

---

## 2. `src/zlac8015d_odom.cpp`: odometry bánh xe

**Vai trò:** đọc phản hồi bánh xe của `zlac8015d_move` (stage0), tính vị trí và vận tốc robot.

### Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `wheel_radius` | 0.0535 m | Bán kính bánh |
| `wheel_distance` | 0.34 m | Khoảng cách 2 bánh |
| `publish_tf` | `false` | `true`: tự phát TF `odom → base_footprint` |

Hằng số trong code:

| Hằng số | Giá trị | Dùng cho |
|---|---|---|
| `filter_alpha` | 0.2 | Lọc thông thấp vận tốc |
| `linear_deadband` | 0.004 m/s | Vận tốc nhỏ hơn → 0 |
| `angular_deadband` | 0.004 rad/s | Tốc độ quay nhỏ hơn → 0 |
| `distance_deadband` | 0.0005 m | Bỏ qua dịch chuyển rất nhỏ |

### Đầu vào: `/wheel_feedback`
`std_msgs/Float64MultiArray` gồm `[RPM trái, RPM phải, quãng đường trái (m), quãng đường phải (m)]`. Bánh phải đã
được stage0 đổi dấu. Mảng phải đủ 4 số và hữu hạn, nếu không thì bỏ.

### Cách tính

**Vị trí** (từ quãng đường encoder, mỗi tin nhắn):

```text
d_trái, d_phải = quãng đường hiện tại − lần trước
(nếu cả hai < 0.0005 m thì bỏ qua lần này)
d    = (d_trái + d_phải) / 2
dθ   = (d_phải − d_trái) / 0.34
x   += d · cos(θ + dθ/2)          (tích phân điểm giữa)
y   += d · sin(θ + dθ/2)
θ   += dθ
```

**Vận tốc** (từ RPM):

```text
v_bánh = RPM · 0.0535 · π/30
v      = (v_trái + v_phải) / 2
ω      = (v_phải − v_trái) / 0.34
lọc:  giá trị mới = 0.2 · đo + 0.8 · cũ, rồi qua deadband
```

Bộ lọc 0.2 ở 20 Hz làm vận tốc phản hồi chậm đi khoảng **0.2 s**. Đây là một phần của độ trễ lệnh → phản hồi
mà CustomDWB phải tính đến (`control_latency`).

### Đầu ra

| Topic / service | Nội dung |
|---|---|
| `/wheel_odom` | `nav_msgs/Odometry`, `odom` → `base_footprint` |
| TF `odom → base_footprint` | chỉ khi `publish_tf: true` |
| `/reset_odom` | service `std_srvs/Empty`: đặt x, y, yaw về 0 |

Covariance cố định:

| Đại lượng | Covariance |
|---|---|
| x, y, vx, vy | 0.01 |
| yaw, vyaw | 0.05 |
| z, roll, pitch | 1e6 (không đo) |

Reset odometry:

```bash
ros2 service call /reset_odom std_srvs/srv/Empty
```

---

## 3. `src/imu_odom.cpp`: IMU

**Vai trò:** đọc chuỗi dữ liệu từ ESP32 + BNO055 qua serial, đổi đơn vị, phát hướng (yaw) cho EKF.
Chi tiết phần cứng và firmware: `hardware/imu_bno055/README.md`.

### Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Cổng serial (115200, viết cứng) |
| `frame_id` | `imu_link` | Frame của `/imu/data`, `/imu/mag` |
| `zero_yaw_on_start` | `false` | `true`: hướng lúc khởi động = yaw 0 |

### Đầu vào
Mỗi dòng phải có đúng 15 trường:

```text
IMU,millis,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz
```

Gyro tính bằng độ/s, gia tốc bằng m/s², từ trường bằng µT. Dòng sai định dạng bị bỏ **im lặng** (không log).

> ⚠️ Node **không cảnh báo khi không có dòng hợp lệ nào**: log chỉ có `Connected to /dev/ttyUSB0` rồi im. Dấu hiệu
> đang chạy đúng là dòng `IMU ODOM | Yaw: …` cứ 2 s một lần. Không có dòng này = `/imu_odom` không được phát.

### Đầu ra

| Topic / service | Nội dung |
|---|---|
| `/imu_odom` | `nav_msgs/Odometry`: chỉ có yaw + `angular.z` (cho EKF) |
| `/imu/data` | `sensor_msgs/Imu` đầy đủ (chưa node nào dùng) |
| `/imu/mag` | `sensor_msgs/MagneticField`, Tesla (chưa node nào dùng) |
| `/imu_odom/zero_yaw` | service `std_srvs/Empty`: đặt hướng hiện tại = 0 |

- Thời gian gắn vào message là giờ của máy tính lúc nhận, không dùng `millis` của ESP32.
- Covariance của `/imu_odom`: yaw 0.01, vyaw 0.0004, phần còn lại 1e6.

---

## 4. `config/ekf.yaml`: bộ lọc EKF

**Vai trò:** cấu hình node `ekf_filter_node` (package `robot_localization`). Node này gộp hai nguồn trên.

### Thông số chung

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `frequency` | 30 Hz | Tần số tính và phát |
| `sensor_timeout` | 0.2 s | Không có dữ liệu lâu hơn → chỉ dự đoán |
| `two_d_mode` | `true` | Chỉ x, y, yaw (bỏ z, roll, pitch) |
| `publish_tf` | `true` | Phát TF `odom → base_footprint` |
| `world_frame` | `odom` | Frame gốc |
| `base_link_frame` | `base_footprint` | Frame robot |
| `print_diagnostics` | `true` | Báo lỗi cấu hình trên `/diagnostics` |

### Nguồn đầu vào

| | `odom0` | `odom1` |
|---|---|---|
| Topic | `/wheel_odom` | `/imu_odom` |
| Dùng | `vx`, `vy` | `yaw`, `vyaw` |
| `relative` | `false` | **`true`** |
| `differential` | `false` | `false` |

- **`odom1_relative: true`**: EKF lấy yaw của mẫu IMU đầu tiên làm mốc 0. BNO055 (chế độ NDOF) trả yaw tuyệt đối theo
  hướng Bắc, nhưng frame `odom` vẫn bắt đầu với yaw 0.
- Mỗi dòng `odomX_config` có 15 ô, theo thứ tự:
  `x y z | roll pitch yaw | vx vy vz | vroll vpitch vyaw | ax ay az`.
- Không khai báo `process_noise_covariance`, nên dùng giá trị mặc định của `robot_localization` (**chưa tinh chỉnh**).

---

## 5. `launch/ekf.launch.py`: launch chính

**Vai trò:** chạy odometry đầy đủ. **Không bật motor.** Có mở cổng serial IMU.

```bash
ros2 launch stage1_odom ekf.launch.py                 # có RViz
ros2 launch stage1_odom ekf.launch.py rviz:=false
```

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `imu_port` | `/dev/ttyUSB0` | Cổng IMU |
| `rviz` | `true` | Mở RViz + `robot_state_publisher` (URDF) |

| Node | Tham số |
|---|---|
| `zlac8015d_odom` | 0.0535 / 0.34, `publish_tf: false` |
| `imu_odom` | `port` = `imu_port`, `frame_id: imu_link` |
| `ekf_filter_node` | `config/ekf.yaml` |
| `robot_state_publisher` + `rviz2_odom` | chỉ khi `rviz:=true` |

Ghi chú:
- `/wheel_odom` chỉ có dữ liệu khi stage0 (`zlac8015d_move`) đang chạy.
- Stage 2 (`mapping.launch.py`, `localization.launch.py`) include file này với `rviz:=false`, vì `scan.launch.py` đã
  chạy `robot_state_publisher` và RViz riêng.

---

## 6. `launch/wheel_odom_view.launch.py`: thử odometry bánh xe

**Vai trò:** lái bằng PS4 và xem vết odometry bánh xe trong RViz, **không có EKF**. Dùng để kiểm tra bánh xe, so sánh
hướng IMU với hướng bánh xe.

> ⚠️ **Bật motor mặc định** (`motor:=true`). Kê bánh khỏi mặt đất khi thử lần đầu.

```bash
ros2 launch stage1_odom wheel_odom_view.launch.py              # bật motor
ros2 launch stage1_odom wheel_odom_view.launch.py motor:=false # chỉ xem, không bật motor
```

| Đối số | Mặc định | Ý nghĩa |
|---|---|---|
| `motor` | `true` | Chạy stage0 `move_twist_mux` (PS4 + motor) |
| `imu` | `true` | Chạy `imu_odom` để so sánh hướng |
| `imu_port` | `/dev/ttyUSB0` | Cổng IMU |
| `rviz` | `true` | Mở RViz `odom.rviz` |

Khác với `ekf.launch.py`:
- `zlac8015d_odom` có **`publish_tf: true`**: TF lấy hoàn toàn từ bánh xe.
- `imu_odom` có **`zero_yaw_on_start: true`**: yaw IMU lúc đầu = 0, trùng với bánh xe để dễ so sánh.
- Sau `/reset_odom`, gọi thêm `/imu_odom/zero_yaw` để IMU cũng về 0.

---

## 7. Kiểm tra nhanh (chỉ đọc, khi launch đang chạy)

```bash
ros2 topic hz /wheel_odom              # ≈ 20 Hz (khi stage0 chạy)
ros2 topic hz /imu_odom                # ≈ 50 Hz
ros2 topic hz /odometry/filtered       # ≈ 30 Hz
ros2 run tf2_ros tf2_echo odom base_footprint
```

Thử bằng tay:
- Đẩy robot tiến 1 m: `x` của `/odometry/filtered` tăng khoảng 1.0.
- Xoay robot ngược chiều kim đồng hồ (nhìn từ trên): yaw tăng.

## 8. Lưu ý / giới hạn

- Bán kính bánh 0.0535 m và track 0.34 m là giá trị danh nghĩa theo CAD, **chưa hiệu chuẩn** bằng phép đo quãng đường
  và quay tại chỗ.
- Mọi covariance (bánh xe, IMU) và nhiễu quá trình của EKF là giá trị đặt tay, **chưa đo**.
- Kết quả robot 2026-10-14: yaw của IMU và yaw tính từ bánh xe lệch nhau không quá 0.03 rad khi quay 135°.
  Độ trễ lệnh → `/odometry/filtered` đo được khoảng 0.2 s (driver tăng/giảm tốc 200 ms; nay mặc định lại 500 ms).
- **Hướng của EKF chỉ đến từ IMU** (`/wheel_odom` chỉ góp `vx`, `vy`). IMU mất dữ liệu → yaw của
  `odom → base_footprint` **đứng yên** dù robot quay. Ngày 2026-10-20 (từ 13:56, sau khi cắm lại ESP32 lúc 13:48)
  `imu_odom` không nhận được dòng hợp lệ nào: RViz thấy scan xoay lệch bản đồ, monitor báo LOST, CustomDWB quay
  tại chỗ ~270° rồi abort. Trước khi chạy stage 2/3, luôn kiểm tra `ros2 topic hz /imu_odom` ≈ 50 Hz.
- Xem dữ liệu thô của ESP32 (không chạy node nào khác trên cổng này):
  `stty -F /dev/ttyUSB0 115200 raw -echo && timeout 3 cat /dev/ttyUSB0 | head -5`.
- `package.xml` không khai báo `zlac8015d_hw`, dù `CMakeLists.txt` có `find_package` nó. Build vẫn chạy vì
  `stage0_ps4_control` được build trước.
