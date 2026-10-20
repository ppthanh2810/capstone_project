# IMU BNO055 + ESP32-C3 Mini trong dự án capstone

File này ghi lại cách robot đang dùng IMU: phần cứng nối thế nào, ESP32 gửi gì, node nào đọc, tham số ra sao,
và IMU đóng vai trò gì trong định vị.

> **Điều quan trọng nhất:** robot **không dùng node `bno055_node` của package này**. Mọi launch của stage1–3 đọc IMU
> bằng node **`imu_odom`** trong package `stage1_odom` (`stage1_odom/src/imu_odom.cpp`). Package `imu_bno055` hiện chỉ
> còn là bản cũ và chỗ lưu firmware mẫu (xem mục 9).

---

## 1. Tóm tắt nhanh

| Mục | Giá trị |
|---|---|
| Cảm biến | Bosch BNO055 (9 trục, có bộ lọc fusion bên trong) |
| Chế độ | NDOF (`0x0C`): gia tốc + gyro + từ kế |
| Vi điều khiển | ESP32-C3 Mini |
| Kết nối BNO055 ↔ ESP32 | I2C, địa chỉ `0x29`, 100 kHz |
| Kết nối ESP32 ↔ máy tính | UART 115200 8N1, qua adapter USB-UART |
| Cổng trên máy tính | `/dev/ttyUSB0` (adapter Silicon Labs CP2102) |
| Tần số dữ liệu | ≈ 50 Hz (đo từ bag `~/rot_debug`) |
| Node đọc | `imu_odom` (package `stage1_odom`) |
| Frame | `imu_link` |
| Topic chính cho EKF | `/imu_odom` |

---

## 2. Vai trò của IMU trong robot

IMU chỉ cung cấp **hướng (yaw) và tốc độ quay (yaw rate)**. Vị trí và vận tốc tiến lấy từ encoder bánh xe.

| Nguồn | Đưa gì vào EKF |
|---|---|
| Encoder (`/wheel_odom`) | vận tốc `vx`, `vy` |
| IMU (`/imu_odom`) | góc `yaw`, tốc độ quay `vyaw` |

Vì sao cần IMU:
- Khi xoay tại chỗ hoặc bánh bị trượt, yaw tính từ encoder sai dần theo thời gian.
- Yaw của IMU không phụ thuộc vào bánh xe.
- EKF kết hợp hai nguồn để ra `/odometry/filtered` và TF `odom → base_footprint`.
- AMCL, slam_toolbox và bộ điều khiển DWB đều dùng kết quả này.

Kết quả trên robot (2026-10-14): khi quay 135°, yaw của IMU và yaw tính từ bánh xe lệch nhau không quá 0.03 rad.

---

## 3. Luồng dữ liệu

```text
BNO055 ──I2C 0x29──> ESP32-C3 Mini ──UART 115200──> USB-UART (CP2102) ──/dev/ttyUSB0──> imu_odom
                                                                                       │
                       ┌───────────────────────┬─────────────────────┬─────────────────┘
                       ▼                       ▼                     ▼
                  /imu/data               /imu/mag              /imu_odom ──> ekf_filter_node
                (chưa node nào dùng)  (chưa node nào dùng)     (yaw, vyaw)        │
                                                                                  ▼
                                                    /odometry/filtered + TF odom → base_footprint
```

---

## 4. Phần cứng và nối dây

| Mục | Giá trị | Nguồn |
|---|---|---|
| Chân SDA của ESP32 | GPIO 8 | `src/esp32.c` |
| Chân SCL của ESP32 | GPIO 9 | `src/esp32.c` |
| Địa chỉ I2C BNO055 | `0x29` | `src/esp32.c` |
| Tốc độ I2C | 100 kHz | `src/esp32.c` |
| UART của ESP32 | TX 20, RX 21 | `stage1_odom/readme.txt` |

- Địa chỉ `0x29` nghĩa là chân ADR của BNO055 đang kéo lên mức cao. Nếu kéo xuống thấp thì địa chỉ là `0x28`.
- Chân UART lấy theo ghi chú trong `readme.txt`, **chưa đối chiếu với firmware đang nạp**.
- ESP32 gửi dữ liệu qua adapter USB-UART CP2102, không qua cổng USB của board.
- Kiểm tra adapter: `ls -l /dev/serial/by-id/`. IMU là **CP2102**, LiDAR là **CP2102N**. Hai chip này cùng hãng, nên
  phải nhìn kỹ tên.

---

## 5. Firmware ESP32: gói dữ liệu gửi lên

Mỗi dòng là một chuỗi chữ, kết thúc bằng `\n`. `imu_odom` cần **đúng 15 trường**:

```text
IMU,millis,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz
```

| Trường | Ý nghĩa | Đơn vị ESP32 phải gửi |
|---|---|---|
| `IMU` | nhãn đầu dòng | – |
| `millis` | thời gian của ESP32 | ms (máy tính **bỏ qua**) |
| `qw qx qy qz` | quaternion hướng | không đơn vị (≈ chuẩn 1) |
| `gx gy gz` | tốc độ quay | **độ/s** |
| `ax ay az` | gia tốc (có cả trọng lực) | **m/s²** |
| `mx my mz` | từ trường | **µT** |

ESP32 phải **đổi đơn vị trước khi gửi**. Hệ số đổi từ số thô của BNO055 (theo datasheet):

| Đại lượng | Số thô → đơn vị |
|---|---|
| Quaternion | chia 16384 |
| Gyro | chia 16 → độ/s |
| Gia tốc | chia 100 → m/s² |
| Từ kế | chia 16 → µT |

Thanh ghi BNO055 mà firmware mẫu dùng:
- quaternion `0x20`;
- gyro `0x14`;
- gia tốc `0x28`.

Từ kế nằm ở `0x0E`; firmware mẫu chưa đọc thanh ghi này.

Firmware khởi động BNO055 theo thứ tự:
1. Đọc CHIP_ID, phải bằng `0xA0`.
2. Chuyển sang page 0, rồi CONFIG mode.
3. Đặt Normal power.
4. Chuyển sang **NDOF** và kiểm tra lại mode đã là `0x0C`.

Lỗi khởi tạo thì firmware in `BNO055 initialization failed` và đứng chờ.

> ⚠️ **Firmware trong repo khác firmware đang nạp.** `src/esp32.c` gửi **12 trường** (`IMU`, `millis` và 10 giá trị; thiếu từ kế), và là **số thô
> chưa đổi đơn vị**. `imu_odom` sẽ bỏ mọi dòng như vậy. Firmware đang chạy trên ESP32 (gửi 15 trường đã đổi đơn vị)
> **chưa được lưu vào repo**. Nên chép file đó vào repo để không bị mất.

---

## 6. Node `imu_odom` (`stage1_odom/src/imu_odom.cpp`)

**Vai trò:** mở cổng serial, đọc từng dòng, kiểm tra hợp lệ, đổi đơn vị và publish 3 topic.

### Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Cổng serial |
| `frame_id` | `imu_link` | Frame của `/imu/data`, `/imu/mag` |
| `zero_yaw_on_start` | `false` | `true`: hướng lúc khởi động coi là yaw 0 |

- Baudrate **cố định 115200** trong code, không có tham số để đổi.
- Launch truyền `port` qua đối số `imu_port`. Các file có đối số này: `ekf.launch.py`, `wheel_odom_view.launch.py`,
  `mapping` / `localization.launch.py`, `planning` / `navigation.launch.py`.
- `wheel_odom_view.launch.py` đặt `zero_yaw_on_start: true`. Mọi launch khác để `false`.

### Cách xử lý mỗi dòng
1. Dòng phải bắt đầu bằng `IMU,` và có đúng 15 trường.
2. Mọi số phải đọc được và hữu hạn. Dòng dài quá 255 ký tự thì bỏ.
3. Độ dài quaternion phải trong khoảng 0.5 – 1.5, sau đó chuẩn hóa về 1.
4. Gyro đổi độ/s → rad/s. Từ kế đổi µT → Tesla.
5. Gắn **thời gian nhận trên máy tính** (`now()`), không dùng `millis` của ESP32.
6. Tính yaw từ quaternion. Nếu bật `zero_yaw_on_start` thì trừ yaw ban đầu.

Đọc serial:
- Timer 5 ms, mỗi lần đọc tối đa 16 × 512 byte.
- Rút cáp hoặc lỗi đọc: đóng cổng và thử mở lại mỗi 1 s.

### Topic

| Topic | Kiểu | QoS |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | best effort |
| `/imu/mag` | `sensor_msgs/MagneticField` | best effort |
| `/imu_odom` | `nav_msgs/Odometry` | reliable, 10 |

`/imu_odom` có:
- `frame_id: odom`, `child_frame_id: base_footprint`.
- Chỉ có yaw (trong orientation) và `angular.z` (gyro z).
- Vị trí và vận tốc khác bằng 0, kèm covariance `1e6` ("không đo").

Covariance cố định (chưa đo thật):

| Đại lượng | Covariance |
|---|---|
| Hướng | 0.01 |
| Tốc độ quay | 0.0004 |
| Gia tốc | 0.04 |

### Service

```bash
ros2 service call /imu_odom/zero_yaw std_srvs/srv/Empty
```

Lệnh này đặt hướng hiện tại thành yaw 0 ở mẫu kế tiếp. Thường gọi sau `/reset_odom` của wheel odom.

---

## 7. EKF dùng IMU như thế nào (`stage1_odom/config/ekf.yaml`)

| Tham số | Giá trị |
|---|---|
| `frequency` | 30 Hz |
| `two_d_mode` | `true` |
| `odom1` | `/imu_odom` |
| `odom1_config` | chỉ bật `yaw` và `vyaw` |
| `odom1_relative` | `true` |
| `odom0` (bánh xe) | chỉ bật `vx`, `vy` |

- **`odom1_relative: true`**: EKF lấy yaw của mẫu IMU đầu tiên làm mốc 0. Vì vậy dù BNO055 NDOF trả yaw tuyệt đối
  (theo hướng Bắc từ trường), frame `odom` vẫn bắt đầu với yaw 0.
- `zero_yaw_on_start: false` là đúng khi chạy với EKF: việc đặt mốc do EKF làm.

---

## 8. Vị trí IMU trên robot (TF, từ URDF)

| Mục | Giá trị |
|---|---|
| Cạnh TF | `base_link → imu_link` |
| xyz so với `base_link` | 0.110, 0, 0.1208 m |
| xyz so với mặt đất | 0.110, 0, 0.2343 m |
| Hướng | không xoay (trục trùng robot) |

- URDF giả định trục BNO055 trùng trục robot: X trước, Y trái, Z lên. Board IMU chỉ xoay 90° trong phần hình vẽ.
- Thứ tự trục của chip (axis remap) **chưa kiểm tra bằng đo đạc**. Thực tế yaw tăng đúng chiều và khớp với bánh xe
  (mục 2), nên trục Z và chiều yaw đang đúng.
- Vị trí lấy từ CAD, chưa hiệu chuẩn.

---

## 9. Package `imu_bno055`: bản cũ, đang không dùng

| File | Tình trạng |
|---|---|
| `src/esp32.c` | Firmware mẫu (Arduino). **Khác firmware đang nạp** (mục 5) |
| `src/bno055_node.cpp` | Node ROS cũ, chỉ publish `/imu/data` |
| `include/imu_bno055/bno055.hpp` | Đọc serial, tách `IMU,millis` + 10 giá trị |
| `launch/bno055.launch.py` | Chạy `bno055_node` với `config/bno055.yaml` |
| `config/bno055.yaml` | Phần lớn tham số **không được node đọc** |

Chi tiết về `bno055_node`:
- Tham số thật chỉ có `port` (`/dev/ttyUSB0`) và `frame_id` (`imu_link`).
- Các trường I2C trong `bno055.yaml` (`device`, `address`, `publish_rate`, `operation_mode`, `calibration`) bị bỏ qua.
- **Khối calibration** trong `bno055.yaml` (22 byte, thanh ghi `0x55`–`0x6A`) **không được nạp vào cảm biến** từ ROS.
  Đừng xóa khối này; nó là dữ liệu hiệu chuẩn đã lưu.
- Nếu dùng với `src/esp32.c` (số thô): gyro sẽ lớn gấp **16 lần** và gia tốc lớn gấp **100 lần** thực tế, vì node coi
  số thô là đã đổi đơn vị.

> **Không chạy `bno055_node` cùng `imu_odom`**: cả hai mở cùng một cổng serial và cùng publish `/imu/data`.

---

## 10. Chạy và kiểm tra

Lệnh dưới **mở cổng serial IMU nhưng không bật motor**:

```bash
ros2 launch stage1_odom ekf.launch.py imu_port:=/dev/ttyUSB0
```

Kiểm tra (chỉ đọc):

```bash
ls -l /dev/serial/by-id/                        # adapter CP2102 = IMU
ros2 topic hz /imu_odom                          # ≈ 50 Hz
ros2 topic echo /imu_odom --once --field twist.twist.angular.z
ros2 topic echo /imu/data --once --field header.frame_id    # imu_link
```

Thử bằng tay:
- Xoay robot **ngược chiều kim đồng hồ** (nhìn từ trên xuống): yaw và `angular.z` phải **tăng / dương**.
- Đứng yên: `angular.z` ≈ 0, yaw không trôi.

Xem dữ liệu thô của ESP32: chỉ làm khi **không** có node nào đang mở cổng, vì mở serial là thao tác phần cứng.

---

## 11. Lỗi hay gặp

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| Log `Cannot open /dev/ttyUSB0` | Sai cổng (USB đổi thứ tự) hoặc thiếu quyền `dialout` |
| Đã kết nối nhưng không có topic | Dòng không đủ 15 trường (firmware khác), hoặc sai baudrate |
| Yaw nhảy khi đến gần motor / sắt | NDOF dùng từ kế; nhiễu từ làm yaw lệch |
| Yaw trôi chậm khi đứng yên | Cảm biến chưa tự hiệu chuẩn xong; xoay robot vài vòng |
| Hai node cùng báo lỗi đọc | Đang chạy cả `bno055_node` và `imu_odom` |

Ghi chú về từ kế:
- Ở chế độ NDOF, yaw bị ảnh hưởng bởi từ trường xung quanh: motor, khung sắt, dây điện lớn.
- Mức nhiễu trên robot **chưa được đo**.
- Firmware hiện không gửi trạng thái hiệu chuẩn (thanh ghi `CALIB_STAT 0x35`) lên máy tính.
