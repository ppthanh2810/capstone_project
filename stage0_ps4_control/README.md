# `stage0_ps4_control`: lái robot bằng tay PS4 và điều khiển motor

Package này là **đường ra motor duy nhất** của robot. Mọi lệnh vận tốc (tay PS4 hoặc bộ điều khiển DWB) đều phải đi
qua đây để tới bánh xe.

File này nói về 4 file chính:
- `config/ps4.yaml`;
- `config/twist_mux.yaml`;
- `launch/move_twist_mux.launch.py`;
- `src/zlac8015d_move.cpp`.

> ⚠️ `move_twist_mux.launch.py` **bật motor ngay khi chạy**. Lần đầu: kê bánh khỏi mặt đất, để E-stop trong tầm tay.

---

## 1. Luồng dữ liệu

```text
Tay PS4 ──USB/BT──> joy_node ──/joy──> joy_teleop ──/cmd_vel_joy──┐  (ưu tiên 100)
                                         (ps4.yaml)               │
                                                                  ├──> twist_mux ──/cmd_vel_out──> zlac8015d_move ──Modbus──> ZLAC8015D ──> 2 bánh
DWB (stage3, chỉ khi navigation_enabled:=true) ──/cmd_vel_nav─────┘  (twist_mux.yaml)              │       (/dev/ttyUSB0)
                                                   (ưu tiên 10)                                    │
                                                                                                   └──/wheel_feedback──> zlac8015d_odom (stage1)
```

| Bước | Node | Vào | Ra |
|---|---|---|---|
| 1 | `joy_node` | tay cầm | `/joy` |
| 2 | `joy_teleop` | `/joy` | `/cmd_vel_joy` |
| 3 | `twist_mux` | `/cmd_vel_joy`, `/cmd_vel_nav` | `/cmd_vel_out` |
| 4 | `zlac8015d_move` | `/cmd_vel_out` | motor + `/wheel_feedback` |

---

## 2. `launch/move_twist_mux.launch.py`

**Vai trò:** chạy cả 4 node trên. Đây là launch teleop chính.

```bash
ros2 launch stage0_ps4_control move_twist_mux.launch.py
```

| Node | Package | Tham số |
|---|---|---|
| `joy_node` | `joy` | mặc định của `joy` |
| `joy_teleop` | `joy_teleop` | `config/ps4.yaml` |
| `twist_mux` | `twist_mux` | `config/twist_mux.yaml` |
| `zlac8015d_move` | `stage0_ps4_control` | mặc định trong code |

Ghi chú:
- Launch **không có đối số** nào. Cổng motor viết cứng trong `zlac8015d_move.cpp`.
- `m_to_m` và `move` là bản cũ, đang bị comment trong launch. Chúng chỉ chạy trong `move_ps4.launch.py` (cũ, không dùng).
- Các launch lớn đều include file này, nên **cũng bật motor**:
  - `stage1_odom/wheel_odom_view.launch.py`;
  - `stage2_slam/mapping.launch.py`, `localization.launch.py`;
  - `stage3_navigation/planning.launch.py`, `navigation.launch.py`.

---

## 3. `config/ps4.yaml` (cho `joy_teleop`)

**Vai trò:** đổi cần gạt PS4 thành lệnh vận tốc `geometry_msgs/Twist` trên `/cmd_vel_joy`.

```yaml
joy_teleop:
  ros__parameters:
    move:
      type: topic
      interface_type: geometry_msgs/msg/Twist
      topic_name: /cmd_vel_joy
      deadman_buttons: [1]
      axis_mappings:
        linear-x:  { axis: 1, scale: 0.2, offset: 0.0 }
        angular-z: { axis: 3, scale: 0.4, offset: 0.0 }
```

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `deadman_buttons` | `[1]` | Phải **giữ nút 1** thì mới có lệnh |
| `linear-x.axis` | `1` | Cần trái, lên / xuống |
| `linear-x.scale` | `0.2` | Tiến / lùi tối đa **0.2 m/s** |
| `angular-z.axis` | `3` | Cần phải, trái / phải |
| `angular-z.scale` | `0.4` | Xoay tối đa **0.4 rad/s** |

Ghi chú:
- Lệnh = giá trị cần (−1 … 1) × `scale`. Đẩy cần trái lên hết thì `linear.x = 0.2`.
- Số thứ tự nút và trục phụ thuộc driver tay cầm. Kiểm tra bằng `ros2 topic echo /joy` (bấm nút / gạt cần xem số nào
  đổi). Với tay PS4, nút 1 thường là **nút tròn (○)**.
- **Thả nút deadman thì `joy_teleop` ngừng gửi lệnh.** Robot dừng nhờ timeout của `twist_mux` và watchdog của
  `zlac8015d_move` (mục 5).
- Tay PS4 **lùi được**: `linear.x` âm. Phía sau robot không có LiDAR quan sát, nên khi lùi phải tự nhìn.

---

## 4. `config/twist_mux.yaml` (cho `twist_mux`)

**Vai trò:** chọn **một** nguồn lệnh khi có nhiều nguồn cùng lúc. Nguồn có `priority` cao hơn thắng.

```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation: { topic: /cmd_vel_nav, timeout: 0.5, priority: 10 }
      joystick:   { topic: /cmd_vel_joy, timeout: 0.5, priority: 100 }
```

| Nguồn | Topic | Ưu tiên | Timeout |
|---|---|---|---|
| Tay PS4 | `/cmd_vel_joy` | **100** | 0.5 s |
| Điều hướng (DWB) | `/cmd_vel_nav` | 10 | 0.5 s |

- Đầu ra: `/cmd_vel_out` (tên mặc định của `twist_mux`, không remap).
- **Tay PS4 luôn thắng DWB.** Đang giữ deadman thì lệnh của DWB bị bỏ qua.
- Một nguồn **quá 0.5 s không gửi** thì bị coi là hết hiệu lực; nguồn ưu tiên thấp hơn được dùng.
- Thả tay PS4 thì sau 0.5 s, DWB (nếu đang chạy) lại được điều khiển robot. **PS4 không phải nút dừng khẩn cấp.**
- Không cấu hình `locks` (khóa dừng). Muốn dừng điều hướng: `/goal_path_bridge/cancel`, Ctrl+C, hoặc E-stop phần cứng.
- DWB chỉ gửi vào `/cmd_vel_nav` khi chạy `navigation.launch.py navigation_enabled:=true`. Mặc định nó gửi vào
  `/cmd_vel_dwb`, nên không tới motor.

---

## 5. `src/zlac8015d_move.cpp`

**Vai trò:** nhận lệnh `/cmd_vel_out`, tính tốc độ từng bánh, gửi xuống bộ điều khiển ZLAC8015D qua Modbus. Đồng
thời đọc phản hồi bánh xe cho odometry. Dùng thư viện `hardware/zlac8015d_hw`.

### Tham số

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `wheel_radius` | 0.0535 m | Bán kính bánh |
| `wheel_distance` | 0.34 m | Khoảng cách 2 bánh |
| `accel_time_ms` | 500 ms | Thời gian tăng tốc của driver |
| `decel_time_ms` | 500 ms | Thời gian giảm tốc của driver |

Hằng số viết trong code (không đổi được bằng tham số):

| Hằng số | Giá trị |
|---|---|
| Cổng Modbus | `/dev/ttyUSB0` |
| Baudrate / slave | 115200 / 1 (mặc định thư viện) |
| Timeout lệnh (watchdog) | 0.5 s |
| Chu kỳ kiểm tra watchdog | 100 ms |
| Chu kỳ đọc phản hồi | 50 ms (20 Hz) |

Ghi chú về tham số:
- `accel_time_ms` và `decel_time_ms` phải trong [50, 2000] ms, nếu không node báo lỗi và dừng.
- Trước 2026-10-14 thời gian tăng/giảm tốc cố định 500 ms. Đổi còn 200 ms để giảm trễ khi DWB điều khiển. Mức giật
  và trượt bánh với 200 ms **chưa kiểm tra trên robot**.
- `wheel_radius` và `wheel_distance` phải khớp với `stage1_odom` và URDF.

### Khởi động (ghi xuống driver)

1. `disableMotor`.
2. Đặt thời gian tăng/giảm tốc (200 ms, cả 2 bánh).
3. `setMode(3)`: chế độ điều khiển vận tốc.
4. `setRpm(0, 0)`.
5. `enableMotor`.

### Mỗi lệnh `/cmd_vel_out`

Chỉ dùng `linear.x` (v) và `angular.z` (ω):

```text
bánh trái  = (v − ω · 0.34/2) / 0.0535      [rad/s]
bánh phải  = (v + ω · 0.34/2) / 0.0535      [rad/s]
RPM        = rad/s × 30/π, làm tròn số nguyên
RPM phải   = đổi dấu (motor phải lắp ngược)
```

Ví dụ với cấu hình PS4 hiện tại:

| Lệnh | RPM mỗi bánh |
|---|---|
| Tiến 0.2 m/s | ≈ 35.7 |
| Xoay 0.4 rad/s tại chỗ | ≈ 12.1 (hai bánh ngược chiều) |
| Cả hai tối đa | ≈ 47.8 (bánh ngoài) |

- RPM làm tròn số nguyên, nên mỗi bánh chỉnh được theo bước khoảng 0.0056 m/s.
- Ghi lệnh lỗi (Modbus) thì chỉ cảnh báo `Set RPM failed`, không dừng node.
- Hàng đợi subscription là 1: chỉ giữ lệnh mới nhất.

### Watchdog (an toàn)

- Sau lệnh đầu tiên, nếu **quá 0.5 s không có lệnh mới** thì đặt RPM = 0, log `Command timeout. Motors stopped.`
- Đây là lớp bảo vệ cuối khi PS4 mất kết nối, `twist_mux` ngừng gửi, hoặc DWB bị treo.

### Phản hồi `/wheel_feedback` (20 Hz)

Kiểu `std_msgs/Float64MultiArray`, gồm 4 số:

| Chỉ số | Ý nghĩa | Đơn vị |
|---|---|---|
| 0 | RPM bánh trái | RPM |
| 1 | RPM bánh phải (đã đổi dấu) | RPM |
| 2 | Quãng đường bánh trái | m |
| 3 | Quãng đường bánh phải (đã đổi dấu) | m |

- Quãng đường tính từ encoder: số xung / 4096 × 0.336 m.
- `stage1_odom/zlac8015d_odom` đọc topic này để tính `/wheel_odom`.
- Dữ liệu không hợp lệ (NaN) thì bỏ, cảnh báo `Invalid motor feedback`.

### Khi tắt node

Destructor đặt RPM = 0 rồi `disableMotor`. Nếu tắt kiểu đột ngột (rút nguồn máy tính, `kill -9`), bước này không
chạy; khi đó chỉ còn E-stop.

---

## 6. Kiểm tra nhanh (chỉ đọc, khi launch đang chạy)

```bash
ros2 topic echo /joy                     # xem số nút / trục khi bấm
ros2 topic echo /cmd_vel_joy             # chỉ có khi giữ deadman
ros2 topic echo /cmd_vel_out             # lệnh thật tới motor
ros2 topic hz /wheel_feedback            # ≈ 20 Hz
```

Lần đầu thử:
- Kê bánh khỏi mặt đất.
- Đẩy cần trái lên: cả 2 bánh phải quay **về phía trước**.
- Gạt cần phải sang trái: robot phải quay **ngược chiều kim đồng hồ** (nhìn từ trên).

## 7. Lỗi hay gặp

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| Gạt cần không có gì xảy ra | Chưa giữ deadman, hoặc sai số nút |
| Node motor báo lỗi khi khởi động | Sai cổng (USB đổi thứ tự), thiếu quyền, driver chưa cấp nguồn |
| `Command timeout. Motors stopped.` lặp lại | Bình thường khi thả tay; nếu xảy ra khi đang giữ thì tay cầm mất kết nối |
| Robot quay ngược chiều mong muốn | Sai dấu trục (`scale` âm để đảo) |
| DWB không lái được | Đang giữ deadman PS4 (ưu tiên 100), hoặc `navigation_enabled:=false` |
