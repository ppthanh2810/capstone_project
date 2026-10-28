# Capstone AMR: robot tự hành vi sai (ROS 2 Humble)

Robot tròn Ø400 mm, 2 bánh chủ động (ZLAC8015D) + 2 bánh tự lựa, LiDAR RPLIDAR C1, IMU BNO055 (qua ESP32),
camera RealSense D435. Chạy trên **phần cứng thật**, không có mô phỏng Gazebo.

Dự án làm theo từng giai đoạn (stage). Mỗi package có README riêng với phân tích chi tiết; file này là phần tổng quát.

| Stage | Package | Làm gì |
|---|---|---|
| 0 | `stage0_ps4_control` | Lái tay PS4, điều khiển motor |
| 1 | `stage1_odom` | Odometry bánh xe + IMU + EKF |
| 2 | `stage2_slam` | LiDAR, vẽ bản đồ (SLAM), định vị (AMCL) |
| 3 | `stage3_navigation` | Lập đường A\* + CustomDWB bám đường, tự né vật cản |

Các thư mục khác:
- `hardware/`: driver motor, IMU, LiDAR, mô hình robot (URDF). Xem `hardware/README.md`.
- `maps/`: bản đồ đã lưu (`map.yaml` + `map.pgm`).
- `da1/`: dự án cũ (YOLO + teleop Python), không dùng trong stage 0–3.

---

## 1. Luồng dữ liệu

```text
PS4 → joy_teleop → /cmd_vel_joy ─┐
CustomDWB (stage3) → /cmd_vel_nav ┴→ twist_mux → /cmd_vel_out → zlac8015d_move ──Modbus──> ZLAC8015D
                                                                    └→ /wheel_feedback → zlac8015d_odom → /wheel_odom ┐
ESP32 + BNO055 → imu_odom → /imu_odom ─────────────────────────────────────────────────────────────────────────┴→ EKF → /odometry/filtered + TF odom→base_footprint
RPLIDAR C1 → /scan_raw → scan_rotate → /scan → laser_filters → /scan_filtered → slam_toolbox hoặc AMCL (TF map→odom)
RViz 2D Goal Pose → goal_path_bridge → A* → /global_path → controller_server (CustomDWB + local_costmap) → /cmd_vel_nav
```

- PS4 luôn ưu tiên hơn navigation trong `twist_mux` (100 > 10).
- Mặc định (`navigation_enabled:=false`) lệnh của DWB ra `/cmd_vel_dwb`, **không tới motor**.

---

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/capstone_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. Chạy

> ⚠️ Mọi launch dưới đây (trừ xem URDF) **mở phần cứng và bật motor**. Kê bánh khỏi mặt đất / có E-stop khi thử lần đầu.

| Mục đích | Lệnh |
|---|---|
| Lái tay | `ros2 launch stage0_ps4_control move_twist_mux.launch.py` |
| Vẽ bản đồ | `ros2 launch stage2_slam mapping.launch.py` |
| Định vị | `ros2 launch stage2_slam localization.launch.py` |
| Chỉ lập đường A\* | `ros2 launch stage3_navigation planning.launch.py` |
| A\* + DWB (cách ly motor) | `ros2 launch stage3_navigation navigation.launch.py` |
| Xem URDF (không phần cứng) | `ros2 launch robot_description display.launch.py` |

Cổng USB (có thể đổi khi cắm lại, kiểm tra `ls -l /dev/serial/by-id/`):

| Thiết bị | Cổng |
|---|---|
| Motor ZLAC8015D | `/dev/ttyUSB1` |
| IMU (ESP32) | `/dev/ttyUSB0` |
| LiDAR C1 | `/dev/ttyUSB2` |

---

## 4. Tổng thời gian truyền và phản hồi

Tổng thời gian ước lượng như bảng dưới, đã đổi sang giây và cộng delay. Giả sử hệ thống chạy bình thường, Modbus
không lỗi và driver đang đặt tăng/giảm tốc 0,5 s. Đây là **ngân sách ước lượng**, chưa phải thời gian tối đa đã đo
trên robot. Chi tiết từng phần nằm ở mục *Thời gian truyền và phản hồi* trong README của từng stage:
- stage0 mục 5 (lệnh → motor, feedback);
- stage1 mục 7 (odometry, IMU, EKF);
- stage2 mục 8 (LiDAR, AMCL, CSM, monitor);
- stage3 mục 11 (A\*, vòng điều khiển, vật cản, dừng).

### 4.1 Lệnh, feedback và odometry

| Trường hợp | Tổng | Tính đến thời điểm |
|---|---|---|
| Lệnh đã có trên `/cmd_vel_out` → driver nhận | ≈ 0,05 s | Đã tính chờ callback và ghi Modbus |
| Gạt PS4 → driver nhận lệnh | ≈ 0,10 s | Giả sử tay cầm cập nhật 20 Hz |
| Gạt PS4 → nhận feedback bánh sau khi hoàn tất ramp | ≈ 0,70 s | Gửi lệnh + ramp 0,5 s + đọc feedback |
| Đến lượt controller xử lý → nhận feedback bánh sau ramp | ≈ 0,74 s | Tính cả chờ chu kỳ controller 0,1 s |
| Bánh thay đổi tốc độ → `/wheel_feedback` có mẫu phản ánh thay đổi | ≈ 0,08 s | Chờ polling + đọc RPM và encoder |
| Bánh thay đổi tốc độ → `/odometry/filtered` bắt đầu phản ánh | ≈ 0,12 s | Chưa tính thời gian bộ lọc hội tụ |
| Bánh thay đổi tốc độ → vận tốc odom đạt khoảng 90 % | ≈ 0,64 s | Đã tính bộ lọc vận tốc và chu kỳ EKF |
| Gạt PS4 → vận tốc odom gần ổn định | ≈ 1,20–1,25 s | Cộng bảo thủ cả ramp và bộ lọc |
| Chờ controller → vận tốc odom gần ổn định | ≈ 1,30 s | Cộng bảo thủ cả chu kỳ điều khiển, ramp và lọc |
| Robot quay → yaw được phản ánh qua IMU và EKF | ≈ 0,08 s | Không đi qua bộ lọc vận tốc bánh |

Ví dụ cách cộng trường hợp PS4:

$$
T_{\text{PS4 → feedback}} \approx 0{,}10 + 0{,}50 + 0{,}082 = 0{,}682\ \text{s} \approx \mathbf{0{,}70\ s}
$$

Ramp là quá trình thay đổi tốc độ dần, nên robot thường bắt đầu phản ứng **trước** các mốc "hoàn tất ramp / gần ổn
định".

### 4.2 Vật cản và các tình huống dừng

| Trường hợp | Tổng | Mốc bắt đầu → kết thúc |
|---|---|---|
| LiDAR thấy vật cản → cập nhật local costmap | ≈ 0,30 s | Vật xuất hiện → costmap có vật |
| LiDAR thấy vật cản → bắt đầu giảm tốc | ≈ 0,45 s | Giả sử controller quyết định phanh ngay |
| LiDAR thấy vật cản → hoàn tất giảm tốc về 0 | ≈ 0,95 s | Đã cộng ramp giảm tốc 0,5 s |
| Chỉ camera thấy vật cản → costmap xác nhận | ≈ 0,35–0,50 s | Cần đủ 3 frame và cập nhật costmap |
| Chỉ camera thấy vật cản → hoàn tất dừng | ≈ 1,00–1,20 s | Giả sử controller chọn phanh sau xác nhận |
| Gọi cancel FollowPath → hoàn tất dừng | ≈ 0,65 s | Xử lý hủy + truyền lệnh 0 + ramp |
| Mất toàn bộ lệnh → watchdog cho motor dừng | ≈ 1,05–1,15 s | Từ lệnh cuối: timeout + kiểm tra + Modbus + ramp |
| Bridge nhận trạng thái LOST → hoàn tất dừng | ≈ 0,85 s | Chưa tính thời gian monitor phát hiện lỗi |
| Consistency liên tục thấp → phát hiện LOST và dừng | ≈ 2,85–3,35 s | Có tính 5 lần kiểm tra monitor ở 2 Hz |
| Bridge nhận UNCERTAIN liên tục → hoàn tất dừng | ≈ 2,85 s | Có tính grace 2 s |

Hai ngoại lệ cần nhớ:
- **Thả PS4 khi navigation vẫn chạy**: sau khoảng 0,5 s, quyền điều khiển có thể trở lại navigation; robot không
  nhất thiết dừng.
- **Modbus lỗi, USB nghẽn hoặc callback bị kẹt**: các mốc trên không còn áp dụng. Mã hiện tại chưa cho phép khẳng
  định một giới hạn tổng chắc chắn.

### 4.3 A\* và AMCL

Với A\* và AMCL, chưa có một tổng thời gian cố định:
- A\* chạy theo yêu cầu goal (log robot: trung vị 2,3 ms, tối đa 8,8 ms).
- AMCL còn phụ thuộc scan, mức di chuyển (chỉ cập nhật sau 0,05 m / 0,05 rad) và tải xử lý.

Không nên dùng chu kỳ 1 Hz hoặc scan 10 Hz làm thời gian hoàn thành của chúng.

---

## 5. Giải thích thuật toán CustomDWB

Phân tích đầy đủ (tham số, sơ đồ, kết quả mô phỏng): `stage3_navigation/README.md` mục 6.4, 8 và 9. Các ghi chú
"**Trong code**" bên dưới là chỗ code (`src/dwa_core.cpp`, `config/nav2_params.yaml`) làm khác hoặc chi tiết hơn
phần giải thích đơn giản.

### 5.1 Ý tưởng tổng quát

CustomDWB là bộ điều khiển cục bộ được triển khai trong `controller_server` của ROS 2. Bộ điều khiển hoạt động ở tần
số khoảng **10 Hz**, nghĩa là cứ mỗi 0,1 s robot lại nhận dữ liệu mới, tạo các lệnh vận tốc ứng viên và chọn ra lệnh
phù hợp nhất.

Bốn công thức chính trả lời bốn câu hỏi khác nhau:
1. Robot phải quay nhanh bao nhiêu?
2. Trong chu kỳ này robot có thể đạt những vận tốc nào?
3. Mỗi vận tốc sẽ đưa robot đi đến đâu?
4. Trong các quỹ đạo an toàn, quỹ đạo nào tốt nhất?

Quy trình tổng quát:

```text
Dữ liệu cảm biến + TF + odometry + global path
                    ↓
              PathTracker
                    ↓
       Chọn pha điều khiển
       ├── ROTATE_TO_GOAL
       ├── ROTATE_TO_PATH
       └── FOLLOW_PATH
                    ↓
       Tạo và mô phỏng quỹ đạo
                    ↓
        Loại quỹ đạo không an toàn
                    ↓
       Chấm điểm và chọn quỹ đạo
                    ↓
              /cmd_vel_nav
```

### 5.2 Dữ liệu đầu vào

CustomDWB nhận ba nhóm dữ liệu chính:
- **Pose, TF và odometry:** vị trí, hướng, vận tốc dài $v$ và vận tốc góc $\omega$ của robot.
- **Local costmap:** vị trí vật cản, vùng inflation và các ô chưa biết quanh robot.
- **Global path A\*:** đường đi tổng thể do `planner_server` tạo ra.

Các dữ liệu này được sử dụng trong mỗi chu kỳ điều khiển để quyết định lệnh vận tốc tiếp theo.

> **Trong code:** pose cũ hơn 0,5 s (`max_pose_age`) hoặc `/odometry/filtered` im lặng quá 0,5 s (`odom_timeout`)
> → không tính lệnh, `controller_server` phát lệnh 0.

### 5.3 PathTracker

Đường toàn cục được biến đổi về hệ tọa độ cục bộ của robot. Bộ điều khiển không sử dụng toàn bộ đường A\* trong một
lần, mà chỉ lấy phần đường phía trước robot:
- Độ dài local path: khoảng **1,5 m** (`local_plan_length`, còn bị cắt ở mép local costmap).
- Khoảng cách carrot point: khoảng **0,8 m** (`carrot_distance`).

Carrot point là điểm nhìn trước trên đường. Robot cố gắng tiến về điểm này thay vì chỉ bám vào vị trí gần nhất trên
đường. Cách này giúp robot có xu hướng tiến về phía trước và giảm tình trạng đứng yên hoặc rung tại một điểm.

> **Trong code:** điểm gần nhất được tìm trong 1,0 m phía trước (`closest_search_distance`), tiến trình không bao
> giờ lùi lại (không nhảy nhánh ở đường chữ U). Lệch đường > 1,0 m (`max_path_deviation`) → từ chối, FollowPath abort.

### 5.4 Chọn pha điều khiển

CustomDWB không phải lúc nào cũng chạy Dynamic Window. Bộ điều khiển lựa chọn một trong ba pha:

- **`ROTATE_TO_GOAL`**: robot đã ở gần đích nhưng hướng cuối chưa đúng. Robot dừng tiến và chỉ xoay để đạt sai số góc
  nhỏ hơn ngưỡng cho phép.
- **`ROTATE_TO_PATH`**: robot chưa hướng đúng theo đoạn đường phía trước hoặc carrot point. Robot quay tại chỗ để căn
  hướng trước khi bắt đầu chạy tiến.
- **`FOLLOW_PATH`**: robot đã có hướng tương đối phù hợp với đường. Bộ điều khiển bắt đầu tạo các cặp vận tốc
  $(v,\omega)$, mô phỏng quỹ đạo và chọn lệnh tốt nhất bằng hàm chi phí.

Việc tách ba pha giúp robot không vừa quay mạnh vừa chạy tiến, từ đó giảm nguy cơ lệch đường hoặc va chạm khi đổi
hướng.

> **Trong code:**
> - `ROTATE_TO_GOAL`: vào khi cách đích < 0,9 × 0,15 m, dừng xoay khi lệch < 0,9 × 0,25 rad.
> - `ROTATE_TO_PATH`: chỉ vào khi lệch hướng tới carrot > **2,5 rad** (~143°, robot gần như quay lưng), thoát khi
>   < 1,25 rad. Ngưỡng cao này có chủ ý (tuning 2026-10-21): với 0,8 rad robot quay ngược vào vật cản giữa lúc né.
> - Vì vậy phần lớn các khúc cua (kể cả 135°) được chạy thành **cung** trong `FOLLOW_PATH`, tức robot vẫn vừa quay
>   vừa tiến; an toàn do lớp kiểm tra cứng (mục 5.8) đảm bảo.

### 5.5 Công thức 1 — Lệnh quay tại chỗ

Khi robot ở pha `ROTATE_TO_GOAL` hoặc `ROTATE_TO_PATH`, vận tốc dài được đặt bằng không:

$$
v^* = 0
$$

Vận tốc góc được tính bởi:

$$
\omega^* = \operatorname{sign}(e_\theta)\,\min\left(k|e_\theta|,\ \sqrt{2\alpha|e_\theta|},\ \omega_{\max}\right)
$$

Ý nghĩa các thành phần:
- $e_\theta = \theta_{target} - \theta_{robot}$: sai số góc, chuẩn hóa về $[-\pi, \pi]$.
- $\operatorname{sign}(e_\theta)$: xác định robot phải quay trái hay quay phải.
- $k|e_\theta|$: thành phần tỉ lệ. Sai số càng lớn thì robot quay càng nhanh. Cấu hình hiện tại $k = 1{,}5$
  (`rotate_gain`).
- $\sqrt{2\alpha|e_\theta|}$: giới hạn vận tốc để robot còn đủ khả năng giảm tốc trước khi đến góc mục tiêu.
- $\alpha$: giới hạn giảm tốc góc, $2{,}0\ \mathrm{rad/s^2}$ (`decel_lim_theta`).
- $\omega_{\max}$: vận tốc góc lớn nhất, $1{,}0\ \mathrm{rad/s}$ (`max_vel_theta`).

Bộ điều khiển lấy giá trị nhỏ nhất trong ba giới hạn. Vì vậy robot quay nhanh khi còn sai số lớn và tự giảm tốc khi
gần đúng hướng.

**Ví dụ.** Giả sử $e_\theta = 0{,}4\ \mathrm{rad}$, $k = 1{,}5$, $\alpha = 2$, $\omega_{\max} = 1$:

$$
k|e_\theta| = 1{,}5 \times 0{,}4 = 0{,}6 \qquad
\sqrt{2\alpha|e_\theta|} = \sqrt{2 \times 2 \times 0{,}4} \approx 1{,}265 \qquad
\omega_{\max} = 1
$$

$$
\omega^* = \min(0{,}6;\ 1{,}265;\ 1) = 0{,}6\ \mathrm{rad/s}
$$

Khi chỉ còn $e_\theta = 0{,}05\ \mathrm{rad}$:

$$
\omega^* = \min(0{,}075;\ 0{,}447;\ 1) = 0{,}075\ \mathrm{rad/s}
$$

Robot vì vậy giảm tốc khi gần đúng hướng và hạn chế quay vượt góc.

> **Trong code** (`DwaCore::rotateCommand`):
> - $|e_\theta|$ được thay bằng **góc còn lại** = $|e_\theta|$ − phần đang quay "trong đường ống"
>   ($\omega_{ref} \times (0{,}2 + 0{,}05)$ s, do trễ `control_latency`). Còn lại ≤ 0 → lệnh hướng về 0.
> - Có sàn `min_in_place_vel_theta` 0,1 rad/s (thắng ma sát), nên ở ví dụ thứ hai lệnh là **0,1 rad/s**, không
>   phải 0,075.
> - Lệnh không nhảy thẳng tới $\omega^*$ mà chỉ đổi tối đa $\alpha\Delta t = 0{,}2$ rad/s mỗi chu kỳ, rồi vẫn qua
>   kiểm tra va chạm như mọi quỹ đạo khác.

### 5.6 Công thức 2 — Cửa sổ vận tốc động

Robot không thể thay đổi vận tốc tức thời. Vì vậy CustomDWB chỉ thử các vận tốc mà robot có thể đạt được trong chu
kỳ tiếp theo.

Đối với vận tốc dài:

$$
v \in \left[\max(v_{\min},\ v_{ref} - a_x\Delta t),\ \min(v_{\max},\ v_{ref} + a_x\Delta t)\right]
$$

Đối với vận tốc góc:

$$
\omega \in \left[\max(-\omega_{\max},\ \omega_{ref} - \alpha\Delta t),\ \min(\omega_{\max},\ \omega_{ref} + \alpha\Delta t)\right]
$$

Ý nghĩa các thành phần:
- $v_{ref}, \omega_{ref}$: vận tốc tham chiếu tại đầu chu kỳ, dựa trên lệnh trước đó và được đối chiếu với vận tốc đo.
- $a_x$: giới hạn gia tốc dài, $0{,}5\ \mathrm{m/s^2}$ (`acc_lim_x`, `decel_lim_x`).
- $\alpha$: giới hạn gia tốc góc, $2{,}0\ \mathrm{rad/s^2}$.
- $\Delta t$: thời gian một chu kỳ điều khiển; ở 10 Hz thì $\Delta t = 0{,}1$ s.
- $v_{\min}, v_{\max}$: giới hạn vận tốc dài, $0 \rightarrow 0{,}3\ \mathrm{m/s}$ (không lùi).
- $\omega_{\max}$: giới hạn vận tốc góc, $1{,}0\ \mathrm{rad/s}$.

**Ví dụ.** Giả sử $v_{ref} = 0{,}2\ \mathrm{m/s}$, $a_x = 0{,}5\ \mathrm{m/s^2}$, $\Delta t = 0{,}1$ s. Trong một chu
kỳ, robot chỉ thay đổi được:

$$
\Delta v = a_x\Delta t = 0{,}5 \times 0{,}1 = 0{,}05\ \mathrm{m/s}
\quad\Rightarrow\quad
v \in [0{,}15;\ 0{,}25]\ \mathrm{m/s}
$$

DWB lấy 20 mẫu vận tốc dài trong khoảng này. Với 21 mẫu vận tốc góc, tổng số cặp vận tốc kiểm tra xấp xỉ:

$$
20 \times 21 = 420 \text{ cặp } (v, \omega)
$$

Cửa sổ vận tốc giúp lệnh điều khiển liên tục hơn và phù hợp với giới hạn động học của robot.

> **Trong code:**
> - $v_{ref}$ = lệnh trước, kẹp trong ± 0,15 m/s / ± 0,5 rad/s quanh vận tốc đo (`velocity_reference: command`).
>   Dùng thẳng vận tốc đo làm gốc thì odometry trễ ~0,2 s làm robot quay lố ~50° (robot 2026-10-14).
> - Luôn thêm mẫu $v = 0$ và $\omega = 0$, rồi bỏ mẫu trùng, nên số cặp thực tế ~420–440.
> - Cửa sổ $\omega$ ở đây chỉ rộng ± 0,2 rad/s quanh $\omega_{ref}$.

### 5.7 Công thức 3 — Mô phỏng quỹ đạo

Mỗi cặp vận tốc $(v, \omega)$ được mô phỏng bằng mô hình robot vi sai:

$$
\dot{x} = v\cos\theta, \qquad \dot{y} = v\sin\theta, \qquad \dot{\theta} = \omega
$$

- $x, y$: vị trí dự đoán của robot.
- $\theta$: hướng của robot.
- $v, \omega$: vận tốc của quỹ đạo ứng viên.

Bộ điều khiển tích phân các phương trình này theo từng bước thời gian để dự đoán quỹ đạo trong $T_{sim} = 2{,}0$ s
(`sim_time`).

**Ví dụ đi thẳng.** Với $v = 0{,}2\ \mathrm{m/s}$, $\omega = 0$, sau $T = 2$ s robot đi được
$s = vT = 0{,}2 \times 2 = 0{,}4$ m. Quỹ đạo là một đoạn thẳng dài khoảng 0,4 m.

**Ví dụ đi cong.** Với $v = 0{,}2\ \mathrm{m/s}$, $\omega = 0{,}5\ \mathrm{rad/s}$:

$$
R = \frac{v}{\omega} = \frac{0{,}2}{0{,}5} = 0{,}4\ \mathrm{m}, \qquad
\Delta\theta = \omega T = 0{,}5 \times 2 = 1\ \mathrm{rad}
$$

Nếu robot bắt đầu tại $(0, 0, 0)$:

$$
x = R\sin(\Delta\theta) = 0{,}4\sin(1) \approx 0{,}337\ \mathrm{m}, \qquad
y = R[1 - \cos(\Delta\theta)] \approx 0{,}184\ \mathrm{m}
$$

Costmap được kiểm tra dọc theo đường cong này. Nếu quỹ đạo chạm vùng vật cản thì bị loại.

> **Trong code:** mô phỏng bắt đầu từ **vận tốc đo**, không phải ngay từ $(v, \omega)$ ứng viên:
> 1. 0,2 s đầu (`control_latency`): lệnh trước vẫn đang tác dụng;
> 2. một chu kỳ chuyển tiếp sang $(v, \omega)$ theo giới hạn gia tốc;
> 3. giữ $(v, \omega)$ tới hết 2,0 s (cung tròn chính xác, mỗi bước ≤ nửa ô costmap);
> 4. thêm **đuôi phanh**: từ sau latency + 1 chu kỳ, giảm tốc về 0 theo `decel_lim` + 0,02 m dự phòng.

### 5.8 Kiểm tra an toàn

Trước khi chấm điểm, quỹ đạo ứng viên bị loại ngay nếu:
- Va chạm vật cản.
- Đi ra ngoài local costmap.
- Đi vào vùng chưa biết (`unknown`).
- Không còn đủ khoảng cách để robot phanh an toàn.

Đây là lớp bảo vệ cứng. Một quỹ đạo đã bị loại sẽ không thể được chọn, cho dù nó có bám đường tốt hoặc chạy nhanh.

> **Trong code:** "va chạm" = một pose bất kỳ (kể cả pose đầu và các pose của đuôi phanh) rơi vào ô có giá trị
> ≥ 253 (INSCRIBED: tâm robot cách vật < 0,25 m). Chỉ kiểm tra ô tâm robot, đúng vì thân tròn.
> - Không còn quỹ đạo nào hợp lệ → lỗi, `controller_server` phát lệnh 0, sau 0,3 s abort.
> - Còn quỹ đạo hợp lệ nhưng không có quỹ đạo nào với $v > 0$ → $v = 0$, log `holding position`.
> - Phía sau robot là FREE (LiDAR không thấy), nên robot **không bao giờ lùi**.

### 5.9 Công thức 4 — Hàm chấm điểm quỹ đạo

Sau khi loại các quỹ đạo không an toàn, các quỹ đạo còn lại được đánh giá bằng hàm chi phí:

$$
\begin{aligned}
J ={}& 0{,}5\,d_{path} + 2\,d_{carrot} + 0{,}3\,\frac{|e_{heading}|}{\pi} \\
&+ 4\,\frac{c_{obs}}{252} + 1{,}5\,\frac{v_{\max} - v}{v_{\max}} + I_{osc}
\end{aligned}
$$

Robot chọn quỹ đạo:

$$
(v^*, \omega^*) = \underset{(v,\omega)}{\operatorname{argmin}}\ J(v, \omega)
$$

Nói cách khác, quỹ đạo có $J$ nhỏ nhất sẽ được lựa chọn.

Ý nghĩa từng thành phần:

- **$0{,}5\,d_{path}$ — bám đường** (`path_distance_scale`)
  - $d_{path}$ là khoảng cách từ quỹ đạo đến đường cục bộ / global path.
  - Khuyến khích robot đi gần đường.
  - Trọng số 0,5 tương đối thấp, nên robot được phép lệch khỏi đường khi cần tránh vật cản.
- **$2\,d_{carrot}$ — tiến về phía trước** (`carrot_distance_scale`)
  - $d_{carrot}$ là khoảng cách từ cuối quỹ đạo đến carrot point.
  - Trọng số 2 giúp robot không chỉ bám vào điểm gần nhất mà còn tiến về phía trước trên đường.
  - Góp phần giữ robot trên hướng chuyển động của lane.
- **$0{,}3\,|e_{heading}|/\pi$ — sai lệch hướng** (`heading_scale`)
  - $e_{heading}$ là sai lệch giữa hướng cuối quỹ đạo và hướng mong muốn.
  - Chia cho $\pi$ để chuẩn hóa về khoảng $0 \rightarrow 1$.
  - Giúp robot không chạy ngang hoặc quay lệch so với đường.
- **$4\,c_{obs}/252$ — mức nguy hiểm của vật cản** (`obstacle_scale`)
  - $c_{obs}$ là chi phí vật cản lấy từ costmap (0 = trống, 1–252 = vùng inflation).
  - Chia cho 252 để chuẩn hóa về $0 \rightarrow 1$.
  - Trọng số 4 là lớn nhất trong hàm điểm, nên quỹ đạo gần vật cản bị phạt mạnh.
- **$1{,}5\,(v_{\max} - v)/v_{\max}$ — ưu tiên vận tốc** (`speed_scale`)
  - Phạt các quỹ đạo có vận tốc thấp.
  - Nếu hai quỹ đạo đều an toàn và bám đường tương tự nhau, robot chọn quỹ đạo nhanh hơn.
  - Không được phép vượt qua lớp kiểm tra an toàn cứng.
- **$I_{osc}$ — phạt dao động** (`oscillation_scale` 1,0)
  - $I_{osc} = 1$ nếu quỹ đạo gây đổi chiều quay trái–phải không mong muốn, $= 0$ nếu không.
  - Giúp robot không liên tục chuyển giữa hai hướng điều khiển.

> **Trong code:**
> - $d_{path}$, $d_{carrot}$ đo từ **điểm cuối** quỹ đạo (m); $e_{heading}$ = lệch giữa hướng cuối và hướng **tới
>   carrot**.
> - $c_{obs}$ = chi phí **lớn nhất** dọc một cung dài cố định $v_{\max} \cdot T_{sim} = 0{,}6$ m theo cùng độ cong,
>   không phụ thuộc tốc độ (nếu không, quỹ đạo chậm sẽ "nhìn" ngắn hơn và luôn được điểm vật cản thấp).
>   Ô va chạm / unknown trên cung này tính 252, không loại quỹ đạo.
> - Quỹ đạo $v = 0$ **không có** chi phí vật cản; vì vậy `speed_scale` phải đủ lớn (1,5) để robot không chọn đứng
>   yên mỗi khi gần vật cản.
> - $I_{osc}$: phạt khi $\omega$ quay lại chiều cũ ngay sau một lần đổi chiều; reset sau 0,05 m / 0,2 rad / 2 s.

### 5.10 Ví dụ so sánh ảnh hưởng của trọng số

Giả sử có hai quỹ đạo hợp lệ, $v_{\max} = 0{,}3$ m/s.

**Quỹ đạo A: bám lane tốt nhưng gần vật cản**: $d_{path} = 0{,}05$ m, $d_{carrot} = 0{,}25$ m,
$e_{heading} = 0{,}1$ rad, $c_{obs} = 180$, $v = 0{,}25$ m/s.

$$
J_A = 0{,}5(0{,}05) + 2(0{,}25) + 0{,}3\frac{0{,}1}{\pi} + 4\frac{180}{252} + 1{,}5\frac{0{,}3 - 0{,}25}{0{,}3}
\approx 3{,}64
$$

**Quỹ đạo B: lệch lane hơn nhưng xa vật cản**: $d_{path} = 0{,}18$ m, $d_{carrot} = 0{,}30$ m,
$e_{heading} = 0{,}2$ rad, $c_{obs} = 30$, $v = 0{,}22$ m/s.

$$
J_B = 0{,}5(0{,}18) + 2(0{,}30) + 0{,}3\frac{0{,}2}{\pi} + 4\frac{30}{252} + 1{,}5\frac{0{,}3 - 0{,}22}{0{,}3}
\approx 1{,}59
$$

Vì $J_B < J_A$, CustomDWB chọn quỹ đạo B. Robot chấp nhận lệch khỏi lane một đoạn để có khoảng cách an toàn lớn hơn
với vật cản.

### 5.11 Trọng số hiện tại ưu tiên điều gì?

Thứ tự ưu tiên thực tế của cấu hình hiện tại có thể hiểu như sau:

$$
\boxed{\text{Né vật cản} > \text{tiến về carrot} > \text{duy trì vận tốc} > \text{hạn chế dao động} > \text{bám chính xác tâm lane}}
$$

Cần phân biệt hai tầng:
1. **An toàn bắt buộc:** quỹ đạo va chạm, đi vào vùng không hợp lệ hoặc không đủ khả năng phanh bị loại hoàn toàn.
2. **Tối ưu mềm:** trong các quỹ đạo còn an toàn, các trọng số quyết định quỹ đạo nào được ưu tiên.

Do đó, trọng số vật cản 4,0 không chỉ đơn giản là "gấp tám lần" trọng số bám đường 0,5, vì các đại lượng đã được
chuẩn hóa khác nhau. Nhưng nó cho thấy thiết kế hiện tại ưu tiên tránh vật cản mạnh hơn việc bám chính xác tâm lane.

Robot vẫn có xu hướng quay trở lại lane sau khi vượt vật cản nhờ tổng hợp ba thành phần:

$$
0{,}5\,d_{path} + 2\,d_{carrot} + 0{,}3\,\frac{|e_{heading}|}{\pi}
$$

Trong đó $d_{carrot}$ có trọng số lớn nhất, giúp robot tiếp tục tiến theo hướng của đường thay vì chỉ đứng lệch bên
cạnh vật cản.

Nếu muốn thay đổi hành vi:
- Tăng trọng số $c_{obs}$: robot né vật cản rộng và sớm hơn.
- Tăng trọng số $d_{path}$: robot bám lane chặt hơn nhưng có thể khó vòng tránh vật cản.
- Tăng trọng số $d_{carrot}$: robot ưu tiên tiến về phía trước và nhanh trở lại đường sau khi né.
- Tăng trọng số heading: robot căn hướng mạnh hơn, giảm chạy chéo so với lane.
- Tăng trọng số speed: robot ít chấp nhận chạy chậm, nhưng chỉ áp dụng với các quỹ đạo đã an toàn.

> Các trọng số hiện tại là kết quả tuning 2026-10-21 **chỉ trong mô phỏng** (né 35/35 kịch bản chắn đường, 0 va
> chạm), **chưa thử trên robot**. Xếp hạng độ nhạy từng tham số: `stage3_navigation/README.md` mục 9.

### 5.12 Đầu ra và vòng phản hồi

Sau khi chọn quỹ đạo tốt nhất, bộ điều khiển xuất $(v^*, \omega^*)$ lên topic `/cmd_vel_nav` ở tần số khoảng
**10 Hz**. Lệnh này tiếp tục được chuyển qua `twist_mux` tới tầng điều khiển động cơ.

> Chỉ khi `navigation_enabled:=true`. Mặc định (`false`) lệnh ra `/cmd_vel_dwb`, không tới motor.

Ở chu kỳ kế tiếp, CustomDWB lại nhận:
- Pose và vận tốc mới từ odometry.
- Costmap mới từ cảm biến.
- Global path hoặc local path cập nhật.

Vì vậy thuật toán hoạt động theo vòng lặp:

```text
Đo trạng thái → tạo quỹ đạo → kiểm tra an toàn → chấm điểm
      ↑                                           ↓
      └────────────── lệnh vận tốc mới ──────────┘
```

Tóm lại, CustomDWB không chỉ chọn vận tốc gần với vận tốc hiện tại. Nó dự đoán nhiều quỹ đạo trong tương lai, loại
bỏ quỹ đạo không an toàn, sau đó cân bằng giữa tránh vật cản, bám đường, tiến về phía trước, vận tốc và độ ổn định
của robot. Thời gian của vòng này (lệnh → motor → odometry) xem mục 4.

---

## 6. Tài liệu chi tiết

| Chủ đề | File |
|---|---|
| Phần cứng, cổng, URDF | `hardware/README.md`, `hardware/robot_description/README.md` |
| Teleop, driver motor | `stage0_ps4_control/README.md` |
| Odometry, EKF | `stage1_odom/README.md` |
| SLAM, AMCL, CSM, monitor | `stage2_slam/README.md` |
| A\*, CustomDWB, né vật cản | `stage3_navigation/README.md` |
