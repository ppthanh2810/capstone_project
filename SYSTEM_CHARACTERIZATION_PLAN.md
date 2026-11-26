# SYSTEM CHARACTERIZATION — Bước 3 Master Plan SAC–DWA

**Ngày:** 2026-11-25 · **Commit gốc:** `b24a487` (main) + thay đổi chưa commit liệt kê ở §8 · **Phạm vi:** audit + triển khai
Navigation dùng chung cho M0 (Optimized DWA) và M1 (SAC‑DWA), đặc tả số cho bước 4–7. Chưa có Scenario Generator, chưa training.

Quy ước nguồn của mỗi thông số:

| Nhãn | Ý nghĩa |
|---|---|
| **[ĐO]** | Đo trên robot thật hoặc trong Gazebo, có log/bag |
| **[CFG]** | Lấy từ code/cấu hình hiện tại |
| **[DX]** | Dẫn xuất bằng tính toán từ [ĐO]/[CFG] |
| **[ĐX]** | Đề xuất, chờ pilot/người dùng xác nhận |
| **[TBD]** | Chưa xác nhận, cần đo |

---

## 0. Tóm tắt

| Hạng mục | Kết quả |
|---|---|
| Carrot Selection | Đã triển khai: cà rốt hợp lệ (`carrot_validation`) |
| Path Blockage | Khôi phục `path_blockage.cpp` (1e87e65) |
| Global Replanning | Event‑triggered (`replan.trigger: stall`) |
| Safe Stop | Có: DWB dừng ngay + bridge `FAILED` giữ đứng yên |
| Cấu hình chung | `config_consistency.py`: 0 FAIL, 3 WARN |
| Kiểm chứng | Build OK, unit 16/16, lint 111/0, Gazebo: 7/7 kịch bản đúng quy tắc đạt; R6 (khe 0.4 m) không ổn định; R4 + monitor bị hủy (§7) |
| Robot thật | **Chưa chạy** (mọi kết luận là sim) |

Quyết định cần bạn xác nhận (ảnh hưởng xe thật): §9.

---

## 1. Kiến trúc hiện tại (đã kiểm từ code)

```text
            ROBOT THẬT                                   GAZEBO (stage4_gazebo, cùng node thuật toán)
PS4 → joy_teleop ─┐                                     (joy:=false trong test)
                  ├→ twist_mux → /cmd_vel_out → zlac8015d_move ──Modbus──> ZLAC8015D
CustomDWB → /cmd_vel_nav ┘   (prio 10 < 100)           zlac_sim (ZlacDriverModel: ramp, watchdog, encoder)
ZLAC feedback → /wheel_feedback → zlac8015d_odom → /wheel_odom ┐
ESP32/BNO055 → imu_odom → /imu_odom ───────────────────────────┴→ EKF → /odometry/filtered, TF odom→base_footprint
RPLIDAR C1 → /scan_raw → scan_rotate → laser_filters (±90° trước) → /scan_filtered     (gpu_lidar + lidar_sim_adapter)
map_server + AMCL (+CSM, monitor) → TF map→odom, /localization/status
```

Navigation sau cải tiến (dùng chung M0/M1; M1 chỉ đổi 5 trọng số + `sim_time`):

```text
/goal_pose → goal_path_bridge ──ComputePathToPose──> planner_server (NavFn A*)
                │                                    global_costmap: static(/map SLAM, không sửa)
                │                                                  + obstacle_layer(/scan_filtered)  ← MỚI (khôi phục)
                │                                                  + inflation 0.40
                ├──FollowPath──> controller_server ── CustomDWB (cà rốt hợp lệ ← MỚI) + local_costmap 2×2 m
                │                     └─ lệnh 0 khi không có quỹ đạo an toàn / dữ liệu hỏng (Safe Stop tức thời)
                └─ watchFollow 5 Hz:  path_blockage trên /global_costmap/costmap
                     chặn ≥ confirm 0.5 s  ∧  không tiến ≥ 0.15 m trong 4 s  ⇒ A* lại từ vị trí hiện tại
                     FollowPath ABORTED (≤ 3 lần)                            ⇒ A* lại
                     A* thất bại ⇒ hủy FollowPath (đứng yên) ⇒ thử lại mỗi 2 s ⇒ 15 s ⇒ FAILED (Safe Stop)
```

---

## 2. Audit bốn chức năng Navigation

### 2.1 Carrot Selection

| | |
|---|---|
| Hiện trạng cũ | `pickCarrot()` (`dwa_core.cpp`) lấy điểm cách 0.8 m theo cung, không kiểm tra costmap |
| Vấn đề | Vật cản mới trên đường → cà rốt nằm trong vật cản; `carrot_distance_scale` (2.0) kéo robot vào vật cản. Tuning 10‑21 phải tăng 0.6→0.8 để "cà rốt nằm sau vật cản nhỏ" (workaround) |
| Giải pháp | `pickValidCarrot()`: nếu ô cà rốt không FREE (≥ INSCRIBED / unknown / ngoài costmap) → dời **tiến** từng nửa ô tới điểm FREE đầu tiên sau vật cản; không có → **lùi** về điểm FREE gần nhất trước vật cản, cờ `carrot_blocked` |
| File | `include/stage3_navigation/dwa_core.hpp`, `src/dwa_core.cpp`, `src/dwb_controller.cpp` (tham số + `/CustomDWB/state`: `carrot_shift`, `carrot_blocked`), `config/nav2_params.yaml` |
| Tắt | `CustomDWB.carrot_validation: false` (hành vi cũ) |

### 2.2 Path Blockage Detection

| | |
|---|---|
| Hiện trạng cũ | Không có. Chỉ gián tiếp: `holding position` → progress checker abort sau 10 s |
| Vấn đề | A* chỉ thấy static map → không phân biệt "DWB đang né" với "đường bị chặn" |
| Giải pháp | Khôi phục `path_blockage.cpp` (ROS‑free, 10 gtest cũ PASS) + `obstacle_layer` trong global costmap. Bị chặn = ô ≥ 99 dọc đường trong 2.5 m phía trước |
| Persistence / hysteresis | Chặn phải liên tục `confirm_time` 0.5 s; bỏ trạng thái chặn chỉ khi thông liên tục `clear_time` 1.0 s (**mới**) |

### 2.3 Global Replanning (event‑triggered)

| Quy tắc yêu cầu | Cách thực hiện |
|---|---|
| DWA tự né được → không replan | Bị chặn nhưng tiến trình dọc đường còn tăng ≥ `stall_progress` 0.15 m → bỏ qua |
| Kẹt kéo dài → A* lại | Chặn ∧ không tiến trong `stall_time` 4 s (< progress checker 10 s) |
| A* dùng vật cản mới | `obstacle_layer` global (map SLAM giữ nguyên, combination max) |
| Chống replan liên tục | cooldown `min_interval` 2 s, `max_obstacle_replans` 10, `max_abort_replans` 3 |
| Không có đường | hủy FollowPath (robot dừng), thử lại tới `wait_timeout` 15 s → `FAILED` |
| Chế độ cũ | `replan.trigger: immediate` (1e87e65) hoặc `replan:=false` |

File: `src/goal_path_bridge.cpp` (khôi phục 1e87e65 + trigger stall/hysteresis), `src/path_blockage.cpp`,
`include/stage3_navigation/path_blockage.hpp`, `CMakeLists.txt`, `launch/navigation.launch.py` (`replan:=true`),
`stage4_gazebo/launch/s3_navigation.launch.py` (`replan`), `config/nav2_params.yaml` (`goal_path_bridge.replan.*`,
global `obstacle_layer`, global costmap 1 → 2 Hz).

### 2.4 Safe Stop

| Tình huống | Hành vi (code) | Thời gian tới lệnh 0 |
|---|---|---|
| Không có quỹ đạo an toàn | CustomDWB `PlannerException` → controller_server lệnh 0, abort sau 0.3 s | ≤ 1 chu kỳ (0.1 s) [CFG] |
| Chỉ còn v = 0 an toàn | `holding position` (v = 0) | ≤ 0.1 s [CFG] |
| Odom im > 0.5 s, TF cũ, pose NaN | PlannerException | ≤ 0.1 s [CFG] |
| LiDAR đứng (costmap không current) | controller_server không phát → twist_mux 0.5 s + watchdog 0.5 s | ≤ ~1.0 s [CFG] |
| Không còn đường (sau replan) | bridge hủy FollowPath, giữ phiên, `FAILED: …` | ≤ 0.2 s sau quyết định [CFG] |
| Driver ramp về 0 | `decel_time_ms` 500 | +0.5 s [CFG] |

Bridge không bao giờ phát `cmd_vel` (giữ nguyên nguyên tắc cũ). Sau `FAILED` không có goal FollowPath nào → không
nguồn lệnh nav → robot đứng yên tới goal mới.

---

## 3. Đặc tính robot chuẩn (Single Source of Truth)

Nguồn duy nhất: `stage3_navigation/config/nav2_params.yaml` (navigation), `stage4_gazebo/config/s0_control/zlac_sim.yaml`
(driver sim), URDF/xacro (hình học). Kiểm bằng `stage4_gazebo/test/config_consistency.py`.

### 3.1 Hình học và động học

| Thông số | Giá trị | Nguồn |
|---|---|---|
| Footprint | tròn Ø0.40 m (R 0.20) | [CFG] URDF |
| Chiều cao robot + laptop | 0.60 m | [ĐO] 2026‑11‑03 |
| Khối lượng | 12 kg | [ĐO] 2026‑11‑03 |
| Bán kính bánh / track | 0.0535 / 0.34 m | [CFG], 4 nơi khớp |
| Track hiệu dụng | ~0.334 m (encoder < IMU 1.9 %) | [ĐO] 1 lượt, [TBD] |
| LiDAR cao | 0.347 m | [CFG] URDF |
| Vật cản LiDAR thấy | cao ≥ ~0.40 m (0.297 không thấy, 0.497 thấy) | [ĐO] Gazebo |
| Không lùi | `min_vel_x` 0 | [CFG] (phía sau không có LiDAR) |

### 3.2 Vận tốc, gia tốc, phanh, trễ

| Thông số | Giá trị | Nguồn |
|---|---|---|
| `max_vel_x` / `max_vel_theta` | 0.3 m/s / 1.0 rad/s | [CFG] |
| `acc/decel_lim_x` | 0.5 / 0.5 m/s² | [CFG], [TBD] |
| `acc/decel_lim_theta` | 2.0 / 2.0 rad/s² | [CFG], [TBD] |
| Driver ramp | 500 ms mỗi lần đổi lệnh (`time_to_target`) | [CFG] code; mô hình [ĐX] |
| Bước 0→0.3 m/s (sim) | 10–90 % 0.40 s, tới đích 0.50 s | [ĐO] Gazebo |
| `control_latency` | 0.2 s | [ĐO] bag rot_debug (accel **200 ms**) |
| Lệnh → RPM ≥ 1 | thật 0.26 s / sim 0.08 s | [ĐO] |
| Ngừng lệnh → dừng (sim) | 1.05–1.10 s (mux 0.5 + watchdog 0.1 + ramp 0.5) | [ĐO] |
| Quãng phanh thiết kế ở 0.3 m/s | 0.20 m = v(τ+dt) + v²/2a + 0.02 | [DX] |
| Thời gian tới dừng | 0.9 s | [DX] |

Mô hình motor giữ đơn giản: **trễ thuần + ramp tới đích trong `accel/decel_time` + kẹp ±3000 rpm** (`ZlacDriverModel`).
Không mô hình điện. Hai tham số cần đo lại trên xe với cấu hình cuối: trễ thuần và nghĩa của `accel_time` (§9 Q2).

### 3.3 Cảm biến, costmap, tần số

| Thông số | Giá trị | Nguồn |
|---|---|---|
| LiDAR | 720 tia, 10 Hz, 0.05–16 m | [ĐO] log robot / [CFG] |
| FOV dùng | ±90° trước (sau = NaN) | [CFG] laser_filters |
| Local costmap | 2×2 m rolling, 0.05 m, 5 Hz update | [CFG] |
| Inscribed local | 0.25 m (R + padding 0.05) | [CFG] |
| Inscribed global | 0.21 m (R + padding mặc định 0.01) | [CFG] |
| Inflation local / global | 0.30 / 0.40 m, factor 3.0 | [CFG] |
| Obstacle / raytrace | 2.5 / 3.0 m (global = local) | [CFG] |
| Clearing | raytrace; `inf_is_valid false`; nửa sau không xóa | [CFG] |
| Unknown | local `track_unknown_space false` (unknown = free) | [CFG] |
| Global costmap | 2 Hz (mới, cũ 1 Hz) | [CFG] |
| controller | 10 Hz; EKF 30 Hz; wheel 20 Hz; IMU 50 Hz | [CFG]/[ĐO] |
| CustomDWB compute | p99 ~3 ms / 100 ms | [ĐO] offline 10‑20 |

### 3.4 Thông số dẫn xuất (`config_consistency.py`)

| Đại lượng | Giá trị | Nguồn |
|---|---|---|
| Khe tối thiểu A* | ~0.47 m | [DX] 2·0.21 + 1 ô |
| Khe tối thiểu DWB | ~0.55 m | [DX] 2·0.25 + 1 ô |
| Khe đã qua (sim lõi 10‑21) | 0.58 / 0.64 qua, 0.54 dừng | [ĐO] sim lõi |
| `T_sim` tối đa (costmap ±1 m) | 3.17 s ở 0.3 m/s | [DX] (0.95 m / v) |
| Hở thân nhỏ nhất thiết kế | 0.05 m (padding) | [CFG] |

---

## 4. Định nghĩa critic và miền action cho SAC (bước 9, dùng chung M0/M1)

Công thức trong code (`DwaCore::compute`), mọi thành phần chấm ở **điểm cuối** quỹ đạo (không cộng dồn theo số điểm
rollout → đổi `T_sim` không làm lệch scale do số điểm):

`J = wP·P + wG·G + wO·O + wH·H + wS·S + wosc·Osc`

| Ký hiệu | Tham số code | Thành phần | Thang |
|---|---|---|---|
| P | `path_distance_scale` | khoảng cách điểm cuối → đường cục bộ | m |
| G | `carrot_distance_scale` | khoảng cách điểm cuối → **cà rốt** (không phải goal) | m |
| O | `obstacle_scale` | max cost dọc cung dài `max_vel_x·sim_time` / 252 | 0–1 |
| H | `heading_scale` | lệch hướng tới cà rốt / π | 0–1 |
| S | `speed_scale` | (v_max − v)/v_max | 0–1 |
| Osc | `oscillation_scale` = 1.0 (cố định) | đổi chiều ω | 0/1 |
| T_sim | `sim_time` | chân trời tính từ "bây giờ", gồm `control_latency` | s |

Miền action đề xuất [ĐX] (giá trị hiện tại = điểm giữa vùng tốt của sweep 10‑26):

| Action | Hiện tại | Miền |
|---|---|---|
| wP | 0.5 | [0.0, 2.0] |
| wG | 2.0 | [0.5, 4.0] (> 0: `validateParams` cần P + G > 0) |
| wO | 4.0 | [0.0, 8.0] |
| wH | 0.3 | [0.0, 1.5] |
| wS | 1.5 | [0.0, 3.0] |
| T_sim | 2.0 | [1.5, 3.0] s |

- Cận trên 3.0 s < 3.17 s (§3.4); cận dưới 1.5 s: sweep 10‑26 cho thấy `sim_time` 1.0 + robot trễ 0.7 s → 31/75 va chạm.
- An toàn không phụ thuộc action: đuôi phanh được kiểm tra cứng từ thời điểm cam kết (latency + dt), độc lập `T_sim`.
- Tải tính toán ở T_sim 3.0: ~43 000 pose/chu kỳ < giới hạn 100 000 (`kMaxRolloutPosesPerCycle`) [DX].
- Giao diện cập nhật: M0 dùng `ros2 param set` (đã kiểm nguyên tử). M1: chọn ở bước 9 sau khi đo latency của
  parameter service (giữ cùng `validateParams`).

---

## 5. Thông số cho bước 4–7

### 5.1 Quy tắc chung của Scenario Generator (4.3) [ĐX, căn cứ §3]

Định nghĩa theo D = 0.40 m (đường kính) rồi đổi ra mét.

| Thông số | Giá trị | Căn cứ |
|---|---|---|
| Ô lưới generator | 0.05 m | = costmap |
| Chiều cao vật cản | 0.6 m (≥ 0.45) | LiDAR 0.347 m thấy ≥ ~0.40 |
| Kích thước phòng | 4–16 m mỗi cạnh | map House 18.8×11.2, Office 14.2×13.5 |
| Hành lang | 0.8–2.5 m | ≥ 2·khe DWB, ≤ ~6D |
| Khe khả thi | **≥ 0.60 m (1.5 D)** | > 0.55 DWB + 1 ô |
| Dải loại trừ | 0.45–0.60 m | A* qua, DWB không chắc → không sinh |
| Khe vô nghiệm (S1) | ≤ 0.40 m | < A* 0.47 |
| Start/goal hở | ≥ 0.50 m tới vật cản | > inscribed + inflation 0.30 |
| Khoảng start → vật cản mới | ≥ 3.0 m dọc đường | > obstacle_max_range 2.5 + 0.5 (A* ban đầu chỉ dùng M) |
| Khoảng vật cản → goal | ≥ 1.0 m | goal không nằm trong vùng né |
| Độ dài đường tham chiếu | 3–15 m | sim đã đạt 4.6–14 m |
| Vật cản B1 | 0.2–0.4 m (0.5–1 D), chặn đường A* | DWB né được, ≥ 0.6 m khe bên |
| Vật cản B2 | 0.6–1.0 m (1.5–2.5 D) | lệch né ≤ `max_path_deviation` 1.0 |
| B3 | chắn kín hành lang, còn tuyến khác, detour 1.2–3.0 | cần replan |
| C1 khe | 0.60–0.80 m, dài 0.4–2.0 m | |
| C2 nhiều vật | 3–8 vật trong hành lang ±1 m quanh đường | |
| C3 zigzag | vật xen kẽ, tâm cách 1.0–2.0 m, lệch ±0.3–0.6 m, ≥ 3 lần đổi hướng; **khe mép‑mép ≥ 0.60 m** | R6 (khe 0.4 m) → ABORTED |
| C4 BARN‑like | lưới CA, ô 0.15 m, fill 0.10–0.35, 4 vòng làm mịn, lọc khe < 0.60 | theo BARN (Perille 2020) — đối chiếu paper |

Kiểm tra khả thi (5.3): BFS/A* trên lưới W đã nở **0.30 m** (= inscribed local 0.25 + 1 ô) ⇒ feasible; nở 0.20 m
không có đường ⇒ unsolvable; còn lại (chỉ đi được giữa 0.20 và 0.30) ⇒ **loại** (vùng mơ hồ). Đây là kiểm tra hình
học, không chứng minh khả thi động học.

### 5.2 Độ khó (4.4) [ĐX — ngưỡng chờ pilot]

Năm điểm con trong [0, 1], tính từ hình học W và đường tham chiếu (không dùng kết quả SAC/DWA):

| Điểm con | 0 ↔ 1 |
|---|---|
| c — clearance nhỏ nhất dọc đường W | ≥ 1.20 m ↔ 0.60 m |
| ρ — mật độ vật cản trong hành lang ±1 m | 0 ↔ ≥ 0.30 |
| τ — số khúc rẽ > 45° | 0 ↔ ≥ 4 |
| δ — detour ratio L_W / L_M | 1.0 ↔ ≥ 2.0 |
| ν — khoảng nhìn thấy vật cản chặn | ≥ 2.5 m ↔ ≤ 1.0 m |

`Dscore = 0.30c + 0.20ρ + 0.15τ + 0.20δ + 0.15ν`. Easy < 0.33 ≤ Medium < 0.66 ≤ Hard. Unsolvable = kiểm tra khả thi
thất bại (tách riêng, không có Dscore). Pilot phải cho thấy mỗi ô type×difficulty trong bảng 7.2 sinh được; nếu không,
ghi ô đó là "không hợp lệ" (ví dụ N1‑Hard đã là 0).

### 5.3 Episode Termination (6.2) [ĐX, căn cứ cấu hình]

| Nhãn | Điều kiện (ground truth W) |
|---|---|
| COLLISION | khoảng cách tâm → vật cản < R = 0.20 m (tolerance 0.005), ưu tiên cao nhất |
| SUCCESS | bridge `SUCCEEDED` ∧ sai số GT ≤ 0.25 m (0.15 checker + 0.10 định vị) ∧ yaw ≤ 0.25 rad; 2D sim: 0.15 m |
| SAFE_STOP | scenario vô nghiệm ∧ bridge `FAILED` ∧ không va chạm ∧ \|v\| ≤ 0.02 m/s, \|ω\| ≤ 0.05 rad/s giữ ≥ 2.0 s |
| STUCK | scenario khả thi ∧ (`FAILED`/`ABORTED` không va chạm, **hoặc** GT dịch < 0.25 m trong 30 s) |
| TIMEOUT | t > T_max = 30 s + 3·L_ref / 0.3 m/s, không nhãn khác |
| SYSTEM_ERROR | node chết, TF/odom/scan mất > 2 s do hạ tầng, RTF < 0.5 kéo dài > 10 s |

- 30 s STUCK > `wait_timeout` 15 s + `stall_time` 4 s + cooldown: không gán STUCK khi hệ thống còn đang phục hồi.
- `FAILED` ở scenario khả thi = STUCK (ghi phụ `FALSE_NO_PATH`), không phải SAFE_STOP.
- `follow_timeout` 120 s của bridge phải ≥ T_max của episode, nếu không bridge hủy trước evaluator (đặt khi chạy benchmark).
- Sự kiện trung gian cần log: `REPLANNING`, `WAITING`, `FOLLOWING (replanned Nx)`, `carrot_blocked`, `holding position`.

### 5.4 Yêu cầu Fast 2D Simulator (bước 9) [ĐX]

1. Dùng **chính** `libdwa_core.so` (DwaCore, `validateParams`, `pickValidCarrot`) — không viết lại DWA.
2. Robot: unicycle + trễ thuần 0.2 s + ramp `time_to_target` 0.5 s (= `ZlacDriverModel`), không lùi.
3. LiDAR 2D: 720 tia, ±90° trước, 10 Hz, 2.5 m đánh dấu / 3.0 m xóa, nhiễu 0.01 m.
4. Costmap: Costmap2D thật (rolling 2×2, inflation 0.30) hoặc tái tạo, cùng ngưỡng 253.
5. Replanning: cùng logic `path_blockage` + quy tắc stall/cooldown (thư viện ROS‑free sẵn có).
6. Đo tương đồng với Gazebo trên cùng manifest (gate G3): SR, CR, thời gian, hở thân.
7. Ứng viên tái dùng: `dwa_closed_loop_sim` + `sim_world.cpp` trong git `64df018` (đã xóa khỏi repo 2026‑10‑20).

### 5.5 Điều chỉnh bảng 7.2

Không đổi số lượng. Ghi chú: N1 không có Hard (đã 0). B1‑Hard và C3‑Easy cần pilot xác nhận sinh được với công thức
§5.2; nếu một ô < 50 % tỷ lệ sinh hợp lệ sau 200 lần thử, chuyển phần thiếu sang mức kề và ghi lại.

---

## 6. Thứ tự triển khai tiếp và tiêu chí nghiệm thu

| # | Việc | Nghiệm thu |
|---|---|---|
| 1 | Bạn xác nhận Q1–Q4 (§9) | quyết định ghi vào Master Plan |
| 2 | Regression Gazebo lặp 3 lần (ổn định) | R1–R7 PASS 3/3 |
| 3 | Đo trên xe (kê bánh → sàn): bước vận tốc, phanh 0.3 m/s, trễ, quay 360°×3 | bag + bảng [ĐO] |
| 4 | Cập nhật `zlac_sim` (command_delay) theo số đo | sim vs thật lệch < 10 % (góc, thời gian dừng) |
| 5 | Đóng băng Improved DWA (tag git) | gate G1 |
| 6 | Generator + feasibility/difficulty checker | gate G2 |
| 7 | Fast 2D sim từ `dwa_core` | gate G3 |

---

## 7. Kết quả kiểm thử (2026‑11‑25)

Bằng chứng: `stage4_gazebo/test/results/regression_2026-11-25/` (JSON từng lượt + `config_consistency.txt`).
Môi trường: Gazebo Fortress headless, kho AWS, `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`, AMCL + CSM thật, RTF ~1.0.
Không chạm phần cứng.

### 7.1 Build, unit, lint, cấu hình

| Kiểm tra | Kết quả |
|---|---|
| `colcon build` stage3 + stage4 | OK, 0 warning |
| gtest `pickValidCarrot` (6, scratchpad) | 6/6 PASS |
| gtest `path_blockage` (10, từ 1e87e65) | 10/10 PASS |
| `colcon test` stage3 (lint) | 111 / 0 failures (11 skipped) |
| `colcon test` stage4 | 11 / 0 failures |
| `config_consistency.py` | 0 FAIL, 3 WARN (§9 Q1–Q3) |

Unit test cà rốt: điểm tự do không đổi; vật cản lethal/inscribed trên điểm danh nghĩa → dời tiến ra sau vật cản; chặn
tới mép → lùi về trước vật cản (`blocked`); không còn điểm → `found=false`; `compute()` dùng cà rốt hợp lệ và
`carrot_validation:=false` trả về hành vi cũ.

### 7.2 Regression Gazebo (`s3_regression.py`, (0,0) → (0,−5.5))

Vật cản tĩnh cao 0.6 m, đặt **ngoài tầm LiDAR lúc gửi goal** (y ≤ −3.0). Hở = khoảng cách GT tâm → hộp − 0.20 m.

| ID | Kịch bản | Lượt | Kết quả | Replan | Hở min (m) | carrot_shift max |
|---|---|---|---|---|---|---|
| R1 | không vật cản | 2, 3 | SUCCEEDED ×2 | 0 | — | 0 |
| R2 | hộp 0.3 chắn đường | 2, 3 | SUCCEEDED ×2 | 0 | 0.078 / 0.134 | 0.675 / 0.30 |
| R3 | hộp 0.4×1.2 dọc đường | 2, 3 | SUCCEEDED ×2 | 1 / 1 | 0.109 / 0.046 | 0.75 / 0.325 |
| R4 | tường 3.0 m chắn ngang | 3, 4 | SUCCEEDED ×2 | 2 / 1 | 0.079 / 0.106 | 0.35 |
| R4 | (monitor bật) | 2 | **CANCELED** UNCERTAIN > 2 s | 0 | 0.11 | 0.35 |
| R5 | khe 0.65 m | 3 | SUCCEEDED | 0 | 0.098 | 0 |
| R6 | 3 hộp, khe mép 0.4 m | 3, 4 | **ABORTED** / SUCCEEDED | 4 / 0 | 0.021 / **0.009** | 0.75 |
| R6b | 3 hộp, khe mép 0.8 m (đúng quy tắc) | 4 | SUCCEEDED | 0 | 0.106 | 0.675 |
| R7 | đích bị rào kín (S1) | 3 | **FAILED** sau 15.2 s chờ | 11 | 0.023 | 0.55 |
| R8 | khe 0.50 m (thông tin) | 3 | SUCCEEDED (vòng ngoài) | 1 | 0.043 | 0.60 |

Chống replan liên tục:

| Chỉ số | Kết quả |
|---|---|
| Khoảng cách nhỏ nhất giữa hai REPLANNING | 2.0 s (R7, = cooldown), 4.2 s (R4) |
| Số replan lớn nhất | 11 (R7) ≤ giới hạn 13 |
| Vòng abort (R6 lượt 3) | dừng ở `max_abort_replans` 3 → ABORTED, không lặp vô hạn |
| DWB tự né được (R2, R5, R6b) | 0 replan (trigger stall không kích hoạt) |
| Vết cũ trên đường về | "path is clear again" sau 1.2 s, 0 replan |

Safe Stop (R7): `REPLANNING` → `WAITING: no path` → thử lại mỗi 2 s → `FAILED: no free path to the goal for 15.2 s`;
dịch chuyển GT trong 2 s sau khi ramp xong: **0.0 mm, 0.0°**; không va chạm.

### 7.3 Phát hiện

1. **Vật cản trong tầm LiDAR lúc gửi goal được A* ban đầu né luôn** (lượt 1: R3/R4 có đường 6.6 m ngay từ đầu). Đúng
   về kỹ thuật (A* dùng dữ liệu đã quan sát), nhưng generator phải đặt vật cản "mới" cách start ≥ 3.0 m (§5.1).
2. **Localization monitor hủy điều hướng khi vật cản lớn chưa có trong map** (R4 có monitor: UNCERTAIN > 2 s trước khi
   kịp replan). Cùng loại với lần chạy robot 2026‑10‑17. Ảnh hưởng B2/B3/S1 → Q4.
3. **Khe mép‑mép < khe DWB (R6) không ổn định**: một lần ABORTED (A* lập lại đường DWB không bám được, 4 abort), một
   lần tới đích với hở **9 mm**. Hở thân thiết kế 0.05 m (padding) có thể mất gần hết do kiểm tra ô tâm + lượng tử hóa
   0.05 m → Q1.
4. **R8 (khe 0.50 m)**: A* (inscribed 0.21) lập đường qua khe, DWB (0.25) không qua; event‑trigger phát hiện và A* lập lại
   vòng ngoài → tới đích. Footprint lệch được replan bù khi còn đường khác; nếu không có đường khác → FAILED giả (STUCK).
5. **R7 lượt 1**: rào sát vật trên map (hở 0.15 m) → robot vào hốc; global costmap coi ô robot là lethal → không lập
   được đường nào nữa (không lùi). Kịch bản S1/B3 không được tạo hốc hẹp hơn khe DWB.
6. Một lượt R3 cần 1 replan: DWB dừng trước hộp dài 1.2 m (lệch hướng > `rotate_to_heading_angle`), sau 4.2 s replan.

Chưa làm: lặp đủ 3 lần mỗi kịch bản; world Classroom/House; camera; đo trên robot.

---

## 8. Thay đổi đã thực hiện (chưa commit)

| File | Thay đổi |
|---|---|
| `stage3_navigation/include/stage3_navigation/dwa_core.hpp` | `carrot_validation`, `CarrotChoice`, `pickValidCarrot`, `carrot_shift/blocked` |
| `stage3_navigation/src/dwa_core.cpp` | `pickValidCarrot()`, dùng trong `compute()` |
| `stage3_navigation/src/dwb_controller.cpp` | tham số `carrot_validation`; state `carrot_shift`, `carrot_blocked` |
| `stage3_navigation/src/goal_path_bridge.cpp` | khôi phục 1e87e65 + `replan.trigger/stall_time/stall_progress/clear_time` |
| `stage3_navigation/src/path_blockage.cpp`, `include/.../path_blockage.hpp` | khôi phục 1e87e65 |
| `stage3_navigation/CMakeLists.txt` | thư viện `path_blockage` |
| `stage3_navigation/launch/navigation.launch.py` | arg `replan` (mặc định true) |
| `stage3_navigation/config/nav2_params.yaml` | global `obstacle_layer`, 2 Hz, `replan.*`, `carrot_validation` |
| `stage4_gazebo/launch/s3_navigation.launch.py` | arg `replan` |
| `stage4_gazebo/test/s3_regression.py` | **mới**: regression R1–R8 |
| `stage4_gazebo/test/config_consistency.py` | **mới**: kiểm nhất quán cấu hình |
| `stage4_gazebo/CMakeLists.txt` | cài 2 script trên |
| `stage4_gazebo/test/results/regression_2026-11-25/` | **mới**: bằng chứng kiểm thử |
| `stage3_navigation/README.md` | đầu file + mục 16 (replan, cà rốt) |
| `SAC_DWA_Research_Training_Master_Plan_v2.md` | **mới** |

Không đụng: stage0–2, hardware, URDF, maps, `PROGRESS.md`, Master Plan v1.

---

## 9. Quyết định cần bạn xác nhận

| # | Quyết định | Phương án đề xuất | Ảnh hưởng |
|---|---|---|---|
| Q1 | Footprint global ≠ local (0.21 vs 0.25) và hở thân 0.05 m | Đặt global `footprint_padding: 0.05` (cùng 0.25); cân nhắc local padding 0.05 → 0.10 (R6 hở 9 mm) | Xe thật: A* không qua cửa < ~0.55 m; khe DWB tối thiểu tăng lên ~0.65 m nếu padding 0.10 |
| Q2 | Ramp driver 500 ms nhưng `control_latency` đo ở 200 ms | Chọn một giá trị chuẩn (đề xuất giữ 500 ms) và **đo lại** trễ lệnh→odom trên xe (kê bánh) | stage0 + `CustomDWB.control_latency` |
| Q3 | Sim không có trễ lệnh (0.08 s vs thật 0.26 s) | `zlac_sim.command_delay: 0.15` làm profile chuẩn cho benchmark (sau Q2) | chỉ stage4 |
| Q4 | Monitor hủy khi vật cản lớn | Benchmark sim chạy `require_localized: false` (localization không phải đóng góp chính, 1. Problem Definition); trên robot giữ `true` + giám sát | B2/B3/S1 trên robot có thể bị CANCELED |
| Q5 | Warehouse/Office | Chỉ dùng kiểm thử hệ thống, không pre‑train | tránh leakage |

Lệnh đo trên robot (bạn tự chạy; **kê bánh / E‑stop**):

```bash
ros2 launch stage3_navigation navigation.launch.py navigation_enabled:=true monitor:=false auto_follow:=true
ros2 bag record -o ~/char_$(date +%m%d) /cmd_vel_nav /cmd_vel_out /wheel_feedback /odometry/filtered \
  /imu_odom /scan_filtered /CustomDWB/state /goal_path_bridge/follow_state
```

Phân tích bag bằng `stage4_gazebo/test/real_vs_sim/` (đã có) để có bước vận tốc, quãng phanh, trễ, track hiệu dụng.
