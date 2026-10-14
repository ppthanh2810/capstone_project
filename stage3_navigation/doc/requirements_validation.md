# Nghiệm thu offline CustomDWB — R1..R10

Ngày: 2026-10-14. Máy: AMD Ryzen 5 7535HS (12 luồng), Ubuntu 22.04, ROS 2 Humble, Nav2 1.1.20.
Trạng thái: **PASS** = có bằng chứng offline trong repo; **PARTIAL** = một phần; **NOT_RUN** = chưa chạy;
**NOT_VERIFIED (hardware)** = cần đo/thử trên robot. **Không có mục nào được coi là đã an toàn trên robot.**

Bằng chứng:
- `T:` = gtest `test_dwa_core` (lõi, 44 test) / `P:` = gtest `test_dwb_controller_plugin` (plugin, 12 test)
- `C:` = benchmark lõi kín vòng `doc/benchmark/core_2026-10-14/{nominal,sluggish}` (17 kịch bản × 5 seed × 2 mô hình robot)
- `R:` = benchmark ROS offline `doc/benchmark/ros_2026-10-14/` (controller_server thật, 12 kịch bản × 3 profile × 5 lần)

| Mã | Yêu cầu | Test / bằng chứng | Kết quả | Trạng thái |
|---|---|---|---|---|
| R1 | Plugin đúng Humble; lifecycle reset; setPlan/param đồng bộ | P: `PluginlibLoadsTheClass`, `LifecycleAndNominalCommand` (configure→activate→deactivate→activate→cleanup→configure), `InvalidYamlFailsConfigure`; R: plugin nạp trong controller_server, `GetParameters` xác nhận tham số đã nạp; khóa `state_mutex_` (plan/lifecycle/compute) → `params_mutex_` → mutex costmap | Tất cả pass | PASS (offline) |
| R2 | Lệnh hữu hạn; vy = 0; không lùi; chính sách overspeed | T: `DynamicWindow.*` (v0 0.50 > max 0.15: lệnh = 0.15 có cờ, mô phỏng giảm tốc thật; v0 âm → lệnh 0, không lùi; ω overspeed dùng `decel_lim_theta`), `Sampler.*` (luôn có 0, khử trùng, mẫu cuối = biên); P: `SpeedLimitHumbleContract`; C: `limit_violations = 0`, `nonfinite_cmds = 0` mọi lần chạy; R: xem bảng R | Pass | PASS (offline) |
| R3 | Chỉ chuyển động đạt được + phanh được; mọi pha đều kiểm va chạm | T: `Rollout.*` (kết thúc đúng horizon, cung chính xác, bước ≤ nửa ô, vật cản 1 ô giữa hai bước lớn bị phát hiện, pose ban đầu được kiểm), `Braking.*`, `Rotation.ResidualForwardSpeedIsCollisionChecked`; C: 0 va chạm ở 170 lần chạy (kể cả robot chậm/trễ hơn giả định); P: `ObstacleAheadBlocksForwardMotion` | Pass | PASS (offline); trễ/gia tốc thật NOT_VERIFIED (hardware) |
| R4 | Dữ liệu cũ/thiếu không tạo lệnh di chuyển tiếp | P: `StaleOrMissingInputsStopTheController`, `MapToOdomAgeIsCheckedButFutureStampsAreFine`, `OldPathStampUsesLatestTransform`, `InvalidPlansAreRejected`; R: `stale_odom` (CustomDWB: lệnh ≠ 0 cuối 0.40 s sau khi odom ngừng, abort, robot dừng sau 0.91 s / 0.11 m; **Nav2 DWB cả hai profile: chạy tiếp 17 s tới đích**, 2.3 m), `stale_scan` (cả 3 profile: server không gọi controller, không publish, không nhận cancel → lệnh hết hạn nhờ timeout 0.5 s kiểu twist_mux, robot dừng ≤ 1.51 s / ≤ 0.20 m; action treo) | Plugin pass; giới hạn server đã đo | PASS cho phần plugin; `stale_scan` = giới hạn của controller_server Humble (P2 #2) |
| R5 | GoalChecker và pha đích thống nhất | T: `Goal.*` (dung sai từ checker, nhả latch khi trôi / khi đúng hướng mà ngoài ngưỡng vào, yaw qua ±π); P: `GoalCheckerTolerancesAreUsed`; C+R: `goal_yaw_pi`, `goal_near`, `start_backwards` | Pass | PASS (offline) |
| R6 | Bám đường + né cục bộ có giới hạn; không rung; không thành công giả | T: `PathTracker.*` (tự cắt, chữ U, tiến trình không giảm, điểm trùng, một pose, NaN), `Oscillation.*`, `Rotation.DirectionHeldNearPi`; C: straight / L / S / tự cắt / trùng điểm / hành lang thành công 5/5; `blocked_wall`, `corridor_narrow`, `overspeed_wall` không bao giờ "thành công"; R: xem bảng R | Pass; **giới hạn**: không vòng qua vật cản nằm trên đường (`obstacle_bypass`, `symmetric_obstacle` → dừng rồi progress checker abort) | PASS (offline) với giới hạn đã nêu |
| R7 | Chính sách footprint/padding/unknown/coverage có căn cứ | T: `CollisionModel.FootprintCornerVsCenterAndUnknown`; log activate: `collision check: center (inscribed 0.294 m, circumscribed 0.341 m)` + cảnh báo `track_unknown_space: false`; C: `corridor_narrow` (A* qua khe 0.54 m, controller dừng: chênh lệch global/local được thể hiện, không giảm padding) | Chính sách mô tả bên dưới | PARTIAL: footprint thật, tâm quay, coverage với scan thật NOT_VERIFIED (hardware) |
| R8 | Tham số nguyên tử, snapshot nhất quán, tải được kiểm soát | T: `Validate.*` (NaN/±Inf ở cả 36 tham số thực, miền, quan hệ, tải); P: `ParameterUpdatesAreAtomicAndValidated` (lô có một giá trị sai bị từ chối toàn bộ); C/R: thời gian tính | xem bảng thời gian | PASS (offline); p99 trên máy robot khi chạy đủ stack: NOT_VERIFIED |
| R9 | Fail/cancel/deactivate kết thúc đúng; không tự chạy; không tranh quyền publish motor | R: graph check pass ở 180/180 lần chạy (không node motor/teleop/driver, `/cmd_vel_nav` 0 publisher, `/cmd_vel_dwb` chỉ `controller_server`, `ROS_LOCALHOST_ONLY=1`); `cancel_midway` 5/5 canceled, lệnh cuối = 0, robot dừng ≤ 0.46 s / ≤ 0.046 m; `blocked_wall` 5/5 abort, không va chạm; mặc định `navigation_enabled:=false`, `auto_follow:=false` không đổi | Pass (ngoại trừ treo action ở `stale_scan`, do server) | PASS (offline) |
| R10 | So sánh CustomDWB / Nav2 công bằng, tái lập được | R: cùng map/scan/odom, cùng giới hạn (0.15 m/s, 0.5 rad/s, 0.5 m/s², 1.0 rad/s²), cùng local costmap và goal/progress checker, cùng điều kiện đầu; YAML sinh tự động lưu kèm; raw từng lần chạy | xem bảng R | PASS (offline) |

## Chính sách va chạm / coverage (R7)

- **Ràng buộc cứng** (loại quỹ đạo): pose bắt đầu, mọi pose của quỹ đạo (trễ → chuyển tiếp → giữ lệnh, tới
  `sim_time`) và mọi pose của đuôi phanh (+ `stopping_margin`) phải có ô tâm < INSCRIBED (253), nằm trong local
  costmap, và không phải NO_INFORMATION khi `unknown_is_obstacle`. Với footprint đa giác (`collision_check:
  footprint` hoặc `auto` khi costmap không dùng `robot_radius`): thêm viền footprint < LETHAL.
- **Kiểm tra ô tâm chỉ đúng khi**: footprint tròn (hiện `robot_radius 0.20` + `footprint_padding 0.10` → inscribed
  ~0.294 m theo costmap), tâm quay trùng tâm thân, không có phần nhô ra. Chưa kiểm chứng trên robot.
- **Vùng chưa quan sát**: local costmap `track_unknown_space: false` ⇒ nửa sau (NaN) và vùng ngoài tầm tia là FREE,
  không phải unknown; `unknown_is_obstacle: true` **không** bảo vệ vùng đó. Bảo vệ thực tế: không bao giờ lùi
  (`min_vel_x = 0`, lệnh âm không bao giờ được phát), xoay tại chỗ với thân tròn không quét vùng mới, vật cản đã
  đánh dấu không bị xóa khi ra sau robot. Bật `track_unknown_space` là đề xuất P2 #7 (cần scan thật).
- **Không** giảm padding hay cho unknown là free để test qua.

## Thời gian / dừng
Ngân sách đề xuất: p99 compute < 80 ms ở 10 Hz; mọi lần > 100 ms được đếm.
- Lõi (C:, 1 luồng): p99 ≤ 2.6 ms; max 12 ms (trùng lúc benchmark ROS khởi động); 0 lần > 100 ms.
- CustomDWB trong controller_server (R:, 4 worker song song cùng CPU): p99 ≤ 3.6 ms, max 7.0 ms, 0 lần > 100 ms;
  chu kỳ lệnh p99 ≈ 101–102 ms mọi profile; 0 cảnh báo "missed its desired rate".
- Nav2 DWB: không có số đo thời gian tính trong plugin (chỉ có chu kỳ lệnh).
- Quãng dừng / độ trễ dừng phụ thuộc mô hình trễ 0.1 s + gia tốc 0.5 m/s² của fixture — **không** phải số đo robot
  (NOT_VERIFIED hardware).

## Kết quả benchmark ROS (bảng R)
Nguồn: `doc/benchmark/ros_2026-10-14/summary.md` (đầy đủ), `runs.csv`, `raw.tar.xz`. Mã chạy = checksum
`sources.md5` (khớp mã hiện tại). 180 lần chạy, 0 va chạm, 0 vượt giới hạn, 0 lệnh lùi, 0 lệnh không hữu hạn.

| Kịch bản (kỳ vọng) | CustomDWB | Nav2 DWB (Standard) | Nav2 DWB (LimitedAccel) |
|---|---|---|---|
| straight, l_turn, s_curve, corridor_ok, goal_yaw_pi, overspeed_start (thành công) | 30/30 thành công | 30/30 | 30/30 |
| obstacle_near_path (thành công) | 5/5, khoảng hở ≥ 0.160 m | 5/5, ≥ 0.157 m | 4/5 (1 abort "Failed to make progress") |
| start_backwards (thành công) | 5/5 | 5/5 | **0/5** (abort "Failed to make progress", nguyên nhân chưa phân tích) |
| blocked_wall (dừng) | 5/5 abort, khoảng hở 0.099 m | 5/5, **0.047 m** ở 3 lần | 5/5, 0.099 m |
| cancel_midway (dừng) | 5/5, dừng ≤ 0.46 s | 5/5 | 5/5 |
| stale_odom (dừng) | **5/5 dừng** (0.91 s, 0.11 m) | **0/5**: chạy tiếp tới đích | **0/5** |
| stale_scan (dừng nhờ timeout hạ nguồn) | 5/5 dừng ≤ 1.51 s, action treo | như CustomDWB | như CustomDWB |
| Thời gian tới đích trung bình (straight / l_turn / corridor) | 19.6 / 24.9 / 26.3 s | 21.2 / 27.1 / 34.3 s | 21.8 / 29.2 / 34.3 s |
| Bám đường ct RMS (l_turn / s_curve) | 0.027 / 0.039 m | 0.020 / 0.025 m | 0.012 / 0.024 m |

Nhận xét (không phải kết luận chung — 12 kịch bản tổng hợp, fixture không có động lực học/nhiễu LiDAR):
CustomDWB nhanh hơn và an toàn hơn với dữ liệu cũ; Nav2 DWB bám góc cua sát hơn. Không đủ để nói thuật toán nào tốt
hơn trên robot.
