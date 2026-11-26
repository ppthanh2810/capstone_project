# Báo cáo nghiệm thu — Replanning V5 "Local Avoidance First" (2026-11-27)

Phạm vi: xây dựng lại cơ chế Replanning, so sánh R0–R3 bằng thực nghiệm, chọn cơ chế dùng chung cho M0/M1, kiểm chứng
Fast 2D + Gazebo, chạy lại Sensitivity / Oracle Gap / Ceiling và G3. **Chỉ mô phỏng**, không chạy robot, không mở
`*_test`, không đổi dataset v3. Chưa commit.

Dữ liệu và hình: `stage5_benchmark/results/2026-11-27/` (episode nén, báo cáo JSON/MD, hình `figs/`).
Lệnh chạy: §10.

## 0. Tóm tắt

| Câu hỏi | Kết luận |
|---|---|
| V4 (early 1 m) có làm A\* né thay DWA? | **Có.** Local Avoidance Rate 0 % (dev), replan ở 124 đoạn mà DWA tự vượt được; lúc kích hoạt robot còn 0.30 m/s và 81 % cung tự do |
| V4 có làm giảm độ nhạy trọng số? | **Có.** SR 98.9–100 % với cả 8 bộ tham số, kể cả `obstacle_scale` = 0 |
| Cơ chế chọn | **R2s** = Local Avoidance First + tiếp cận chậm 0.15 m/s (`nav2_params.yaml`) |
| Dev, bộ mặc định | SR 99.2 % (V4 100), STR 0.4 %, CR 0, replan 16.6 % ep (V4 38.6 %), LAR 57.7 % (V4 0 %) |
| An toàn | CR 0 % ở mọi cơ chế × 8 bộ tham số (Fast 2D); Gazebo regression ×3: 0 va chạm |
| Regression Gazebo | 35/39; R12 FAIL có từ trước; R7 1/3 ABORTED (đã dừng) thay vì FAILED |
| Sensitivity (§7) | V5 trả lại độ nhạy: SR 82.8–100 % theo 30 bộ (V4 98.8–100 %, replan 37 % ở mọi bộ); CR 0 mọi bộ |
| Oracle Gap đúng (§7) | SR 0 / 2.5 điểm (main / stress), q 0.016 / 0.052 < noise floor → khoảng trống SAC nhỏ |
| G3 (§8) | Khớp nhãn 103/106 episode hợp lệ (97 %), 0 va chạm, sai số thời gian 0.45 % |

## 1. Vấn đề của V4 (đo trước khi sửa)

CustomDWB chỉ "nhìn" vật cản trong cung dài `max_vel_x × sim_time` = 0.3 × 2.0 = 0.6 m (chi phí vật cản mềm) và hard
check trong `sim_time`. V4 lập lại A\* khi ô bị chặn còn ≤ 1.0 m dọc đường, tức **trước khi DWA bắt đầu phản ứng**.

General train (900 scenario, bộ mặc định):

| Cơ chế | SR | Episode có replan | LAR |
|---|---:|---:|---:|
| R0 (V3 stall 4 s) | 97.2 | 18.2 | 49.1 |
| R1 (V4 early 1 m) | 99.9 | 35.4 | **0.3** |
| R3 (chỉ DWA) | 82.8 | 0 | 51.9 |

LAR (Local Avoidance Rate) = tỷ lệ đoạn tương tác (vật cản mới Δ chặn đường A\* ban đầu, điểm đường cách Δ < 0.25 m)
được vượt qua mà KHÔNG có replan từ lúc tới s_in − 2.5 m đến lúc vượt s_out + 0.3 m.

## 2. Thiết kế lại ReplanSupervisor

### 2.1 Trạng thái

| Trạng thái | Điều kiện | Hành động |
|---|---|---|
| NORMAL | đường thông | — |
| PATH_BLOCKED | ô global ≥ 99 trên đường trong 2.5 m, liên tục `confirm_time` 0.5 s | **không lập lại**; ô chặn ≤ 1.2 m → SpeedLimit 0.15 m/s |
| LOCAL_BLOCKED | bằng chứng DWA xấu liên tục 0.4 s, hoặc không tiến hữu ích 4 s | → GLOBAL_BLOCKED |
| GLOBAL_BLOCKED | A\* lập lại (cooldown 2 s, ≤ 10 lần) | đường mới → NORMAL |
| SAFE_STOP | hết lượt, hoặc không có đường 15 s | `FAILED`, đứng yên |

Abort của controller (patience 0.3 s / progress 10 s) vẫn lập lại tối đa 3 lần như V3.

### 2.2 Tín hiệu (ROS-free, `replan_supervisor.cpp`; bridge đọc `/CustomDWB/state`)

| Nhóm yêu cầu | Tín hiệu dùng | Ghi chú |
|---|---|---|
| Path Blockage | khoảng cách tới ô chặn (`block_dist`) | dùng để bật tiếp cận chậm |
| Valid Trajectories | DWA `BLOCKED` / `n_forward_valid` = 0 | đứng yên / chỉ xoay = không tiếp tục được |
| Clearance (không gian) | tỷ lệ cung tiến tự do `fs_forward_free/fs_arcs` < 0.15 | 21 cung 0.6 m, \|κ\| ≤ 4 m⁻¹ |
| Useful Progress | tiến ≥ 0.15 m dọc đường **hoặc** dịch chuyển ≥ 0.3 m trong 4 s | đi vòng = tiến triển |
| Persistence | 0.4 s liên tục; xác nhận chặn 0.5 s | |
| Cooldown / Retry | 2 s; 10 + 3 lần | như V3 |

Phép đo không gian trống (`DwaCore::probeFreeSpace`) tính trên local costmap mỗi chu kỳ, mọi pha, với hằng số cố
định → **không phụ thuộc trọng số, `sim_time`, vận tốc**; không ảnh hưởng lệnh (gtest `FreeSpace.*`).

Không dùng (có bằng chứng, §3): khoảng hở tâm robot, độ lệch khỏi đường, tỷ lệ quỹ đạo hợp lệ, khoảng cách tới goal.

### 2.3 Thay đổi DwaCore

`obstacle_lookahead` dùng `max_vel_x` danh nghĩa thay vì `max_vel_x_eff`. Không có SpeedLimit → **không đổi**
hành vi. Có SpeedLimit 0.15 m/s → tầm chi phí vật cản giữ 0.6 m (trước đó co còn 0.3 m, DWA né muộn hơn; Gazebo
R2_detour replan 3/3 thay vì tự vòng).

### 2.4 An toàn (không đổi, độc lập trọng số / SAC / replan)

Loại cứng (va chạm, phanh, unknown, ngoài costmap) trong DwaCore; không còn quỹ đạo hợp lệ → DwaCore không phát
lệnh → controller_server phát 0 ngay (gtest `Safety.NoAdmissibleTrajectoryMeansNoCommand`), không chờ xác nhận replan.
Giữ nguyên: footprint clearing tắt, padding 0.05, local 4×4 m @ 0.025 m, inflation 0.30, `rotate_to_heading_angle`
2.5, mô hình motor (trễ 0.15 s + ramp 0.5 s), không lùi, không recovery planner.

## 3. Chọn ngưỡng từ dữ liệu (general_train, không dùng dev)

### 3.1 Tín hiệu nào báo trước kẹt (R3 chỉ DWA, 166 đoạn tự né thành công vs 154 đoạn kẹt)

| Quy tắc (khi đường bị chặn) | Báo nhầm trên đoạn tự né | Bắt được đoạn kẹt | Báo trước (trung vị) |
|---|---:|---:|---:|
| cung tự do < 0.10, giữ 0.4 s | 12/166 | 145/154 | 2.4 s |
| cung tự do < 0.15, giữ 0.4 s | 14/166 | 147/154 | 2.6 s |
| cung tự do < 0.30, giữ 0.4 s | 40/166 | 149/154 | 3.0 s |
| khoảng hở tâm < 0.05 m, giữ 0.4 s | 83/166 | 91/154 | 1.2 s |
| khoảng hở tâm < 0.10 m | 165/166 | 154/154 | — |
| đứng yên (v < 0.02), giữ 1 s | 9/166 | 43 kịp / 90 muộn | — |

- DWA né thành công vẫn sát vật cản (khoảng hở tâm min trung vị 0.04 m) → khoảng hở không tách được.
- Độ lệch khỏi đường khi ô chặn còn ≤ 0.4–1.0 m: < 0.10 m ở 164/165 đoạn thành công (DWA né rất muộn) → không tách.
- Mức chặn (độ dịch ngang cần thiết): trung vị 0.45 m ở đoạn tự né thành công, 1.05 m ở đoạn kẹt; trùng nhau lớn.
- Kết luận: tín hiệu cục bộ chỉ chắc chắn khi ô chặn còn ~0.2–0.35 m.

### 3.2 Lưới mô phỏng (540 episode có vật cản, train)

| Cấu hình | SR | STR | Replan thừa¹ | LAR | t² |
|---|---:|---:|---:|---:|---:|
| R1 | 99.8 | 0.0 | 163 | 0.3 | 25.3 |
| cung < 0.10, giữ 0.4 s | 97.0 | 2.0 | 13 | 47.8 | 25.6 |
| cung < 0.15, giữ 0.4 s (= R2) | 97.6 | 1.7 | 15 | 47.2 | 25.7 |
| cung < 0.20, giữ 0.4 s | 97.6 | 1.3 | 19 | 46.2 | 25.7 |
| cung < 0.15, giữ 1.0 s | 95.9 | 3.0 | 12 | 48.1 | 25.7 |
| + tiếp cận 0.15 m/s ≤ 1.2 m (R2s, trước sửa §2.3) | 98.5 | 1.1 | 58 | 35.0 | 28.6 |
| + tiếp cận 0.20 m/s ≤ 0.8 m | 97.6 | 1.1 | 32 | 43.4 | 27.1 |

¹ episode có replan mà R3 (chỉ DWA) vẫn thành công. ² trung bình trên 371 episode thành công ở mọi cấu hình.
`useful_displacement` 0 / 0.3 m khác nhau ≤ 0.2 điểm SR (giữ 0.3 để đi vòng không bị coi là kẹt).

Nguyên nhân STUCK còn lại của R2: kích hoạt ở 0.23–0.30 m/s khi ô chặn còn 0.10–0.33 m; sau replan robot còn trôi
(trễ 0.15 s + ramp 0.5 s) và LiDAR thấy thêm cạnh vật cản → ô tâm thành INSCRIBED, không lùi được (Q6). Do đó thêm
tiếp cận chậm (R2s).

## 4. So sánh R0–R3 trên dev (validation)

Dev = general_val 300 + house_val 100 + classroom_val 100 + S1 50 (550). Cùng scenario, seed 0, cấu hình robot.

### 4.1 Bộ mặc định

| Cơ chế | SR | CR | STR | SSR | Replan % | Replan/ep | LAR | t (s)³ | L (m)³ | hở min |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| R0 V3 | 97.4 | 0 | 2.2 | 96 | 17.8 | 0.27 | 54.1 | 27.8 | 7.91 | 0.038 |
| R1 V4 | 100.0 | 0 | 0.0 | 100 | 38.6 | 0.47 | **0.0** | 27.7 | 7.85 | 0.071 |
| R2 | 98.2 | 0 | 1.4 | 96 | 19.2 | 0.28 | 51.0 | 28.0 | 7.92 | 0.040 |
| **R2s** | 99.2 | 0 | 0.4 | 96 | **16.6** | **0.20** | **57.7** | 29.8 | 7.87 | 0.060 |
| R3 chỉ DWA | 83.8 | 0 | 16.0 | 78 | 0 | 0 | 58.2 | 27.8 | 7.91 | 0.038 |

³ episode thành công ở cả 5 cơ chế. Safe Stop Rate (SSR) chỉ trên S1 (vô nghiệm).

### 4.2 Theo nhóm (bộ mặc định): SR / % episode có replan / LAR

| Nhóm | R1 | R2s | R3 |
|---|---|---|---|
| N1–N3 | 100 / 0–2 / – | 100 / 0 / – | 100 / 0 / – |
| B1 | 100 / 100 / 0 | 96.5 / 19 / 81 | 78.9 / 0 / 79 |
| B2 | 100 / 100 / 0 | 100 / 21 / 79 | 77.2 / 0 / 77 |
| B3 | 100 / 100 / 0 | 95.3 / 100 / 0 | 0 / 0 / 0 |
| C1 | 100 / 6 / 0 | 100 / 6 / 0 | 100 / 0 / 100 |
| C2 | 100 / 49 / 0 | 100 / 21 / 59 | 80.7 / 0 / 65 |
| C3 | 100 / 2 / – | 100 / 2 / – | 100 / 0 / – |
| C4 | 100 / 11 / 0 | 100 / 6 / 50 | 94.3 / 0 / 50 |
| S1 (SSR / replan) | 100 / 2.1 | 96 / 1.9 | 78 / 0 |

### 4.3 Nguyên nhân kích hoạt (bộ mặc định, dev) và trạng thái DWA lúc đó (trung vị)

| Cơ chế | Nguyên nhân | Số | Cung tự do | Ô chặn còn (m) | v (m/s) | Hở GT (m) |
|---|---|---:|---:|---:|---:|---:|
| R1 | `distance_1m` | 333 | 0.81 | 1.03 | 0.30 | 0.33 |
| R2s | `local_free_space` | 112 | 0.05 | 0.20 | 0.15 | 0.22 |
| R2s | FollowPath abort | 71 | — | — | 0.11 | 0.14 |
| R2s | không tiến hữu ích | 10 | 0.14 | 0.12 | 0 | 0.21 |
| R2s | DWA BLOCKED | 1 | 0.05 | 0.12 | 0.15 | 0.08 |

("retry" = thử lại khi đang chờ đường, chủ yếu S1: R1 325, R2s 305.) Lúc R2s kích hoạt, DWA vẫn còn 99.6 % quỹ đạo
tiến hợp lệ (trung vị): số quỹ đạo hợp lệ không phải tín hiệu kẹt.

### 4.4 Nhiều bộ tham số (B1–B3, C1, C2, C4 + S1 dev = 331 scenario; default = 550)

Bộ (định nghĩa trước khi chạy, `replan_compare.py` `SETS`): timid (wO 8, wS 0.5, T 2.5), bold (wO 1, wS 3, T 1.5),
pathy (wP 2, wG 1, wH 1), carrot (wP 0, wG 4, wH 0), long (T 3.0), short (T 1.5), wO0 (wO 0).

| Bộ | R0 SR | R1 SR / LAR | R2 SR | R2s SR / STR / LAR | R2s replan % | R3 SR |
|---|---:|---:|---:|---:|---:|---:|
| default | 97.4 | 100 / 0 | 98.2 | 99.2 / 0.4 / 58 | 17 | 83.8 |
| timid | 90.0 | 99.6 / 0 | 90.4 | 96.4 / 0.0 / 56 | 30 | 56.9 |
| bold | 65.5 | 99.3 / 0 | 70.1 | 89.0 / 9.3 / 50 | 35 | 45.6 |
| pathy | 84.3 | 99.3 / 0 | 85.8 | 98.6 / 0.0 / 59 | 28 | 44.8 |
| carrot | 95.7 | 100 / 0 | 98.9 | 98.9 / 0.4 / 63 | 26 | 76.9 |
| long | 93.2 | 99.3 / 0 | 95.0 | 97.5 / 0.7 / 58 | 29 | 70.5 |
| short | 86.5 | 100 / 0 | 91.5 | 97.5 / 1.8 / 61 | 27 | 61.9 |
| wO0 | 63.3 | 98.9 / 0 | 62.6 | 65.8 / 28.8 / 1 | 68 | 34.9 |

CR = 0.00 ở cả 40 ô (cơ chế × bộ). R1 xóa gần hết khác biệt giữa các bộ (98.9–100 %); R2s giữ khác biệt (65.8–99.2 %)
nhưng không phạt nặng bộ hợp lý (≥ 96.4 % trừ bold / wO0). Hình: `figs/summary_sets.png`.

### 4.5 Ảnh hưởng của `sim_time` (R3, general_val B1/B2/C2, cùng vị trí hình học)

| Ô chặn còn | n_forward/n_candidates T 1.5 / 2.0 / 3.0 | cung tự do T 1.5 / 2.0 / 3.0 |
|---|---|---|
| 0.0–0.3 m | 0.69 / 0.78 / 0.71 | 0.41 / 0.50 / 0.49 |
| 0.3–0.6 m | 0.84 / 0.76 / 0.71 | 0.42 / 0.40 / 0.50 |
| 0.6–0.9 m | **1.00 / 0.97 / 0.72** | **0.80 / 0.80 / 0.82** |
| 0.9–1.2 m | 1.00 / 1.00 / 0.96 | 0.87 / 0.87 / 0.87 |

Một trigger theo tỷ lệ quỹ đạo hợp lệ sẽ bị SAC kích hoạt chỉ bằng cách tăng `sim_time` (−28 điểm ở 0.6–0.9 m).
Cung tự do không đổi theo `sim_time` (khác biệt còn lại do vị trí robot khác nhau giữa các lần chạy).

## 5. Phân tích theo yêu cầu (dev, bộ mặc định, ghép cặp R1 / R2s / R3)

| Loại | Số | Ghi chú |
|---|---:|---|
| R1 replan nhưng R2s hoặc R3 vượt đoạn đó không replan | 124 đoạn | R2s tự vượt 113, R3 tự vượt 114; B1 49, B2 47, C2 24, C4 3, C1 1 |
| Bắt buộc cần Global Replanning (R3 thất bại, R2s thành công nhờ replan) | 68 ep | B3 41, B2 10, C2 9, B1 7, C4 1 |
| Giảm replan làm xấu đi (R2s ít replan hơn R1 và kết cục xấu hơn) | 2 | `general_val_B3_h_0001` (TIMEOUT sát T_max, 94.6 vs 93.4 s), `house_s1_val_S1_u_0004` (TIMEOUT thay vì Safe Stop) |
| R2s coi là kẹt nhưng chỉ DWA vẫn thành công | 12 | 8 chỉ `free_space`, 4 kèm abort; hình `general_val_C2_h_0002` |
| Tiềm năng SAC (bộ khác vượt không replan khi default phải replan / thất bại) | 24 ep | B1 7, B2 7, C2 5, B3 3, C4 2 (R1: 0) |

Thất bại của R2s trên dev mặc định (6/550): `general_val_B1_h_0000` STUCK (tiếp cận chậm đổi hướng né của DWA, kẹt
góc tường; R3 chạy nhanh thì thành công — hình), `general_val_B1_h_0004` TIMEOUT, 2 × B3 (TIMEOUT / ABORTED),
2 × S1 TIMEOUT (vòng quanh rào tới hết T_max, Q8).

### 5.1 Hình (map M xám, Δ đỏ, A\* ban đầu xanh đứt, A\* sau replan cam, robot đen, sao = replan; dưới: trạng thái DWA)

| Hình | Nội dung |
|---|---|
| `figs/general_val_B1_e_0002_default.png` | R1 replan, R2s và R3 tự né (LAR) |
| `figs/general_val_B2_h_0001_default.png` | như trên, B2 |
| `figs/general_val_B2_h_0001_bold.png` | bộ bold: R3 kẹt, R2s tự vượt, R1 replan |
| `figs/general_val_C2_h_0002_default.png` | R1 3 replan; R2s 1 (báo nhầm free_space); R3 0 |
| `figs/classroom_val_B3_h_0005_default.png` | B3 bắt buộc replan: R1, R2s thành công, R3 kẹt |
| `figs/general_val_B3_h_0001_default.png` | R2s thử len khe rồi mới replan → TIMEOUT sát T_max |
| `figs/general_val_B1_h_0000_default.png` | R2s STUCK do tiếp cận chậm; R3 thành công |
| `figs/house_s1_val_S1_u_0004_default.png` | S1: R1 Safe Stop, R2s TIMEOUT |
| `figs/summary_sets.png` | SR / LAR / % replan theo bộ × cơ chế |

## 6. Kiểm chứng Gazebo (regression bắt buộc, kho AWS, headless, ×3)

`stage4_gazebo/test/results/regression_2026-11-27/v5config_planner_r3.{json,txt}`.

| Kịch bản | V3 (11-26) | V5 R2s trước sửa §2.3 | **V5 cuối** |
|---|---|---|---|
| R1 normal, R5 narrow, R10 gap060 | 3/3 | 3/3 | 3/3 |
| R2 detour (yêu cầu 0 replan) | 3/3 | **0/3** (replan 1) | **3/3, 0 replan** |
| R3 carrot | 2/3 | 3/3 | 3/3 (0 replan) |
| R4 replan (yêu cầu ≥ 1 replan) | 3/3 | 3/3 | 3/3 |
| R6, R6b, R9, R11 | R11 1/3 | 3/3 | 3/3 |
| R7 no path (yêu cầu FAILED + đứng yên) | 3/3 | 3/3 | **2/3**: 1 lượt ABORTED (4 abort, lệnh 0) |
| R8 gap 0.50 (ngoài quy tắc, INFO) | — | 3 SUCCEEDED | 3 SUCCEEDED |
| R12 LiDAR ngừng (dừng ≤ 1.5 s) | 0/3 (1.9–2.1 s) | 0/3 | 0/3 (có từ trước, không liên quan replan) |
| Tổng | | 33/39 | **35/39**, 0 va chạm, hở min 0.059 m |

## 7. Parameter Sensitivity, Oracle Gap, Ceiling (Fast 2D)

30 bộ tham số (hiện tại + 12 one-at-a-time lo/hi + 17 LHS, `--sets-from v4/sens_r1.json`), 2 seed. Main =
`general_train` 125 scenario (phân tầng), stress = `stress_pilot` 120 scenario. File:
`results/2026-11-27/sensitivity/sens_{main_v5,stress_v5,main_r1}.json`.

Oracle Gap **đúng** (`oracle_gap_v5` trong `sensitivity_study.py`): J trung bình qua seed; oracle = trung bình theo
scenario của max theo bộ; best fixed = max theo bộ của trung bình (CR ≤ 2 %); gap = oracle − best ≥ 0. Bản cũ (chọn
trên nửa A, đo trên nửa B) cho gap âm (main SR −4.8, stress −11.7 điểm): nó đo sai số chọn do nhiễu, không phải
khoảng trống của SAC — giữ làm cận dưới (`gap_*_cross`). Noise floor = gap khi chỉ khác seed.

### 7.1 V5 (R2s)

| Chỉ số | Main | Stress |
|---|---|---|
| SR hiện tại | 99.6 % | 95.8 % |
| SR best fixed / oracle | 100 / 100 % | 97.5 / 100 % |
| **Gap SR** | **0.0 điểm** | **2.5 điểm** |
| q hiện tại / best / oracle | 0.825 / 0.837 / 0.853 | 0.627 / 0.641 / 0.692 |
| **Gap q** | **0.016** | **0.052** |
| Noise floor q | 0.088 | 0.231 |
| CR max (30 bộ) | 0 | 0 |
| SR min–max theo bộ | 82.8–100 % | 55.8–97.5 % |
| Bộ có SR ≥ 95 % | 23 / 30 | 11 / 30 |
| Scenario thành công với mọi bộ | 64.8 % | 14.2 % |
| Scenario không bộ nào thành công | 0 % | 0 % |

### 7.2 Theo nhóm (SR min–max theo 30 bộ)

| Nhóm | Main | Stress |
|---|---|---|
| B1 | 47–100 | 40–100 |
| B2 | 53–100 | 20–100 |
| B3 | 70–100 | 53–100 |
| C1 | 80–100 | 60–100 |
| C2 | 87–100 | 27–97 |
| C3 / C4 | 93 / 87–100 | — |
| N1 / N2 / N3 | 90 / 100 / 93–100 | — |

### 7.3 V5 so với V4 (cùng 125 scenario, cùng 30 bộ, cùng seed)

| Chỉ số (main, 125 × 30 × 2) | V4 (R1) | **V5 (R2s)** |
|---|---|---|
| SR hiện tại | 100 % | 99.6 % |
| SR min–max theo bộ | 98.8–100 % | 82.8–100 % |
| Độ lệch chuẩn SR giữa các bộ | 0.39 điểm | 3.99 điểm |
| STR max theo bộ | 1.2 % | 14.0 % |
| CR max | 0 | 0 |
| % episode có replan (min–max theo bộ) | 37.2–37.6 % | 14.0–37.6 % |
| Bộ có SR ≥ 95 % | 30 / 30 | 23 / 30 |
| Scenario thành công với mọi bộ | 94.4 % | 64.8 % |
| `obstacle_scale` thấp: SR | 99.2 % | 82.8 % |
| q hiện tại / best / oracle | 0.895 / 0.906 / 0.911 | 0.825 / 0.837 / 0.853 |
| Gap q / noise floor | 0.005 / 0.026 | 0.016 / 0.088 |
| Nhóm B1 / B2: SR min theo bộ | 93 / 93 % | 47 / 53 % |

- Dưới V4, replan xảy ra ở **37 % episode với mọi bộ trọng số**: nó do hình học quyết định, không do DWA. Trọng số gần
  như không đổi được kết cục (SR lệch 0.39 điểm) → không còn bài toán cho SAC. Đây là bằng chứng định lượng cho giả
  thuyết của bạn.
- V5 trả lại độ nhạy (độ lệch SR ×10, B1/B2 xuống 47–53 % với bộ tệ) mà vẫn CR 0 với mọi bộ.
- Cái giá của V5 trên q: 0.825 so với 0.895 ở bộ hiện tại (đi chậm khi tiếp cận + tự né dài hơn đường A\* mới).

### 7.4 Trần và khoảng trống cho SAC

- Trọng số quyết định kết cục dưới V5 (bộ tệ: SR 82.8 %, replan 36 % ep), nhưng một bộ cố định tốt đã gần trần: gap
  SR 0 (main) / 2.5 điểm (stress); gap q 0.016 / 0.052 nhỏ hơn noise floor 0.088 / 0.231.
- Khoảng trống còn lại nằm ở tập khó (stress, B1/B2/C2) và ở q (thời gian, hở), không phải SR. Với 2 seed chưa tách
  được khỏi nhiễu → khi làm baseline cần ≥ 5 seed cho tập khó.
- Độ khó Dscore tương quan với thất bại chỉ ở stress (Spearman 0.31, p 5e-4); main 0.12 (p 0.17).
- **Không đổi cơ chế để tăng khoảng trống cho SAC**: V5 được chọn vì DWA tự né (§9), headroom là hệ quả đo được.

## 8. G3: Fast 2D ↔ Gazebo (planner_gt, 24 general_val + 2 S1 × 5 bộ)

Bộ tham số: base, timid_short, bold_long, wO_low, fast_short (`results/2026-11-27/g3/param_sets.json`). Gazebo
headless, domain 94, định vị GT; Fast 2D seed 0–4. File: `results/2026-11-27/g3/` (`cmp.json`, `gz_*.jsonl`).

| Chỉ số | Kết quả |
|---|---|
| Episode Gazebo | 130 (24 general_val + 2 S1 × 5 bộ); Fast 2D seed 0–4 |
| Lỗi hạ tầng | 30: 20 "bridge không có trạng thái 15 s" (4 scenario), 10 REJECTED (2 scenario) — chạy lúc máy quá tải (load ≈ 12/12 lõi) |
| Chạy lại 6 scenario đó (bộ base, tải thấp hơn) | 6/6 khởi động được; 5/6 khớp nhãn |
| **Khớp nhãn, episode điều hướng hợp lệ** | **98 / 100** lượt đầu; **103 / 106** gộp cả chạy lại (97 %) |
| Va chạm | 0 Gazebo, 0 Fast 2D |
| Sai số trung vị thời gian / quãng đường / A\* ban đầu | 0.45 % / 0.19 % / 0.22 % |
| Thời gian trung bình theo bộ (Gazebo / Fast 2D) | base 32.2 / 32.3 s; bold_long 36.2 / 36.3 s — cùng thứ hạng |
| Lệch nhãn | B1_e_0008 timid (Gz SUCCESS, F2 đa số TIMEOUT); C2_e_0013 timid (Gz STUCK sau 4 replan, F2 SUCCESS); B1_h_0004 (Gz TIMEOUT, F2 3/5 SUCCESS) — đều ở vùng biên, F2 cũng không nhất trí theo seed |
| Replan | S1_u_0004: Gazebo 8 lần, Fast 2D 1 lần, cùng kết cục SAFE_STOP; nguyên nhân chênh lệch **chưa phân tích** (Q14) |

So với G3 v3 (17/20 = 85 %, AMCL): **G3 đạt** cho Planner Benchmark.

## 9. Lựa chọn cuối và cơ sở

| Tiêu chí | R1 | R2 | **R2s** |
|---|---|---|---|
| CR ≤ 2 % toàn bộ validation | ✓ (0) | ✓ (0) | ✓ (0) |
| Không va chạm mới ở regression | ✓ | — | ✓ (0 va chạm) |
| Giảm replan không cần thiết | ✗ (124 đoạn) | ✓ | ✓ (11 đoạn còn replan trong 124) |
| Hiệu suất danh định (N1–N3) | 100 % | 100 % | 100 %, thời gian như nhau |
| Hạn chế STUCK (dev mặc định) | 0.0 % | 1.4 % | 0.4 % |
| DWA thật sự tránh vật cản cục bộ | ✗ (LAR 0) | ✓ (51) | ✓ (58) |
| A\* xử lý trường hợp cần đổi tuyến | ✓ | ✓ | ✓ (B3 95.3 %, 68 ep cần A\*) |
| Không phụ thuộc sim_time cho quyết định | — | ✓ | ✓ |

Chọn **R2s**. Cái giá đã đo: SR −0.8 điểm, STR +0.4 điểm, thời gian +2.1 s (+7.5 %, chỉ ở episode có vật cản
chặn), SSR S1 96 % (R1 100 %) so với R1 ở bộ mặc định. Không giảm replan chỉ để có lợi cho SAC: R2s vẫn lập lại khi
DWA hết không gian (112 lần) và B3 vẫn 100 % có replan; DWA không bị làm yếu (thay đổi DwaCore duy nhất giữ nguyên
tầm nhìn vật cản khi bị giới hạn tốc độ). Cùng một `ReplanSupervisor` + `nav2_params.yaml` cho M0, M1, Fast 2D,
Gazebo và robot.

## 10. Lệnh tái lập

```bash
source /opt/ros/humble/setup.bash && source ~/capstone_ws/install/setup.bash
S=~/capstone_ws/capstone_project/stage5_benchmark/scripts; D=~/capstone_ws/bench_data/v3
DEV=general_val,house_val,classroom_val,general_s1_val,house_s1_val,classroom_s1_val
python3 $S/replan_compare.py --data $D --splits general_train --mechs R3,R0,R1 --signals --out design_train.jsonl
python3 $S/replan_compare.py --data $D --splits $DEV --mechs R0,R1,R2,R2s,R3 --out dev_default.jsonl
python3 $S/replan_compare.py --data $D --splits $DEV --types B1,B2,B3,C1,C2,C4,S1 --mechs R0,R1,R2,R2s,R3 \
    --sets timid,bold,pathy,carrot,long,short,wO0 --out dev_sets.jsonl
python3 $S/replan_report.py --inputs dev_default.jsonl,dev_sets.jsonl --out report.json --md report.md
python3 $S/replan_figures.py --data $D --split general_val --ids general_val_B1_e_0002 --mechs R1,R2s,R3 --out figs
stage4_gazebo/test/run_regression.sh ~/capstone_ws/bench_runs/v5/reg2 v5_final_planner --repeat 3 --no-require-localized
SETS=~/capstone_ws/bench_runs/v4/sens_r1.json
python3 $S/sensitivity_study.py --data $D --split general_train --sets-from $SETS --workers 7 --out sens_main_v5.json
python3 $S/sensitivity_study.py --data ~/capstone_ws/bench_data --split stress_pilot --all --sets-from $SETS --workers 7 --out sens_stress_v5.json
python3 $S/sensitivity_study.py --data $D --split general_train --sets-from $SETS --workers 7 \
    --replan '{"trigger":"stall","early_distance":1.0,"early_max_deviation":0.15,"approach_speed":0.0}' --out sens_main_r1.json
python3 $S/gazebo_episode.py --manifest $D/general_val --ids <24 id> --param-sets g3/param_sets.json --out g3/gz_val.jsonl --mode planner_gt --domain 94
python3 $S/sim2gz_compare.py --manifest $D/general_val,$D/general_s1_val --gazebo g3/gz_val.jsonl,g3/gz_s1.jsonl \
    --param-sets g3/param_sets.json --out g3/cmp.json
colcon test --packages-select stage5_benchmark && colcon test-result --verbose   # 12 gtest
```

## 11. Vấn đề còn mở

| # | Vấn đề | Đề xuất |
|---|---|---|
| Q6 | STUCK còn lại = ô tâm thành INSCRIBED (không lùi), footprint clearing tắt | giữ (ràng buộc của bạn); SAC có thể giảm bằng wO / T_sim |
| Q8 | S1: vòng quanh rào tới hết T_max (R2s SSR 96 %) | xem lại T_max cho S1 hoặc watchdog tiến triển |
| Q10 | Tiếp cận chậm đôi khi đổi hướng né của DWA (`general_val_B1_h_0000`) | theo dõi khi baseline / SAC |
| Q11 | Regression R7: 1/3 kết thúc ABORTED (đứng yên) thay vì FAILED | chấp nhận như Safe Stop hoặc nới tiêu chí R7 — cần bạn quyết |
| Q12 | R12 (LiDAR ngừng) dừng 1.8–2.1 s > 1.5 s | có từ trước; không thuộc phạm vi replan |
| Q13 | Gazebo: scenario sinh ra đôi khi không khởi động (bridge không có trạng thái 15 s / REJECTED) khi máy quá tải | chạy Gazebo khi không có tải nền; chạy lại 6/6 OK |
| Q14 | G3 S1_u_0004: Gazebo 8 replan, Fast 2D 1 (cùng SAFE_STOP) | xem log bridge trước khi đo replan rate S1 bằng Gazebo |
| — | Ngưỡng chọn trên mô phỏng, chưa chạy robot; tải CPU free-space probe (6400 ô + 21 cung / chu kỳ) chưa đo trên laptop | chờ bạn |
