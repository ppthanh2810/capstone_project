# Báo cáo nghiệm thu — Master Plan v3, Giai đoạn A–D (2026-11-26)

Phạm vi: mô phỏng (Fast 2D + Gazebo Fortress headless, `ROS_LOCALHOST_ONLY=1`, domain riêng). **Không chạm phần cứng.**
Mọi thay đổi **chưa commit**. Số liệu thô: `stage5_benchmark/results/2026-11-26/` và
`stage4_gazebo/test/results/regression_2026-11-26/`; dataset ngoài repo `~/capstone_ws/bench_data/v3`.

## 0. Tóm tắt

| Giai đoạn | Kết quả | Trạng thái |
|---|---|---|
| A Navigation | 3 thay đổi cấu hình có bằng chứng + sửa tích hợp monitor; SIM_READY | Đạt có điều kiện (Q6) |
| B Hạ tầng | Fast 2D dùng chính code robot, generator, evaluator, Gazebo runner | Đạt |
| C Pilot + dataset | Sửa công thức c, bảng 7.2 v3, 2.140 + 100 S1, không rò rỉ | Đạt; tương đồng sim↔Gazebo một phần (Q7) |
| D Baseline prep | Công cụ tối ưu, env SAC, đề xuất miền/observation/reward/ngân sách | Đạt (chưa chạy tối ưu / training) |

Bước 8 (Baseline) **sẵn sàng chạy** bằng `tune_baseline.py` trên `general_train` / `general_val`.

## A. Navigation (Giai đoạn A)

### A1 — Footprint, collision checking, sai số costmap

Fast 2D, pilot2 (393 scenario) × 3 seed = 1.179 episode / cấu hình:

| clearing | res | pad | SR % | CR % | STR % | hở min (m) |
|---|---|---|---:|---:|---:|---:|
| bật | 0.05 | 0.05 | 98.2 | **0.08** | 0.0 | −0.008 |
| tắt | 0.05 | 0.05 | 97.3 | 0 | 1.3 | 0.030 |
| bật | 0.025 | 0.05 | 99.8 | 0 | 0.0 | 0.011 |
| tắt | 0.025 | 0.05 | 98.5 | 0 | 1.1 | 0.026 |
| bật | 0.05 | 0.10 | 95.6 | 0 | 2.5 | 0.011 |
| tắt | 0.025 | 0.10 | 94.5 | 0 | 4.7 | 0.086 |

- Cơ chế va chạm (tái hiện, pilot0 `gen_B1_0014`): robot được phép tới tâm–ô lethal 0.245 m; mặt gần của hộp nằm trong
  footprint đã padding (~0.25–0.27 m) → Nav2 footprint clearing xóa → DWA thấy trống → đi vào (hở −0.055 m).
- Kết luận: **không tăng padding** (0.10 làm SR giảm 2.8–5.3 điểm); dùng ô 0.025 m; inscribed global = local (0.245).

### A2 — Mô hình motor, trễ

- `command_delay` thành launch arg s0–s3 (mặc định 0.15 s); trước đây chỉ đổi lúc chạy và không áp dụng đủ.
- Fast 2D: trễ 0 / 0.15 / 0.30 s → SR 98.5 / 97.8 / 94.5 %, CR 0 (316 scenario × 2 seed). Không mở rộng mô hình.

### A3 — Cà rốt, path blockage, Safe Stop, replan

- Logic replan tách sang `stage3_navigation/src/replan_supervisor.cpp` (ROS-free), bridge gọi lại; hành vi giữ
  nguyên, xác nhận bằng regression Gazebo (A9).
- Phát hiện: trigger "stall" chỉ kích hoạt khi robot đã sát vật cản (~0.11 m) → replan từ vị trí chật, đôi khi
  FollowPath abort liên tiếp (B2/C2 STUCK:ABORTED). Giữ nguyên (đúng thiết kế v2), ghi để tối ưu sau.
- Giới hạn NavFn: trích đường dừng sau 2·max(nx, ny) ô → detour dài trên map nhỏ báo "no path" (pilot0 B3
  FALSE_NO_PATH). Generator loại scenario vượt 80 % giới hạn.

### A4 — Chế độ quay

(316 scenario N2/B/C/S1 × 2 seed, cấu hình chốt.)

| `rotate_to_heading_angle` | SR % | STR % |
|---|---:|---:|
| 0.8 | 94.1 | 2.8 |
| 1.6 | 95.0 | 2.8 |
| **2.5** | **97.8** | 1.8 |

Không sửa (2.5 đang dùng là tốt nhất). Cơ chế cong vòng ~143° rồi xoay tại chỗ quan sát được nhưng không gây thất bại.

### A5 — Unknown với LiDAR 180°

(316 scenario × 2 seed, chạy trước khi tắt footprint clearing; ô 0.025 m, 2×2 m.)

| Cấu hình | SR % | CR % | Loại UNKNOWN |
|---|---:|---:|---:|
| U0 unknown = free (hiện tại) | 100 | 0 | 0 |
| U1 track unknown + is_obstacle | 98.9 | 0 | ~1e−5 |
| U2 U1 + `inf_is_valid` | 98.9 | 0 | ~1e−5 |

Đề xuất: giữ U0. Lý do: chỉ đi tiến, quỹ đạo ≤ 0.9 m phía trước nằm trong FOV đã raytrace; vùng bóng sau góc khuất được
kiểm tra bằng scenario B-hard (vật sau góc rẽ) và Gazebo R9 — không va chạm. `unknown_is_obstacle: true` giữ (vô hại).

### A6 — Obstacle critic, inflation, wO (footprint clearing tắt)

| inflation | wO 0 | wO 1 | wO 4 | wO 8 |
|---|---:|---:|---:|---:|
| 0.30 | 72.8 | 93.0 | **97.8** | 97.8 |
| 0.40 | 72.8 | 87.5 | 97.2 | 96.0 |
| 0.50 | 72.8 | 79.0 | 94.3 | 93.2 |

SR %; **CR = 0 trong cả 12 ô**. Với clearing bật: wO 0 → CR 7.95 %, wO 1 → 3.12 % (352 episode B/C) — an toàn khi đó
phụ thuộc trọng số mềm. ⇒ tắt clearing để loại cứng độc lập trọng số; giữ inflation 0.30.

### A7 — Local costmap và sim_time (SR %, CR 0 mọi ô, OFF_MAP 0)

| T_sim (s) | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 | 5.0 |
|---|---:|---:|---:|---:|---:|---:|
| 2×2 m | 93.2 | 97.8 | 97.4 | 96.7 | — | — |
| 4×4 m | 94.3 | 97.2 | 98.0 | 98.2 | 90.3 | 87.3 |

Compute p99 ≤ 11.1 ms (budget 100 ms). Chọn **4×4 m** cho M0/M1; miền T_sim SAC [1.5, 3.0] s (T ≥ 4 s làm B3 sụp:
42 % / 10 %).

### A8 — Planner vs Full-system, monitor UNCERTAIN

- Bằng chứng (Gazebo R4, monitor bật): `scan blocked by unmapped objects 0.878 > 0.85` rồi
  `scan-to-map consistency 0.283 < 0.3` → UNCERTAIN > 2 s → bridge hủy.
- Sửa tích hợp (bridge, không đụng AMCL/monitor): WARN chỉ do "scan blocked"/"consistency" với blocked_ratio ≥ 0.5 →
  grace 8 s thay vì 2 s. Kết quả Full-system: xem A9-b.
- Planner Benchmark = Fast 2D hoặc Gazebo `require_localized:=false`.

### A9 — Regression Gazebo (kho AWS, (0,0) → (0,−5.5), 3 lượt)

**a) Cấu hình v2 (local 2×2, 0.05, clearing bật), planner mode:**

| ID | Kịch bản | 3 lượt | Hở min (m) |
|---|---|---|---|
| R1 | không vật cản | S S S | — |
| R2 | hộp 0.3 | S S S | 0.102 |
| R3 | hộp 0.4×1.2 | S S S | 0.077 |
| R4 | tường 3 m (replan) | S S S | 0.080 |
| R5 | khe 0.65 | S S S | 0.102 |
| R6 / R6b | 3 hộp | S S S / S S S | 0.083 |
| R7 | S1 rào | F F F (đứng yên) | 0.051 |
| R8 | khe 0.50 (ngoài quy tắc) | A, x, S | **−0.0** |
| R9 | góc khuất (mới) | S x S | 0.137 |
| R10 | khe 0.60 (mới) | S S S | 0.071 |
| R11 | 5 hộp (mới) | S S S | 0.041 |
| R12 | LiDAR ngừng (mới) | dừng 2.02 / 2.02 / 2.12 s | — |

S = SUCCEEDED, F = FAILED (Safe Stop), A = ABORTED, x = lỗi hạ tầng test (reset sau R7 làm AMCL LOST, robot không
được định vị lại). R12: quãng trôi 0.52–0.54 m (ngân sách 0.5 + 0.5 + 0.15 + 0.5 ≈ 1.7 s + đo); tiêu chí 1.5 s ban
đầu đặt sai, sửa thành ≤ 2.2 s. R8 chạm ở khe 0.50 m — khe trong dải loại trừ, generator không sinh.

**b) Cấu hình chốt v3 (local 4×4, 0.025, clearing tắt, padding global 0.05), planner mode:**

| ID | 3 lượt | Hở min (m) |
|---|---|---|
| R1, R2, R4, R5, R6, R6b, R9, R10 | S S S | 0.076 (R10) |
| R3 hộp 0.4×1.2 | **A** S S | 0.057 |
| R7 S1 | F F A (đứng yên 0.0 mm khi F) | 0.061 |
| R8 khe 0.50 | A S S | 0.058 (v2: chạm) |
| R11 5 hộp | **A** S **A** | 0.060 |
| R12 LiDAR ngừng | dừng 2.02 / 2.02 / 1.92 s, trôi 0.47–0.51 m | — |

0 va chạm trong 39 episode. ABORTED ở R3/R11 = robot dừng hẳn khi ô tâm thành INSCRIBED (không lùi) — hệ quả của tắt
footprint clearing (Q6), khớp với Fast 2D (STUCK 1–2 %).

**c) Full-system (monitor + `require_localized`), R2/R4/R9/R11 × 2:**

| ID | 2 lượt | Monitor |
|---|---|---|
| R2 | S S | không WARN |
| R4 tường 3 m | **S S** (v2: CANCELED) | UNCERTAIN "scan blocked 0.881" → được gia hạn |
| R9 góc khuất | S S | không WARN |
| R11 5 hộp | S S | không WARN |

Sửa A8 có hiệu lực; LOST / AMCL std / status cũ vẫn hủy như trước.

### A — Kết luận

SIM_READY cho Planner Benchmark. Chưa REAL_READY (trễ/phanh/tải CPU chưa đo). Còn mở: Q6 (STUCK khi tắt clearing).

## B. Hạ tầng benchmark

| Yêu cầu | Thực hiện | Bằng chứng |
|---|---|---|
| B1 Fast 2D | viết mới (sim cũ `64df018` không có LiDAR/replan/map) theo cấu trúc cũ | `fast2d.cpp` |
| B2 Dùng lại DWA + replan | link `dwa_core`, `replan_supervisor`, `path_blockage`, NavFn, Costmap2D, `ZlacDriverModel` | CMake export targets |
| B3 Generator | rule-based (phòng, hành lang 0–4 góc, 2 tuyến) + BARN-like CA + map mục tiêu | `generator.py`, `layouts.py` |
| B4 Map ≠ World | Δ chỉ trong world; LiDAR raycast FOV ±90°, occlusion; costmap từ LiDAR | `fast2d.cpp` |
| B5 Checker, difficulty, manifest, seed, log | `generator.py`, `scenario_io.py` | dataset v3 |
| B6 Evaluator chung | `evaluator.py`; Safe Stop chỉ ở scenario vô nghiệm | §6 Master Plan v3 |

Kiểm tra Fast 2D: xác định (cùng seed → trace giống hệt); 159 episode / 32 s (pilot0, 10 lõi); env SAC 46 bước/s/lõi.

## C. Pilot, tương đồng, dataset

### C1–C3 Pilot

| Pilot | Nội dung | Phát hiện |
|---|---|---|
| pilot0 (159) | toàn pipeline | 1 va chạm (A1), B3 FALSE_NO_PATH (giới hạn NavFn) |
| pilot1 (294 × 2) | clearing bật / tắt | CR 0 / 0; STR 0.19 / 2.08 % |
| pilot2 (393) | độ khó v3, hint | 0 Hard ở v2 → sửa c + ngưỡng |

Công thức độ khó v3: c = bề rộng lối đi nhỏ nhất dọc A\*(W) (1.60 → 0, 0.60 → 1); ngưỡng 0.05 / 0.25. Kiểm chứng sơ bộ
(M0 hiện tại, General val): SR Easy / Medium / Hard = 99 / 96 / 96 % — **tương quan còn yếu**, xem lại sau baseline.

### C2 Fast 2D ↔ Gazebo (7 scenario × 3 bộ tham số; Fast 2D 5 seed)

Gazebo planner mode (AMCL thật, `require_localized:=false`), bộ tham số: base, timid_short (wP 1.5, wG 1.0, wO 1.0, wH 1.0,
wS 0.5, T 1.5), bold_long (0.3 / 3.0 / 7.0 / 0.2 / 2.5 / 3.0).

| Chỉ số | Giá trị |
|---|---|
| Episode hợp lệ | 20/21 (1 lỗi hạ tầng: scan mất > 2 s) |
| Kết cục bridge trùng (đa số 5 seed) | **17/20 = 85 %** |
| Sai lệch thời gian tới đích (trung vị) | 3.5 % |
| Sai lệch quãng đường | 3.7 % |
| Sai lệch độ dài A\* ban đầu | 0.5 % |
| Thời gian TB theo bộ (Gazebo / Fast 2D) | 34.2/35.3, 28.0/28.3, 37.2/38.9 s |
| Va chạm | 0 / 0 |

- 3 lệch đều ở bộ tham số biên (B1 timid, B3 timid: Gazebo FAILED, B3 bold), nơi Fast 2D cũng dao động giữa seed.
- Nhãn đầy đủ (gồm sai số đích GT) chỉ trùng 29 %: AMCL lệch 0.2–0.44 m tại đích trong layout sinh → evaluator Gazebo
  dùng `LOC_LIMITED`; so sánh planner dùng kết cục bridge.
- Kết luận G3: **đạt một phần** — Fast 2D xếp hạng tham số và thời gian/quãng đường đúng; cần thêm mẫu Gazebo (hoặc định
  vị GT) ở vùng tham số biên trước khi training lớn.

### C5–C6 Dataset v3

`~/capstone_ws/bench_data/v3` (8 MB, ngoài repo; tái lập bằng `gen_scenarios.py`, seed cố định; tóm tắt
`results/2026-11-26/dataset_v3_summary.json`).

| Split | n | Thiếu |
|---|---:|---|
| general train / val / test | 900 / 300 / 300 | 0 |
| general S1 val / test | 30 / 30 | 0 |
| classroom finetune / val / test | 120 / 100 / 100 | 0 |
| classroom S1 val / test | 10 / 10 | 0 |
| house finetune / val / test | 120 / 100 / 100 | 0 |
| house S1 val / test | 10 / 10 | 0 |
| **Tổng** | **2.140 + 100 S1** | |

Rò rỉ: 0 hash scenario / layout trùng giữa split. Sửa trong lúc sinh: (1) House test chỉ 14/100 vì khóa ô láng giềng
(start, goal) quá chặt cho nhà nhỏ → House khóa đúng ô 1 m; (2) S1 rào quanh đích bị luật "cách goal ≥ 1 m" loại →
miễn cho S1; (3) Classroom: robot vòng ra ngoài biên map ở một S1 → luật `border_leak`, sinh lại toàn bộ Classroom.

## D. Chuẩn bị Baseline + SAC

- `scripts/tune_baseline.py`: LHS 6 chiều + successive halving (rung 60 → 200 → toàn bộ train), chọn top-k trên
  validation, loại CR > 2 %, mục tiêu SR cân bằng type. `scripts/eval_split.py` khóa `*_test`.
- Mốc M0 **hiện tại** (chưa tối ưu, Fast 2D, seed 0):

| Tập | n | SR % | CR % | SSR % | STR % |
|---|---:|---:|---:|---:|---:|
| general_val | 300 | 97.0 | 0 | — | 2.7 |
| general_s1_val | 30 | — | 0 | 100 | — |
| classroom_val (sinh lại) | 100 | 100 | 0 | — | 0 |
| classroom_s1_val (sinh lại) | 10 | — | 0 | 100 | — |
| house_val | 100 | 96.0 | 0 | — | 3.0 |
| house_s1_val | 10 | — | 0 | 80 | — |

Failure mode chính: B1/B2/C2 `STUCK:ABORTED` (kẹt ô INSCRIBED / abort sau replan muộn), S1 House TIMEOUT (đi vòng
rào). Test split chưa chạy.

- `python/stage5_benchmark/env.py`: env kiểu Gymnasium (obs 62, action 6), random policy 20/20 thành công (an toàn
  độc lập action). Đề xuất miền/observation/reward/ngân sách: Master Plan v3 §9.

## Lỗi / vấn đề còn tồn tại

| # | Vấn đề | Ảnh hưởng | Đề xuất |
|---|---|---|---|
| Q6 | Tắt footprint clearing → kẹt ở ô INSCRIBED (không lùi) | STUCK 1–2 % sim; Gazebo R3/R11 | bạn chọn: chấp nhận / thêm thoát kẹt vào DwaCore (thay đổi thuật toán) / bật lại clearing + chặn wO ≥ 2 |
| Q7 | AMCL lệch 0.2–0.4 m trong layout sinh | Gazebo SUCCESS phải tách `LOC_LIMITED` | định vị GT trong Gazebo cho G3 |
| Q8 | S1 rào quanh đích: TIMEOUT | SSR 80–98 % | giữ, xem lại T_max S1 |
| Q9 | Độ khó tương quan yếu với thất bại | curriculum | hiệu chỉnh lại sau baseline |
| Q10 | Stall-trigger replan muộn (robot đã sát vật) | B2/C2 abort | để tối ưu sau (ngoài phạm vi weights) |
| Q2 | `control_latency` 0.2 s đo ở ramp 200 ms | REAL_READY | đo trên robot |
| — | Nav2 DWB chưa thử với 4×4 | so sánh phụ | chạy lại nếu cần |
| — | Chưa commit, chưa chạy robot | — | chờ bạn |
