# SAC–DWA — RESEARCH & TRAINING MASTER PLAN

**Phiên bản 3.0 (2026-11-26) | Đồ án tốt nghiệp — Navigation System**
**Repository:** ppthanh2810/capstone_project (thay đổi v3 **chưa commit**)
**Phạm vi:** điều hướng robot vi sai trong môi trường tĩnh, biết trước một phần; SAC chỉnh thích nghi 5 trọng số DWA
và thời gian dự đoán. Không nghiên cứu động lực học, không nghiên cứu SLAM/localization.

Báo cáo nghiệm thu chi tiết (bằng chứng, lệnh chạy, số liệu từng thí nghiệm): `stage5_benchmark/docs/ACCEPTANCE_2026-11-26.md`.

## Thay đổi so với v2

- Giai đoạn A: Navigation mô phỏng **SIM_READY** (chưa REAL_READY). Ba thay đổi cấu hình có bằng chứng: inscribed global
  = local (padding 0.05), local costmap **0.025 m, 4×4 m, footprint clearing tắt**. Bridge: gia hạn grace khi LiDAR
  bị vật cản chưa có trong map che (A8). Logic replan tách thành thư viện ROS-free `ReplanSupervisor`.
- Giai đoạn B: package mới `stage5_benchmark` — Fast 2D dùng chính `DwaCore` + `ReplanSupervisor` + NavFn + Costmap2D +
  `ZlacDriverModel`; Scenario Generator, Feasibility/Difficulty, Evaluator, manifest, runner Gazebo cùng scenario.
- Giai đoạn C: pilot, sửa công thức độ khó (thành phần c), điều chỉnh bảng 7.2 theo ô sinh được, sinh **2.140 scenario
  cố định + 100 S1 riêng**, kiểm tra rò rỉ.
- Giai đoạn D: công cụ tối ưu DWA cố định + môi trường SAC kiểu Gymnasium (chưa train), đề xuất miền action /
  observation / reward / ngân sách từ pilot.
- Không đổi M0/M1, H1–H6, BM1–BM5, 70/30, curriculum 80/20/0 → 30/50/20 → 33/34/33, CR ≤ 2 %.

Nhãn nguồn: **[ĐO]** đo (sim/robot, có log), **[CFG]** cấu hình, **[DX]** dẫn xuất, **[ĐX]** đề xuất, **[TBD]** chưa có.

## A. Bảng theo dõi tiến độ 1–10

| Bước | Trạng thái v3 |
|---:|---|
| 1 Problem | Đã chốt (không đổi) |
| 2 Hypothesis | Đã chốt (không đổi) |
| 3 System Characterization | **SIM_READY**: đóng băng cho mô phỏng; REAL_READY chờ đo trễ/phanh trên robot |
| 4 Scenario Taxonomy | Đã triển khai, thông số chốt cho mô phỏng (§4) |
| 5 Scenario Generation | Đã triển khai + pilot (§5) |
| 6 Metrics/Protocol | Evaluator chung Fast 2D/Gazebo đã triển khai (§6) |
| 7 Dataset | **Đã sinh**: 2.140 cố định + 100 S1, không rò rỉ (§7) |
| 8 Baseline | Công cụ sẵn sàng, mốc M0 hiện tại trên validation; **chưa tối ưu** (§8) |
| 9 Training | Env + đề xuất thiết kế; **chưa training** (chờ bạn xem baseline) |
| 10 Real-world | Chưa |

---

## 1–2. Problem Definition, Research Hypothesis

Giữ nguyên v2 (M0 Optimized DWA, M1 SAC-DWA; action `[wP, wG, wO, wH, wS, T_sim]`, oscillation cố định; H1, H2, H3, H5,
H6; N/B/C/S1). Định nghĩa critic theo code (`DwaCore::compute`): P = `path_distance_scale`, G = `carrot_distance_scale`
(khoảng cách tới **cà rốt**), O = `obstacle_scale`, H = `heading_scale`, S = `speed_scale`, T_sim = `sim_time`.

## 3. System Characterization — Giai đoạn A

### 3.1 Cấu hình chốt cho mô phỏng (SIM_READY)

| Thông số | Giá trị | Nguồn |
|---|---|---|
| Footprint | R 0.20 + padding 0.05 (global = local) | [CFG] A1 |
| Inscribed (Nav2, đa giác 16) | 0.245 m | [DX] |
| Local costmap | 4×4 m, **0.025 m**, rolling | [CFG] A1/A7 |
| Local footprint clearing | **tắt** | [CFG] A1/A6 |
| Inflation local / global | 0.30 / 0.40 m | [CFG] A6 |
| Unknown local | `track_unknown_space: false` | [CFG] A5 |
| `rotate_to_heading_angle` | 2.5 rad (giữ) | [ĐO] A4 |
| v / ω max | 0.3 m/s / 1.0 rad/s | [CFG] |
| Ramp driver | 500 ms | [CFG] A2 |
| Trễ lệnh sim | `command_delay` 0.15 s (+~0.08 nội tại) | [CFG] A2 |
| `control_latency` DWA | 0.2 s | [ĐO 2026-10-14] |
| Replan | stall 4 s, cooldown 2 s, 10 + 3 lần, chờ 15 s | [CFG] |
| Bridge (full-system) | grace 2 s; che khuất ≥ 0.5 → 8 s | [CFG] A8 |

Kiểm nhất quán: `stage4_gazebo/test/config_consistency.py` → 0 FAIL, 1 WARN (trễ đo ở ramp 200 ms, cần đo lại trên
robot). Khe tối thiểu A\* = DWB ≈ 0.53 m [DX].

### 3.2 Kết quả từng hạng mục A1–A9 (tóm tắt; chi tiết ở báo cáo nghiệm thu)

| # | Kết luận | Bằng chứng |
|---|---|---|
| A1 | Sai số rời rạc 0.05 m + footprint clearing làm mất biên 0.05 m; sửa bằng ô 0.025 + tắt clearing, **không** tăng padding | Fast 2D 1.179 ep/cấu hình: clearing+0.05 → 1 va chạm (hở −0.008); padding 0.10 → SR −2.8…−5.3 điểm |
| A2 | Giữ mô hình đơn giản; delay 0.15 s chạy đúng qua launch | SR 98.5 / 97.8 / 94.5 % ở trễ 0 / 0.15 / 0.30 s |
| A3 | Cà rốt, path blockage, Safe Stop, replan giữ nguyên; logic replan tách thư viện, Gazebo xác nhận hành vi không đổi | Regression Gazebo (§3.3) |
| A4 | `rotate_to_heading_angle` 2.5 tốt nhất | SR 94.1 / 95.0 / 97.8 % ở 0.8 / 1.6 / 2.5 |
| A5 | Theo dõi unknown không có lợi với LiDAR 180° + chỉ đi tiến | SR 100 / 98.9 / 98.9 % (U0 / U1 / U2), CR 0 cả ba |
| A6 | Tắt clearing → loại cứng độc lập trọng số; inflation 0.30 tốt nhất | CR 0 với mọi (r, wO); clearing bật: wO 0 → CR 7.95 % |
| A7 | 4×4 m cho cả M0/M1 | 4×4 T 2.0–3.0 s: SR 97.2–98.2 %, OFF_MAP 0 |
| A8 | Planner Benchmark tách khỏi monitor; Full-system giữ monitor + sửa grace che khuất | R4 Gazebo: blocked 0.88, consistency 0.28 → hủy (v2) |
| A9 | Regression ×3 + kịch bản mới (góc khuất, khe 0.60, 5 vật, stale LiDAR) | §3.3 |

`unknown_is_obstacle: true` vẫn không có tác dụng khi `track_unknown_space: false`: giữ nguyên (vô hại, plugin cảnh
báo lúc activate); bảo vệ thực tế là chỉ đi tiến + raytrace ±90° + horizon ≤ 0.9 m phía trước.

### 3.3 Regression Gazebo

Kho AWS, 13 kịch bản (R9 góc khuất, R10 khe 0.60, R11 5 hộp, R12 LiDAR ngừng là mới), 3 lượt mỗi kịch bản:

| Cấu hình | Kết quả |
|---|---|
| v2, planner | 0 va chạm có quy tắc; R8 (khe 0.50, ngoài quy tắc) chạm 1 lần |
| v3 chốt, planner (39 ep) | **0 va chạm**, hở min 0.057 m; R3 2/3, R11 1/3 (ABORTED, Q6); còn lại 3/3; R7 Safe Stop |
| v3 chốt, full-system (8 ep) | 8/8 SUCCEEDED; R4 không còn bị hủy (A8) |
| R12 LiDAR ngừng | dừng 1.9–2.1 s, trôi 0.47–0.54 m, không chạm |

### 3.4 SIM_READY vs REAL_READY

| Mức | Điều kiện | Trạng thái |
|---|---|---|
| SIM_READY | Cấu hình chốt, regression sim ×3, Fast 2D CR 0 | **Đạt có điều kiện**: CR 0 ở Fast 2D và Gazebo; STUCK do Q6 cần bạn quyết |
| REAL_READY | Đo trễ lệnh→odom ở ramp 500 ms, quãng phanh 0.3 m/s, tải CPU costmap 0.025 m trên laptop, Full-system trên robot | Chưa |

Lệnh đo trên robot (bạn tự chạy, **kê bánh / E-stop**) giữ như v2 §9.

## 4. Scenario Taxonomy

N1–N3, B1–B3, C1–C4 + S1 riêng; nhãn known/unknown cho C. Thông số sinh (`stage5_benchmark/config/benchmark.yaml` `gen`):

| Thông số | Giá trị |
|---|---|
| Phòng / hành lang | 4–16 m / 0.8–2.5 m |
| Khe khả thi / vô nghiệm | ≥ 0.60 / ≤ 0.40 m (0.40–0.60 không sinh) |
| Vật cản mới | cao 0.6 m, cách start ≥ 3.0 m, cách goal ≥ 1.0 m |
| B1 / B2 | 0.2–0.4 / 0.6–1.0 m |
| B3 | chặn hành lang ±1 m, detour ≤ 3.0 |
| C1 / C2 / C3 | khe 0.60–0.80 / 3–8 vật / xen kẽ 1.0–2.0 m |
| C4 | CA ô 0.15 m, fill 0.10–0.35, 4 vòng |
| L_ref | 3–15 m |

### 4.4 Độ khó (v3, sửa theo pilot)

`Dscore = 0.30c + 0.20ρ + 0.15τ + 0.20δ + 0.15ν` (giữ trọng số v2). Đổi:

- **c** = bề rộng lối đi nhỏ nhất dọc đường A\* trên W (tia vuông góc trái + phải), ≥ 1.60 m → 0, 0.60 m → 1.
  Lý do: v2 dùng "khe rộng nhất toàn tuyến" → B1/C2/C3 có c ≈ 0 (pilot1: 0/265 Hard, Dscore max 0.52).
- Ngưỡng **Easy < 0.05 ≤ Medium < 0.25 ≤ Hard** (≈ phân vị 1/3 và 2/3 của pilot2, 349 scenario).
- Còn kiểm chứng: độ khó phải tương quan với thất bại của baseline (bước 8).

Curriculum (giữ): 80/20/0 → 30/50/20 → 33/34/33.

## 5. Scenario Generation

Hybrid 70 % offline / 30 % online (giữ). `W = M ⊕ Δ`; LiDAR chỉ thấy W qua raycast; Δ không bao giờ vào DWA/A\*/SAC.

Kiểm tra (generator.py): khả thi (W nở 0.30 còn nối), vô nghiệm (W nở 0.20 mất nối), loại vùng mơ hồ; đường tắt qua
khe 0.40–0.60; khe quanh Δ; khoảng cách start/goal; B1/B2 né được trong ±1 m, B3 phải replan; giới hạn độ dài đường
của NavFn (2·max(nx, ny) ô); **rò ra ngoài biên map** (Classroom có sàn trống chạm mép lưới).

Fast 2D (`stage5_benchmark/src/fast2d.cpp`): dùng chính DwaCore, ReplanSupervisor, NavFn, Costmap2D, ZlacDriverModel;
LiDAR 720 tia ±90°, ObstacleLayer/InflationLayer theo Nav2; định vị hoàn hảo; ~25–50× thời gian thực mỗi lõi;
xác định theo seed (seed ≠ 0 thêm nhiễu pose xuất phát 0.03 m / 0.05 rad).

## 6. Metrics and Protocol

Năm chỉ số chính giữ nguyên. Evaluator chung (`evaluator.py`):

| Nhãn | Điều kiện |
|---|---|
| SYSTEM_ERROR | lỗi hạ tầng (exception sim, GT/scan mất > 2 s, RTF < 0.5 > 10 s) |
| COLLISION | hở GT < −0.005 m |
| SUCCESS | SUCCEEDED ∧ GT ≤ 0.15 (2D) / 0.25 m (Gazebo) ∧ yaw ≤ 0.27; Gazebo: lệch giải thích được bằng AMCL → SUCCESS `LOC_LIMITED` |
| SAFE_STOP | chỉ scenario vô nghiệm: FAILED/ABORTED + đứng yên ≥ 2 s |
| STUCK | khả thi: FAILED (`FALSE_NO_PATH`), ABORTED, không tiến 30 s |
| TIMEOUT | t > 30 + 3·L/0.3 |

**Planner Benchmark** = Fast 2D (định vị hoàn hảo) hoặc Gazebo `require_localized:=false`; **Full-system Benchmark** =
Gazebo với AMCL + CSM + monitor + `require_localized` (và robot thật). Episode ở Gazebo tự phân biệt lỗi hạ tầng.

## 7. Dataset (đã sinh, `~/capstone_ws/bench_data/v3`, ngoài repo, 8 MB)

| Split | Số | Ghi chú |
|---|---:|---|
| General train / val / test | 900 / 300 / 300 | val/test 100 mỗi mức |
| Classroom finetune / val / test | 120 / 100 / 100 | 9 type (không C4) |
| House finetune / val / test | 120 / 100 / 100 | 9 type |
| S1 general val / test | 30 / 30 | |
| S1 Classroom, House | 10 / 10 mỗi map | |

Bảng 7.2 v3 (General train, điều chỉnh vì ô không sinh được): N1 60/30/0, N2 0/60/30, N3 30/30/30, B1 40/30/20,
B2 30/30/30, B3 0/45/45, C1 0/0/90, C2 30/30/30, C3 40/30/20, C4 30/30/30 (Easy/Medium/Hard; tổng 900).
Lý do: B3 luôn có detour + replan (không Easy), C1 khe 0.60–0.80 luôn Hard, N2 layout hành lang có góc không Easy,
C3-Hard hiếm. Chống rò rỉ: seed namespace mỗi split, hash scenario/layout không trùng giữa split, map mục tiêu khóa ô
(start, goal) 1 m (Classroom cả ô láng giềng). Warehouse/Office chỉ dùng regression.

## 8. Baseline Evaluation (chuẩn bị)

- Công cụ: `stage5_benchmark/scripts/tune_baseline.py` (LHS + successive halving, chọn bằng validation, ràng buộc CR ≤ 2 %,
  mục tiêu SR cân bằng theo type), `eval_split.py` (khóa `*_test`).
- Mốc **DWA hiện tại (chưa tối ưu)** trên validation, Fast 2D: General SR 97.0 % / CR 0 / STR 2.7 %; Classroom 100 %;
  House 96.0 %; SSR S1: 100 / 100 / 80 %. Test chưa chạy.
- Fast 2D ↔ Gazebo (G3): kết cục bridge trùng 17/20 (85 %), thời gian/quãng đường lệch ~3.5 %, cùng thứ hạng bộ tham
  số; lệch ở vùng tham số biên → **G3 đạt một phần**.
- Chưa chạy tối ưu chính thức.

## 9. SAC-DWA — đề xuất thiết kế (CHƯA TRAINING)

| Thành phần | Đề xuất [ĐX] |
|---|---|
| Action | wP [0, 2], wG [0.5, 4], wO [0, 8], wH [0, 1.5], wS [0, 3], T_sim [1.5, 3.0] s |
| Chu kỳ quyết định | 0.5 s (5 chu kỳ DWA) |
| Observation (62) | 36 sector LiDAR ±90° (≤ 3 m), cà rốt trong frame robot, dist goal, heading error, carrot shift, tiến trình, v/ω đo + lệnh, pha DWA, action trước, cờ chặn / cà rốt bị chặn / tỷ lệ quỹ đạo hợp lệ |
| Reward | +10·Δtiến trình, −0.05/bước, +20 SUCCESS, −30 COLLISION, −15 STUCK, phạt hở < 0.10 m (GT, chỉ reward), −0.02·‖Δa‖² |
| An toàn | loại cứng trong DwaCore độc lập action (A6: CR 0 với wO 0); `validateParams` từ chối action lỗi |
| Throughput | ~46 bước quyết định/s/lõi (có tải nền) → 1 M bước ≈ 6 h·lõi |
| Ngân sách khởi đầu | 0.5–1 M bước, 3 seed, 8 env song song; validation mỗi 25 k bước (chốt sau baseline) |

Lưu ý thiết kế từ pilot: (1) wO thấp → STUCK tăng mạnh (wO 0: SR 72.8 %), nên miền wO giữ [0, 8] — SAC sẽ bị phạt qua
STUCK chứ không qua va chạm; (2) T_sim ≥ 4 s làm B3 sụp, miền 1.5–3.0 hợp lý; (3) seed Fast 2D chỉ đổi nhiễu nhỏ, cần
online generator để đa dạng.

## 10. Real-world Validation

Giữ v2. Trước BM4: đo trễ/phanh, tải CPU costmap 0.025 m, chạy Full-system trên robot (kê bánh → sàn).

---

## D. Vấn đề còn mở

| # | Vấn đề | Đề xuất |
|---|---|---|
| Q2 | `control_latency` đo ở ramp 200 ms | đo lại trên robot |
| Q6 | Tắt footprint clearing → robot có thể dừng hẳn ở ô INSCRIBED (STUCK ~1–2 %), không lùi | cần bạn quyết: chấp nhận, hoặc thêm thoát kẹt (xoay / đi ra khi chi phí giảm) vào DwaCore — thay đổi thuật toán |
| Q7 | Gazebo: AMCL lệch 0.2–0.4 m trong layout sinh ra (ít đặc trưng) | Planner Benchmark Gazebo dùng trạng thái bridge; cân nhắc định vị GT trong Gazebo cho G3 |
| Q8 | S1 rào quanh đích: robot đi vòng rào tới hết T_max (SSR 80–98 %) | giữ (hành vi thật); xem lại T_max cho S1 |
| Q9 | Nav2 DWB (FollowPath, so sánh) chưa thử với local 4×4 | chạy lại nếu còn dùng |
| — | Thay đổi v3 chưa commit, chưa chạy robot | chờ bạn |
