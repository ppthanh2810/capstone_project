# SAC–DWA — RESEARCH & TRAINING MASTER PLAN

**Phiên bản 5.0 (2026-11-27) | Đồ án tốt nghiệp — Navigation System**
**Repository:** ppthanh2810/capstone_project (thay đổi v3–v5 **chưa commit**)
**Phạm vi:** điều hướng robot vi sai trong môi trường tĩnh, biết trước một phần; SAC chỉnh thích nghi 5 trọng số DWA
và thời gian dự đoán. Không nghiên cứu động lực học, không nghiên cứu SLAM/localization.

Báo cáo nghiệm thu: `stage5_benchmark/docs/ACCEPTANCE_2026-11-26.md` (v3/V4) và
**`stage5_benchmark/docs/ACCEPTANCE_2026-11-27.md` (Replanning V5)**. Bản v3 giữ nguyên để đối chiếu; không có file
"v4" riêng (V4 chỉ là thay đổi replanning, ghi trong báo cáo 2026-11-26).

## Thay đổi so với v3 / V4

- **Replanning V5 "Local Avoidance First — Global Replanning Second"** thay cho V3 (stall 4 s) và V4 (replan sớm khi
  đường bị chặn trong 1 m). V4 bị loại vì nó để A\* né thay DWA: Local Avoidance Rate (LAR) 0 %, SR 98.9–100 % với
  mọi bộ trọng số (kể cả `obstacle_scale` 0), tức xóa luôn bài toán của SAC.
- Đường bị chặn **chỉ đưa vào PATH_BLOCKED**. Chỉ lập lại khi có **bằng chứng DWA hết cách** ≥ 0.4 s, hoặc không có
  tiến triển hữu ích 4 s. Tiếp cận chậm 0.15 m/s khi ô bị chặn cách ≤ 1.2 m (`/speed_limit`).
- `DwaCore`: thêm `probeFreeSpace` (21 cung 0.6 m, không phụ thuộc trọng số / `sim_time`). Tầm dò vật cản dùng
  `max_vel_x` danh định, không bị giới hạn tốc độ làm ngắn lại. Loại cứng và dừng an toàn **không đổi**.
- Sensitivity / Oracle Gap tính lại **đúng định nghĩa** (cùng dữ liệu, cùng seed; gap ≥ 0 theo cấu trúc). Bản cũ "cross"
  (chọn trên nửa A, đo trên nửa B) cho gap âm và chỉ là cận dưới.
- G3 Fast 2D ↔ Gazebo làm lại với định vị GT (`planner_gt`): 26 scenario × 5 bộ.
- **Không đổi** M0/M1, H1–H6, BM1–BM5, dataset v3, 70/30, curriculum, CR ≤ 2 %, cấu hình SIM_READY của v3.

Nhãn nguồn: **[ĐO]** đo (sim/robot, có log), **[CFG]** cấu hình, **[DX]** dẫn xuất, **[ĐX]** đề xuất, **[TBD]** chưa có.

## A. Bảng theo dõi tiến độ 1–10

| Bước | Trạng thái v5 |
|---:|---|
| 1 Problem | Đã chốt (không đổi) |
| 2 Hypothesis | Đã chốt (không đổi) |
| 3 System Characterization | SIM_READY; **Replanning V5 (R2s) được chọn, chờ bạn nghiệm thu** |
| 4 Scenario Taxonomy | Không đổi (v3 §4) |
| 5 Scenario Generation | Không đổi; thêm split `stress_pilot` (ngoài dataset v3, chỉ dùng chẩn đoán) |
| 6 Metrics/Protocol | Thêm LAR, trạng thái replan, nguyên nhân kích hoạt (§6) |
| 7 Dataset | Không đổi: 2.140 + 100 S1, `*_test` vẫn khóa |
| 8 Baseline | **Tạm dừng** đến khi V5 được nghiệm thu; Sensitivity / Oracle / Ceiling đã chạy lại (§8) |
| 9 Training | **Tạm dừng** (chưa training) |
| 10 Real-world | Chưa |

---

## 1–2. Problem Definition, Research Hypothesis

Giữ nguyên v2/v3 (M0 Optimized DWA, M1 SAC-DWA; action `[wP, wG, wO, wH, wS, T_sim]`; P = `path_distance_scale`,
G = `carrot_distance_scale`, O = `obstacle_scale`, H = `heading_scale`, S = `speed_scale`, T_sim = `sim_time`).
**M0 và M1 dùng cùng một cơ chế replanning (V5)**, cùng `ReplanSupervisor` và `nav2_params.yaml`.

## 3. System Characterization — Giai đoạn A

### 3.1 Cấu hình chốt cho mô phỏng (SIM_READY)

| Thông số | Giá trị | Nguồn |
|---|---|---|
| Footprint | R 0.20 + padding 0.05 (global = local) | [CFG] A1 |
| Local costmap | 4×4 m, 0.025 m, rolling, footprint clearing **tắt** | [CFG] A1/A6/A7 |
| Inflation local / global | 0.30 / 0.40 m | [CFG] A6 |
| `rotate_to_heading_angle` | 2.5 rad | [ĐO] A4 |
| v / ω max | 0.3 m/s / 1.0 rad/s | [CFG] |
| Mô hình động cơ | ramp 500 ms, `command_delay` 0.15 s, không lùi | [CFG] A2 |
| **Replan** | **V5 R2s** (§3.5); cooldown 2 s, 10 + 3 lần, chờ 15 s (giữ) | [CFG] [ĐO] |

### 3.2–3.4

Giữ v3 (A1–A9, SIM_READY vs REAL_READY). Regression Gazebo với V5: §3.5.

### 3.5 Replanning V5 (R2s) — tóm tắt

| Trạng thái | Khi nào | Hành động |
|---|---|---|
| NORMAL | đường A\* thông | DWA bám đường |
| PATH_BLOCKED | ô chặn trên đường (xác nhận) | **không replan**; ≤ 1.2 m → giới hạn 0.15 m/s |
| LOCAL_BLOCKED | bằng chứng DWA ≥ 0.4 s, hoặc 4 s không tiến | yêu cầu A\* |
| GLOBAL_BLOCKED | đã gửi replan | chờ đường mới; cooldown 2 s |
| SAFE_STOP | hết lượt / A\* không có đường | dừng, FAILED |

Bằng chứng DWA (chu kỳ bám đường): DWA BLOCKED; 0 quỹ đạo tiến hợp lệ; tỷ lệ cung tự do < 0.15. Xoay tại chỗ là
trung lập (không phải bằng chứng đang đi tiếp). Không dùng khoảng cách tới goal tăng hay độ lệch đường. Tiến triển
hữu ích = 0.15 m dọc đường **hoặc 0.3 m dịch chuyển** (đi vòng vẫn tính).

Vì sao không phụ thuộc `sim_time`: tỷ lệ quỹ đạo hợp lệ giảm mạnh khi T_sim dài (1.00 / 0.97 / 0.72 ở T 1.5 / 2.0 /
3.0 s, cùng vị trí hình học), còn tỷ lệ cung tự do gần như không đổi (0.80 / 0.80 / 0.82) [ĐO]. Nên SAC không thể
kích hoạt replan bằng cách kéo dài horizon.

| Bằng chứng | Kết quả |
|---|---|
| Fast 2D dev 550 (mặc định) | R2s SR 99.2 %, STR 0.4, CR 0, replan 16.6 % ep, LAR 57.7 % (V4: 100 / 0 / 0 / 38.6 / **0**) |
| Dev 8 bộ tham số | CR 0 mọi cơ chế × mọi bộ; R2s SR 65.8–99.2 % (vẫn nhạy trọng số) |
| Gazebo regression ×3 | 35/39, **0 va chạm**, hở min 0.059 m; R12 FAIL có từ trước; R7 1/3 ABORTED |
| gtest | `test_replan_supervisor` 12/12 PASS |

## 4–5. Scenario Taxonomy, Generation

Giữ v3 §4–§5. Thêm split chẩn đoán `stress_pilot` (`~/capstone_ws/bench_data`, sinh bằng `gen_stress.py`; không thuộc
dataset v3, không dùng để chọn mô hình cuối).

## 6. Metrics and Protocol

Giữ v3 §6 (SR, CR, STR, SSR, thời gian, quãng đường, hở min; nhãn evaluator). Thêm cho replanning:

| Chỉ số | Định nghĩa |
|---|---|
| Replan rate | % episode có ≥ 1 replan; số replan / episode |
| **LAR** | % đoạn tương tác (Δ trong 0.25 m của A\* ban đầu) robot vượt qua **không** replan |
| Nguyên nhân | `no_free_space`, `no_forward`, `dwa_blocked`, `no_progress`, … (log bridge + Fast 2D) |
| Trạng thái lúc kích hoạt | v, tỷ lệ cung tự do, khoảng cách ô chặn |

## 7. Dataset

Giữ v3 §7. `*_test` khóa; mọi lựa chọn V5 dùng `general_train` (ngưỡng) và các split `*_val` (so sánh).

## 8. Baseline Evaluation — Sensitivity / Oracle / Ceiling (V5)

Fast 2D, 30 bộ tham số (hiện tại + 12 one-at-a-time lo/hi + 17 LHS), 2 seed. Main = `general_train` 125 scenario (lấy mẫu
phân tầng), stress = `stress_pilot` 120 scenario.

**Oracle Gap đúng:** J trung bình qua seed; oracle = trung bình theo scenario của max theo bộ tham số; best fixed =
max theo bộ tham số của trung bình (ràng buộc CR ≤ 2 %); gap = oracle − best fixed ≥ 0. Noise floor = gap của cùng
phép tính khi chỉ khác seed.

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

So sánh V4 (R1) trên **cùng 125 scenario, cùng 30 bộ, cùng seed**: §8.1.

### 8.1 V5 so với V4 (cùng dữ liệu)

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

### 8.2 Diễn giải cho SAC

- Trọng số **vẫn quyết định kết cục** dưới V5: một bộ tệ (`obstacle_scale` thấp, `speed_scale` thấp) làm SR main còn
  82.8 % và gấp đôi số replan (17 → 36 % ep). Nhóm nhạy nhất: B1, B2 (SR min 47 / 53 % main, 40 / 20 % stress), C2
  stress (27 %). N1–N3, C3 gần như không nhạy.
- Nhưng **một bộ cố định tốt đã gần trần**: gap SR 0 (main) / 2.5 điểm (stress), gap q nhỏ hơn noise floor ở cả hai.
  Với 2 seed, lợi ích của việc chọn tham số theo từng scenario chưa tách được khỏi nhiễu.
- Hệ quả: H1 (SAC > M0 tối ưu) có khoảng trống **nhỏ** trên phân bố chung. Nơi có khoảng trống là tập khó (stress,
  B1/B2/C2 Hard) và chỉ số q (thời gian, quãng đường, hở), không phải SR. Cần thêm seed trước khi kết luận.

## 9. SAC-DWA — đề xuất thiết kế (CHƯA TRAINING)

Giữ bảng v3 §9 (action, chu kỳ 0.5 s, reward, an toàn, ngân sách). Bổ sung từ V5:

| Điểm | Đề xuất [ĐX] |
|---|---|
| Observation | thêm tỷ lệ cung tự do (`fs_forward_free/fs_arcs`), hở `fs_clearance`, trạng thái replan (one-hot 5), khoảng cách ô chặn |
| Replan trong env | dùng đúng R2s (cùng `ReplanSupervisor`); SAC **không** điều khiển replan |
| Chống khai thác | bằng chứng replan không phụ thuộc T_sim / trọng số (§3.5) → SAC không thể "mua" replan bằng horizon |
| Reward | có thể thêm phạt nhỏ mỗi replan (−1) để SAC ưu tiên tự né, chỉ khi baseline cho thấy cần |
| Tập huấn luyện | ưu tiên B1/B2/C2 Hard và stress (nơi có khoảng trống, §8.2) |

## 10. Real-world Validation

Giữ v2/v3. Bổ sung: đo tải CPU của `probeFreeSpace` (6.400 ô + 21 cung / chu kỳ) trên laptop robot; kiểm `/speed_limit`
tới `controller_server` trên robot.

## G3. Fast 2D ↔ Gazebo (V5, planner_gt)

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

So với G3 v3 (17/20 = 85 %, AMCL): **G3 đạt** cho Planner Benchmark. Fast 2D dùng được để chọn tham số và huấn luyện,
Gazebo dùng để xác nhận.

---

## D. Vấn đề còn mở

| # | Vấn đề | Đề xuất |
|---|---|---|
| Q2 | `control_latency` đo ở ramp 200 ms | đo lại trên robot |
| Q6 | STUCK còn lại = ô tâm thành INSCRIBED, không lùi | giữ (ràng buộc); SAC có thể giảm bằng wO / T_sim |
| Q7 | AMCL lệch 0.2–0.4 m trong layout sinh ra | G3 dùng định vị GT (`planner_gt`) |
| Q8 | S1: vòng quanh rào tới hết T_max (R2s SSR 96 %) | xem lại T_max cho S1 |
| Q10 | Tiếp cận chậm đôi khi đổi hướng né của DWA | theo dõi ở baseline / SAC |
| Q11 | Regression R7: 1/3 ABORTED (đứng yên) thay vì FAILED | chấp nhận như Safe Stop hoặc nới tiêu chí — cần bạn quyết |
| Q12 | R12 (LiDAR ngừng) dừng 1.8–2.1 s > 1.5 s | có từ trước, ngoài phạm vi replan |
| Q13 | Gazebo: một số scenario sinh ra không khởi động được (bridge không có trạng thái 15 s / REJECTED) | xem §G3; lỗi hạ tầng, không phải điều hướng |
| Q14 | G3: S1 Gazebo replan 8 lần so với 1 ở Fast 2D (cùng SAFE_STOP) | xem log bridge trước khi dùng Gazebo đo replan rate S1 |
| — | V5 chưa commit, chưa chạy robot; baseline / SAC chờ bạn nghiệm thu V5 | chờ bạn |
