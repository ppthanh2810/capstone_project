# stage5_benchmark — Hạ tầng benchmark SAC-DWA (chỉ mô phỏng)

Benchmark cho Master Plan v3 (`SAC_DWA_Research_Training_Master_Plan_v3.md`): Fast 2D simulator, Scenario Generator,
Feasibility/Difficulty, Episode Evaluator, chạy Gazebo cùng scenario, pilot, sinh dataset, công cụ baseline.
**Không chạm phần cứng.** Báo cáo nghiệm thu: `docs/ACCEPTANCE_2026-11-26.md` (v3) và
`docs/ACCEPTANCE_2026-11-27.md` (V5: cơ chế Replanning "Local Avoidance First", Sensitivity, Oracle Gap, G3).
Baseline M0 (bước 8): `docs/BASELINE_2026-11-27.md`.

## 1. Thành phần

| Thành phần | File |
|---|---|
| Fast 2D (C++) | `src/fast2d.cpp`, `include/.../fast2d.hpp` |
| Python binding | `src/fast2d_py.cpp` → `stage5_benchmark.fast2d` |
| Cấu hình | `python/.../config.py`, `config/benchmark.yaml` |
| Lưới, khả thi | `python/.../grid.py` |
| Layout | `python/.../layouts.py` |
| Generator, độ khó | `python/.../generator.py` |
| Evaluator | `python/.../evaluator.py` |
| Runner song song | `python/.../runner.py` |
| Lưu manifest | `python/.../scenario_io.py` |
| Env SAC (chưa train) | `python/.../env.py` |
| So sánh cơ chế replan | `scripts/replan_compare.py` (R0–R3, R2s, nhiều bộ tham số, LAR, nguyên nhân replan) |
| Báo cáo replan | `scripts/replan_report.py` (bảng theo nhóm, phân loại ghép cặp, tiềm năng SAC) |
| Hình replan | `scripts/replan_figures.py` (map, Δ, A* trước/sau replan, vị trí kích hoạt, trạng thái DWA) |
| Sensitivity / Oracle Gap | `scripts/sensitivity_study.py` (V5: Oracle Gap đúng định nghĩa + cận dưới chéo seed) |
| gtest | `test/test_replan_supervisor.cpp` (R2/R3/R0, free-space, dừng an toàn) |
| Đóng băng V5 | `scripts/freeze_v5.py` (`--write` / `--check`) → `config/navigation_v5_frozen.json` |
| Tối ưu M0 | `scripts/tune_baseline.py` (GP-BO + chọn từ điển trên val) |
| Đánh giá bộ cố định | `scripts/baseline_eval.py` (LAR, replan, q, nhiều seed) + `scripts/baseline_report.py` |
| Hình baseline | `scripts/baseline_figures.py` (`episodes` / `summary`) |
| M0 cho bước 9 | `config/m0_v5.yaml` |
| Script | `scripts/*.py` |

## 2. Fast 2D dùng lại code nào

Dùng **nguyên** code của robot (không viết lại DWA/replan):

| Phần | Nguồn |
|---|---|
| DWA | `stage3_navigation::DwaCore`, `PathTracker`, `validateParams` |
| Replan | `stage3_navigation::ReplanSupervisor` + `path_blockage` |
| A* | `nav2_navfn_planner::NavFn` (như `NavfnPlanner::makePlan`) |
| Costmap | `nav2_costmap_2d::Costmap2D`, `raytraceLine`, footprint |
| Driver | `stage4_gazebo::ZlacDriverModel` (trễ + ramp) |

Phần mô phỏng lại (đơn giản, có chủ đích):

- World = map M (lưới) + vật cản mới Δ (hộp xoay / tròn), hình học chính xác.
- LiDAR: 720 tia/360°, giữ ±90° trước, 10 Hz, nhiễu 0.01 m, tầm 0.05–16 m.
- ObstacleLayer (clear rồi mark, rolling, footprint clearing, unknown, inf) + InflationLayer theo Nav2 Humble.
- controller_server 10 Hz (progress/goal checker, failure_tolerance, chờ costmap current),
  watchdog driver 0.5 s, odometry v lọc thông thấp như `zlac8015d_odom`.
- **Định vị hoàn hảo** (Planner Benchmark). Không mô hình điện/mô-men/ma sát.

Tốc độ đo (2026-11-26): ~25–50× thời gian thực mỗi lõi; 786 episode ≈ 6 phút với 6 tiến trình.

## 3. Chạy nhanh

```bash
source /opt/ros/humble/setup.bash && source ~/capstone_ws/install/setup.bash
S=~/capstone_ws/capstone_project/stage5_benchmark/scripts
python3 $S/pilot.py --out ~/capstone_ws/bench_runs/pilot --per-type 30 --seeds 0,1
python3 $S/a_stage_tests.py --scenarios <pilot>/scenarios --out a.json --exp footprint,rotate
python3 $S/gen_scenarios.py --out ~/capstone_ws/bench_data/v3
python3 $S/gazebo_episode.py --manifest <dir> --ids a,b --param-sets sets.json --out gz.jsonl
python3 $S/sim2gz_compare.py --manifest <dir> --gazebo gz.jsonl --param-sets sets.json --out cmp.json
python3 $S/freeze_v5.py --check                       # trước mọi lần tối ưu / đánh giá / training
python3 $S/tune_baseline.py --train v3/general_train --val v3/general_val,v3/general_s1_val --out <dir>
python3 $S/baseline_eval.py --splits v3/general_val,stress_pilot --sets sets.json --names default --seeds 0,1,2 --out ev.jsonl.gz
python3 $S/baseline_report.py --inputs ev.jsonl.gz --out rep.json --md rep.md
```

`gazebo_episode.py` tự launch `s3_navigation` headless với `ROS_DOMAIN_ID` riêng + `ROS_LOCALHOST_ONLY=1`.

So sánh cơ chế Replanning (V5, chỉ train/dev; test bị khóa):

```bash
python3 $S/replan_compare.py --data ~/capstone_ws/bench_data/v3 --splits general_val,house_val,classroom_val,\
general_s1_val,house_s1_val,classroom_s1_val --mechs R0,R1,R2,R2s,R3 --sets default,timid,bold --out dev.jsonl
python3 $S/replan_report.py --inputs dev.jsonl --out report.json --md report.md
python3 $S/replan_figures.py --data ~/capstone_ws/bench_data/v3 --split general_val --ids <id> --mechs R1,R2s,R3 --out figs
colcon test --packages-select stage5_benchmark && colcon test-result --verbose     # gtest test_replan_supervisor
```

| Cơ chế | `goal_path_bridge.replan.*` |
|---|---|
| R0 | V3: `trigger: stall` (bị chặn + không tiến 4 s) |
| R1 | V4: + `early_distance` 1.0 m (A\* né thay DWA, chỉ để so sánh) |
| R2 | V5 không tiếp cận chậm: `trigger: conditional`, cung tự do < 0.15 / 0.4 s, dự phòng 4 s |
| **R2s** | **V5 đã chọn** = R2 + `approach_speed` 0.15 m/s khi ô chặn ≤ 1.2 m (mặc định trong `nav2_params.yaml`) |
| R3 | chỉ DWA sau đường ban đầu (`obstacle_replan: false`, `max_abort_replans: 0`), CHỈ chẩn đoán mô phỏng |

Kết quả V5 (2026-11-27, chi tiết `docs/ACCEPTANCE_2026-11-27.md`, dữ liệu `results/2026-11-27/`):

| Thí nghiệm | Kết quả |
|---|---|
| Dev 550, mặc định | R2s SR 99.2 %, STR 0.4, CR 0, replan 16.6 % ep, LAR 58 % (V4: LAR 0) |
| Sensitivity main (125 × 30 bộ) | SR 82.8–100 % (V4 98.8–100 %), CR 0 mọi bộ |
| Oracle Gap đúng, main / stress | SR 0 / 2.5 điểm; q 0.016 / 0.052 (< noise floor 0.088 / 0.231) |
| Ceiling main / stress | 23 / 11 trên 30 bộ có SR ≥ 95 % |
| G3 (planner_gt, 26 scenario × 5 bộ) | khớp nhãn 103/106 episode hợp lệ, 0 va chạm, sai số thời gian 0.45 % |

`sensitivity_study.py`: `oracle_gap_v5` = J trung bình qua seed; oracle = TB theo scenario của max theo bộ;
best fixed = max theo bộ của TB (CR ≤ 2 %); gap ≥ 0. `gap_*_cross` (chọn nửa A, đo nửa B) chỉ là cận dưới.
`--replan '<json>'` ghi đè `replan.*` để so sánh cơ chế trên cùng dữ liệu.

### Baseline M0 (bước 8, 2026-11-27)

Chi tiết + lệnh tái lập: `docs/BASELINE_2026-11-27.md`. Dữ liệu: `results/2026-11-27/baseline/`.

| Mục | Kết quả |
|---|---|
| Phương pháp | GP-BO (Matern 5/2 ARD, EI) trên miền action SAC; chọn trên val: CR → SR → STR → SSR → q |
| Ngân sách | 109 lần thử + đánh giá cuối: 84 060 episode, ≈ 49 giờ·lõi, 4.6 h |
| M0 General | = bộ hiện tại (wP 0.5, wG 2.0, wO 4.0, wH 0.3, wS 1.5, T 2.0) |
| General val (5 seed) | SR 99.3, CR 0, STR 0.2, SSR 100, LAR 64 %, replan 15 % ep |
| Zero-shot Classroom / House | SR 99.2 / 99.8; SSR 100 / 82 |
| Stress | SR 95.7 (C2 Hard 87.6) |
| Per-map tuning | không lợi (Classroom 99.0 so với 99.2 zero-shot) |
| Cảnh báo | wS ≈ 0 làm A\* né thay DWA (LAR 3–32 %) → cần chặn khi thiết kế SAC |

Trace Fast 2D (V5) thêm cột `fs_clearance`, `fs_free_ratio` (DwaCore, không phụ thuộc tham số), `sup_state`
(0 NORMAL, 1 PATH_BLOCKED, 2 LOCAL_BLOCKED, 3 GLOBAL_BLOCKED, 4 SAFE_STOP), `block_dist`, `local_bad`,
`n_candidates`, `deviation`; kết quả có `plans` (mọi Global Path đã gửi, để vẽ trước/sau replan).

## 4. Scenario

| Trường | Ý nghĩa |
|---|---|
| `type` | N1–N3, B1–B3, C1–C4, S1 |
| `feasibility` | feasible / unsolvable (từ generator) |
| `difficulty` | score + c, ρ, τ, δ, ν, level |
| `knowledge` | known (vật cản trong M) / unknown (Δ) |
| `obstacles` | Δ, chỉ có trong world |
| `map_id` | lưới M trong `maps/<id>.npz` |
| `hash`, `seed`, `split` | tái lập, chống rò rỉ |

Quy tắc kiểm tra (Master Plan v3 §5.3): khả thi nếu W nở 0.30 m còn nối; vô nghiệm nếu W nở 0.20 m mất nối; còn lại
loại. Thêm: đường tắt qua khe 0.40–0.60 m → loại; khe quanh mỗi vật Δ ∉ (0.40, 0.60); vật Δ cách start ≥ 3.0 m;
B1/B2 phải né được trong hành lang ±1 m, B3 phải cần replan; giới hạn độ dài đường của NavFn.

## 5. Nhãn episode (dùng chung Fast 2D / Gazebo)

| Nhãn | Điều kiện |
|---|---|
| SYSTEM_ERROR | lỗi hạ tầng (sim exception, sensor/GT mất > 2 s, RTF < 0.5) |
| COLLISION | hở GT < −0.005 m |
| SUCCESS | bridge SUCCEEDED ∧ GT ≤ 0.15 m (2D) / 0.25 m (Gazebo) ∧ yaw ≤ 0.25 |
| SAFE_STOP | **chỉ** scenario vô nghiệm: FAILED/ABORTED + đứng yên ≥ 2 s |
| STUCK | scenario khả thi: FAILED (FALSE_NO_PATH), ABORTED, không tiến 30 s |
| TIMEOUT | còn lại, t > T_max = 30 + 3·L/0.3 |

## 6. Giới hạn đã biết

- Định vị hoàn hảo, không trượt bánh: Fast 2D không thay được Gazebo cho Full-system Benchmark.
- Vật cản cao 0.6 m (LiDAR luôn thấy); không mô phỏng camera / vật thấp.
- Không mô phỏng trễ mạng ROS, lịch chạy bất đồng bộ (pha cố định, xác định).
- Báo cáo `/CustomDWB/state` tới bridge 5 Hz (Fast 2D báo mỗi 2 chu kỳ 10 Hz, như robot).
