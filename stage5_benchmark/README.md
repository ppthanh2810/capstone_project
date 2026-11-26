# stage5_benchmark — Hạ tầng benchmark SAC-DWA (chỉ mô phỏng)

Benchmark cho Master Plan v3 (`SAC_DWA_Research_Training_Master_Plan_v3.md`): Fast 2D simulator, Scenario Generator,
Feasibility/Difficulty, Episode Evaluator, chạy Gazebo cùng scenario, pilot, sinh dataset, công cụ baseline.
**Không chạm phần cứng.** Báo cáo nghiệm thu: `docs/ACCEPTANCE_2026-11-26.md`.

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
python3 $S/tune_baseline.py --train <train> --val <val> --out <dir> --budget 120
```

`gazebo_episode.py` tự launch `s3_navigation` headless với `ROS_DOMAIN_ID` riêng + `ROS_LOCALHOST_ONLY=1`.

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
