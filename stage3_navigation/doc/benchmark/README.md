# Benchmark CustomDWB — dữ liệu thô + cấu hình

**Tất cả là kết quả OFFLINE trên mô hình tối giản. Không chứng minh an toàn hay chất lượng trên robot.**

## `core_2026-10-14/` — lõi kín vòng, không ROS graph (`dwa_closed_loop_sim`)
- `nominal/`: robot mô phỏng khớp giả định controller (acc/decel 0.5 m/s², 1.0 rad/s², trễ 0.1 s, nhiễu
  vận tốc odom σ = 0.002).
- `sluggish/`: robot **chậm hơn giả định** (acc/decel 0.3 m/s², 0.6 rad/s², trễ 0.2 s, σ = 0.005) — controller
  vẫn dùng tham số mặc định (trễ 0.1 s, 0.5 m/s²). Kiểm tra độ bền khi mô hình sai.
- `nominal_traces/`: 1 seed/kịch bản, vết từng chu kỳ (`traces/<scenario>_<seed>.csv`: t, pose, v, ω, lệnh, pha).
- Mỗi thư mục: `params.yaml` (tham số + mô hình robot), `runs.csv` (một dòng/lần chạy), `summary.json`,
  `stdout.txt`. Seed 1000..1004 (nhiễu pose đầu σ = 0.02 m/rad ở một số kịch bản, nhiễu odom).
- Tái lập: `ros2 run stage3_navigation dwa_closed_loop_sim --out <dir> --runs 5 --robot nominal|sluggish`.
- Giới hạn mô hình: costmap tổng hợp thấy **toàn bộ** vật cản (không có nửa sau bị che), inflation tính hình học
  (gần đúng InflationLayer), localization hoàn hảo, không trượt bánh.

## `ros_2026-10-14/` — controller_server thật (fixture `test/offline/`)
- Profile: `custom` = CustomDWB; `nav2_dwb` = `FollowPath` (DWB, StandardTrajectoryGenerator — cấu hình robot hiện
  tại); `nav2_dwb_la` = `FollowPathLA` (bản sao FollowPath + `dwb_plugins::LimitedAccelGenerator`,
  `sim_period 0.1`; chỉ có trong `nav2_params_benchmark.yaml`). Cùng local costmap, goal/progress checker, giới hạn.
- Fixture: `offline_sim.py` (LiDAR 360 tia, nửa sau NaN; odom 30 Hz; robot trễ 0.1 s, acc 0.5 m/s²; lệnh cũ hơn
  0.5 s → 0 như `twist_mux`). Mỗi lần chạy là một graph mới, `ROS_DOMAIN_ID` 150..153, `ROS_LOCALHOST_ONLY=1`;
  graph được kiểm (không có node motor/teleop/driver) trước khi gửi FollowPath.
- Chạy 4 worker song song trên cùng CPU: thời gian tính/chu kỳ lệnh **bao gồm tải đó**.
- Tệp: `summary.md` (bảng), `summary.json`, `runs.csv` (mọi chỉ số từng lần chạy), `nav2_params_benchmark.yaml`,
  `meta.json`, `sources.md5` (checksum mã đã chạy), `raw.tar.xz` (`raw/<scenario>__<profile>__<run>/`:
  `sim.csv` quỹ đạo thật + khoảng hở, `client.json` lệnh + kết quả + tham số đã nạp + graph check,
  `launch.log` log controller_server).
