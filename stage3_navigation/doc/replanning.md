# Replanning khi gặp vật cản (Stage 3.3) — 2026-10-15

## 1. Vấn đề (đã kiểm tra trước khi sửa)

Trước thay đổi này, từ mã nguồn:
- `global_costmap` chỉ có `static_layer` + `inflation_layer` → A\* (`planner_server`) chỉ thấy bản đồ đã lưu, không thấy
  vật cản LiDAR.
- `goal_path_bridge` chỉ gọi `ComputePathToPose` khi có `/goal_pose` mới; FollowPath bị abort → trạng thái `ABORTED`, hết.
- Không có BT navigator (`nav2_bt_navigator` cài nhưng không launch nào dùng).

Kiểm chứng offline (`run_replan_test.py --replan false --scenario replan_detour`, hộp 0.4 × 1.0 m chưa có trong bản đồ
nằm trên đường thẳng): CustomDWB dừng cách vật cản 0.097 m, `controller_server` báo `Failed to make progress`,
FollowPath `ABORTED`, robot còn cách đích 2.8 m. Nghĩa là cả 4 ý trong nhận định đều đúng.

## 2. Thiết kế

```
LiDAR /scan_filtered ──┐                      (sau này) camera PointCloud2 ──┐
                       ▼                                                     ▼
global_costmap: static_layer → obstacle_layer (scan) → [camera_layer] → inflation_layer
        │  /global_costmap/costmap (2 Hz)                         ▲
        ▼                                                         │ cùng costmap
goal_path_bridge (5 Hz): đoạn đường phía trước robot có ô >= 99 ? │
        │ có, liên tục >= confirm_time                            │
        ▼                                                         │
ComputePathToPose (cùng đích, start = robot) ─────────────────────┘ planner_server (NavFn A*)
        │ thành công                         │ thất bại
        ▼                                    ▼
FollowPath goal mới (preempt, DWB         hủy FollowPath (robot dừng), chờ, thử lại mỗi min_interval,
nhận setPlan mới, không dừng xe)          quá wait_timeout -> FAILED
```

Nguyên tắc:
1. **Một nguồn sự thật = costmap.** Bridge không đọc cảm biến; nó đọc đúng costmap mà A\* dùng. Đường mới vì vậy
   tránh đúng vật cản đã làm đường cũ bị chặn (không lặp vô ích), và thêm cảm biến chỉ là thêm layer trong YAML.
2. **Chỉ trong phiên đã kích hoạt.** Phiên = từ `~/follow` / `auto_follow` tới SUCCEEDED / cancel / goal mới /
   timeout / localization mất / FAILED. Replan không bao giờ tự bắt đầu chuyển động khi người dùng chưa kích hoạt.
3. **An toàn tức thời vẫn do DWB + local_costmap.** Replan chỉ đổi *đường*; CustomDWB vẫn loại cứng mọi quỹ đạo
   va chạm. Không thay đổi `twist_mux`, ưu tiên PS4, `navigation_enabled`.
4. **Chỉ replan khi đường HIỆN TẠI bị chặn** (không replan định kỳ) → không dao động giữa hai đường tương đương.

Logic kiểm tra (`src/path_blockage.cpp`, không phụ thuộc ROS, có gtest): chiếu robot lên đường (chỉ tiến về phía trước
trong cửa sổ 2 m cung — không nhảy nhánh ở đường chữ U), lấy mẫu mỗi nửa ô từ `start_offset` tới `lookahead_distance`,
bị chặn khi chi phí OccupancyGrid ≥ `blocked_cost` (99 = nội tiếp, 100 = lethal; NavFn cũng coi ≥ 99 là vật cản).
Unknown (−1) và ngoài lưới không tính.

FollowPath abort trong phiên (DWB kẹt, không tiến) → cũng lập lại A\* (tối đa `max_abort_replans`), vì Humble
`controller_server` không reset progress checker khi preempt.

## 3. Tham số (`goal_path_bridge.replan.*`, `config/nav2_params.yaml`)

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `enabled` | true | launch `navigation.launch.py replan:=false` tắt (hành vi cũ) |
| `lookahead_distance` | 2.5 m | đoạn đường kiểm tra (= `obstacle_max_range`) |
| `start_offset` | 0.10 m | bỏ qua ngay tại robot |
| `blocked_cost` | 99 | ngưỡng chi phí OccupancyGrid |
| `confirm_time` | 0.5 s | phải bị chặn liên tục (lọc nhiễu) |
| `min_interval` | 2.0 s | giữa hai lần gọi A\* |
| `max_obstacle_replans` | 20 | mỗi phiên; quá → FAILED |
| `max_abort_replans` | 3 | lập lại sau FollowPath abort, mỗi phiên |
| `wait_timeout` | 15 s | không có đường → robot dừng, thử lại tới hết rồi FAILED |
| `costmap_timeout` | 5 s | costmap cũ hơn → không kiểm tra |
| `clear_costmap_on_failure` | **false** | xem §6 |
| `clear_keep_distance` / `clear_costmap_service` | 5.0 m / `/global_costmap/clear_except_global_costmap` | |

Mọi giá trị đọc lại mỗi lần dùng: `ros2 param set /goal_path_bridge replan.min_interval 3.0`.
Trạng thái mới trên `/goal_path_bridge/follow_state`: `REPLANNING: <lý do>`, `FOLLOWING (replanned Nx)`,
`WAITING: <lý do> (...)`, `FAILED: <lý do>`.

Thay đổi `global_costmap`: thêm `obstacle_layer` (`/scan_filtered`, giống local: 2.5 / 3.0 m, `inf_is_valid false`),
`expected_update_rate: 0.0` (planner_server Humble chờ costmap "current" → scan dừng sẽ treo A\*; kiểm tra scan cũ vẫn
ở local_costmap), `update/publish_frequency` 1 → 2 Hz. Ảnh hưởng cả `planning.launch.py` (A\* tránh vật cản đang thấy).

## 4. Tích hợp camera sau này (không cần sửa code)

1. RealSense: `publish_tf:=false` (giữ frame URDF), bật pointcloud; hoặc node phát hiện (YOLO + depth) xuất
   `PointCloud2` các điểm vật cản (vd `/obstacles/camera_points`, `frame_id: camera_depth_optical_frame`).
2. Thêm layer **riêng** `camera_layer` (plugin `nav2_costmap_2d::ObstacleLayer`) vào `plugins` của *cả hai* costmap,
   trước `inflation_layer` — mẫu có sẵn (comment) trong `nav2_params.yaml`. Layer riêng vì trong một layer, tia LiDAR
   (cao 0.35 m) xóa vật cản thấp mà chỉ camera thấy.
3. `min_obstacle_height` ≈ 0.05 m để bỏ sàn (cần TF camera đúng — hiện lấy từ CAD, chưa hiệu chuẩn).
4. Không cần sửa bridge / CustomDWB: replan và tránh vật cản đều đọc costmap.
Nếu sau này cần biết *cảm biến nào* chặn đường (log/UI), có thể kiểm tra từng layer riêng — hiện chưa làm.

## 5. Kiểm thử (offline, không phần cứng)

- `test_path_blockage` (10 gtest): free, nội tiếp, ngưỡng/unknown, ngoài lookahead/sau robot, start_offset,
  đường thưa, ô đích, lưới rỗng, không nhảy nhánh chữ U, tiến trình đơn điệu.
- `test/offline/run_replan_test.py` + `scenarios_replan.json` (controller_server, planner_server thật; `offline_sim.py`,
  trễ 0.2 s; `ROS_DOMAIN_ID=161`, `ROS_LOCALHOST_ONLY=1`; kiểm graph trước khi gửi goal):

| Kịch bản | Kết quả 2026-10-15 (replan bật) |
|---|---|
| `replan_none` (không vật cản) | SUCCEEDED, 0 replan |
| `replan_detour` (1 hộp) | SUCCEEDED, 1 replan, khoảng hở min 0.26 m |
| `replan_twice` (2 hộp nối tiếp) | SUCCEEDED, 2 replan, 0.26 m |
| `replan_mapped_detour` (tường bản đồ + tường mới, đường zig-zag) | SUCCEEDED, 2–3 replan (khác nhau giữa các lần chạy), 0.20 m |
| `replan_no_path` (tường mới chắn hết bản đồ) | robot dò dọc tường (9 replan), hết đường → FAILED sau 15 s chờ, 0.96 m |
| `replan_detour` với `replan:=false` | ABORTED (`Failed to make progress`), dừng cách 0.097 m — hành vi cũ |

Không va chạm, lệnh trong [0, 0.15] m/s, |ω| ≤ 0.5 rad/s ở mọi lần chạy. Chạy lại:
`python3 test/offline/run_replan_test.py --out <dir>` (mã thoát ≠ 0 nếu sai kỳ vọng).

Giới hạn của kiểm thử: localization hoàn hảo, LiDAR không nhiễu, vật cản tĩnh, 1 lần chạy mỗi kịch bản.
Một lần khởi động bị lỗi DDS (`failed to send response to /planner_server/change_state`) → planner không active;
runner nay chờ lifecycle `active` và thử lại khi lỗi setup.

## 6. Giới hạn / NOT_VERIFIED

- **Chưa chạy trên robot.** Ngưỡng, tần số, thời gian chờ là giá trị khởi đầu.
- **Vết vật cản trong frame `map` phụ thuộc localization.** Sai số AMCL (đặc biệt yaw ở xa) làm tường thật được LiDAR
  đánh dấu lệch khỏi tường bản đồ → hành lang hẹp có thể bị coi là chặn (replan / FAILED giả). Nửa sau scan là NaN
  nên vết sau robot không được xóa. Cần bag trên robot (xem `CLAUDE.md`, long-route localization).
- `clear_costmap_on_failure` mặc định **false**: bật lên, offline `replan_no_path` cho thấy việc xóa ngoài hình vuông
  quanh robot làm mất phần tường robot *đã thấy* → A\* quên → robot đi qua lại dọc tường tới `follow_timeout`
  (không va chạm nhưng là livelock). Xóa tay khi có vết cũ:
  `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap`.
- Global (nội tiếp 0.20 m) khác local (0.30 m vì padding 0.10): A\* vẫn có thể đi qua khe 0.4–0.6 m mà DWB không qua
  được → DWB kẹt → abort → lập lại ra cùng đường → sau `max_abort_replans` thì ABORTED. Chưa đổi footprint global
  (sẽ chặn cửa hẹp trên bản đồ thật) — cần quyết định.
- Vật cản động: không dự đoán chuyển động; người đi ngang có thể gây 1 lần replan rồi đường mới hơi vòng.
- `follow_timeout` (120 s) tính cho cả phiên, kể cả thời gian chờ/đi vòng.
- Preempt: Humble `controller_server` không reset progress checker khi đổi đường; nếu robot đứng lâu trước khi
  có đường mới, có thể bị abort sớm → được bù bằng `max_abort_replans`.
