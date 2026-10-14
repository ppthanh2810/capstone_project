# Đề xuất P2 — KHÔNG triển khai trong đợt này

Các mục dưới đây nằm ngoài plugin CustomDWB hoặc chạm vào đường lệnh motor / stage0, nên chỉ là đề xuất. Mỗi mục
cần người dùng duyệt trước, và phải tra đúng tài liệu/API **Humble** trước khi dùng (chưa đọc trong đợt này).

| # | Đề xuất | Lý do (bằng chứng) | Phạm vi chạm | Ghi chú |
|---|---|---|---|---|
| 1 | **Replanning / BT navigator** (`nav2_bt_navigator` đã cài): replan định kỳ hoặc khi FollowPath abort, global costmap có obstacle layer | `blocked_wall`, `obstacle_bypass`: controller dừng an toàn rồi abort; không tự đi vòng (giới hạn cấu trúc của DWA chân trời ngắn: 0.15 m/s × 1.7 s = 0.26 m) | stage3 launch/config | Cần quyết định có dùng NavigateToPose thay cho bridge hay không |
| 2 | **Guard khi local costmap stale**: `controller_server` Humble chờ `isCurrent()` vô hạn, không publish 0, không xử lý cancel (B7) | Kịch bản `stale_scan` (benchmark) | Tách: (a) trong stage3: `goal_path_bridge` giám sát tuổi `/scan_filtered` → cancel (nhưng server không xử lý cancel trong vòng chờ!); (b) chắc chắn hơn: lock/timeout ở `twist_mux` (stage0) | Hiện tại dừng nhờ timeout 0.5 s của `twist_mux` + watchdog 0.5 s của `zlac8015d_move`. **Đã có**, nhưng là gián tiếp |
| 3 | **twist_mux lock** (E-stop mềm / khóa navigation) | PS4 chỉ ghi đè tạm thời; dừng navigation hiện chỉ bằng cancel / Ctrl+C / E-stop cứng | stage0 `twist_mux.yaml` (+ nút PS4) | Thay đổi stage0: phải hỏi trước (CLAUDE.md) |
| 4 | **Đồng bộ biên global/local**: global costmap không padding (A* qua khe ≥ 0.4 m), local inscribed 0.30 m (controller cần khe ≥ ~0.6 m) | `corridor_narrow`: A* cho qua, controller dừng trước khe | `global_costmap` (`footprint_padding` hoặc inflation) | Đo footprint thật (laptop, cáp, bánh) trước; không giảm padding local chỉ để qua test |
| 5 | **Collision Monitor** (`nav2_collision_monitor` đã cài) giữa controller và `twist_mux` | Lớp an toàn độc lập với thuật toán (polygon dừng/giảm tốc theo scan) | stage3 launch + remap `cmd_vel` | Không thay thế E-stop vật lý |
| 6 | **Velocity Smoother** (`nav2_velocity_smoother` đã cài) | Giới hạn gia tốc lệnh độc lập; DWA neo cửa sổ vào vận tốc ĐO nên lệnh liên tiếp có thể chênh hơn acc·dt khi trễ lớn (`cmd_rate_exceed` trong sim lõi) | stage3 launch | Phải tính trễ thêm vào `control_latency` |
| 7 | **Theo dõi unknown** (`track_unknown_space: true`) ở local costmap | A6: vùng chưa quan sát hiện là FREE | `nav2_params.yaml` local_costmap | Chỉ bật sau khi thử với scan thật (tia inf không xóa ô) |
| 8 | **Driver**: `/wheel_feedback` / odometry có stamp từ phần cứng; trễ lệnh đo được | `control_latency` đang là synthetic | stage0/stage1 | Thay đổi driver: hỏi trước |
