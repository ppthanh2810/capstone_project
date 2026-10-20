# Kế hoạch: né vật cản bất ngờ (camera + LiDAR) và chống UNCERTAIN giả

Bắt đầu: 2026-10-20. File này ghi lại các quyết định, công đoạn và tiến độ từ P1 tới P4.
Cập nhật file mỗi khi xong một bước.

## 1. Mục tiêu
- Robot **né được vật cản không có trong map** (ví dụ ghế bị dời), kể cả vật LiDAR không thấy vì sai chiều cao.
- Vật lạ **không bị coi là "lạc"**: monitor không báo UNCERTAIN giả, bridge không hủy follow.
- **Không** dùng visual SLAM (RTAB-Map), **không** replanning A\*. Việc tránh vật cản do DWB tự làm.

## 2. Quyết định đã chốt (2026-10-20)

| Module | Quyết định |
|---|---|
| M1 Distance filter cho monitor | Làm (P1) |
| M2 Lớp hợp nhất camera + LiDAR | Làm (P2) |
| M3 YOLO nhận diện vật | Làm (P3) |
| M4 Lọc scan cho AMCL | Làm cùng P3 |
| M5 Object layer + replanning | **Bỏ** (không replanning) |
| M6 Collision Monitor | Chưa cần |

- Đã cài `onnxruntime` 1.23.2 (`pip3 install --user`, numpy giữ 1.26.4).
- `vision_msgs` cần sudo. Bạn tự chạy: `sudo apt install ros-humble-vision-msgs`.

### M4 khác M2 thế nào
- **M2** = hợp nhất camera + LiDAR **vào costmap** để **né**. Đây là lớp "camera thấy mà LiDAR không thấy thì
  vẫn đánh dấu".
- **M4** = bỏ các tia LiDAR chạm vật lạ **khỏi phép định vị** (AMCL). Không liên quan costmap.
- M5 cũ là đưa vật vào global costmap để A\* lập lại đường. Đã bỏ.

### Luật hợp nhất cho M2 (theo yêu cầu)

| LiDAR | Camera | Kết luận vào costmap |
|---|---|---|
| Thấy | Thấy | Vật cản, xác nhận ngay. Vị trí lấy theo LiDAR (chính xác hơn) |
| Không | Thấy | **Vẫn là vật cản (ưu tiên camera)** sau ≥ N frame liên tiếp, để lọc nhiễu độ sâu |
| Thấy | Không | Vật cản (LiDAR tin cậy; camera có thể sót chân ghế mảnh hoặc vật trong vùng < 0.28 m) |
| Không | Không | Trống, được xóa |

- Hai trường hợp "LiDAR không thấy": vật cao hơn hoặc thấp hơn mặt quét 0.35 m (mặt bàn, mặt ghế, chân đế ghế
  xoay), hoặc vật trong suốt / đen mờ.
- Camera chỉ có thẩm quyền trong vùng nhìn của nó (±43° ngang, 0.3–2.5 m). Ngoài vùng đó chỉ có LiDAR.

## 3. Lộ trình

### P1. Distance filter cho monitor (M1). Trạng thái: **XONG offline**, chưa thử trên robot
Nguồn: Fox, Burgard, Thrun 1999 (distance filter); Thrun *Probabilistic Robotics* ch.6 (`z_short`).
- [x] `stage2_slam/src/scan_map_consistency.cpp` (+ `.hpp`): `MapRaycaster` (Amanatides–Woo) và
      `evaluateScanConsistency`.
- [x] Monitor dùng consistency đã lọc. `/localization/status` có thêm `scan_consistency_all`, `blocked_ratio`,
      `through_wall_ratio`, `counted_points`.
- [x] `localization_health`: `blocked_ratio_warn` 0.85.
- [x] Tham số `distance_filter.*` trong `config/localization_monitor.yaml`. `enabled: false` quay về cách cũ.
- [x] gtest `test_scan_map_consistency`: 16 test (raycast, vật cản, che kín, 7 ca lạc, ca có nhiễu, health).
      `colcon test` 17/0 lỗi.
- [x] README `stage2_slam` mục 7 và `stage3_navigation` mục 9–10.
- [ ] **Thử trên robot** (bạn chạy; lần đầu kê bánh khỏi mặt đất, E-stop sẵn sàng):
  1. Robot đứng yên, `navigation.launch.py` (`navigation_enabled:=false`). Đặt 2D Pose Estimate.
     `ros2 topic echo /localization/status`. Đặt ghế hoặc thùng trước robot 0.5–1.5 m.
     Kỳ vọng: `scan_consistency` ≥ 0.45 còn `scan_consistency_all` tụt.
  2. Đặt sai 2D Pose Estimate (lệch ~0.5 m hoặc 30°). Kỳ vọng: UNCERTAIN/LOST.
  3. Ghi bag: `ros2 bag record -o ~/p1_monitor /scan_filtered /map /tf /tf_static /amcl_pose /localization/status`.
  4. Chạy chậm với `navigation_enabled:=true auto_follow:=true`, **giữ `monitor:=true`**, ghế cạnh đường.
- Tiêu chí đạt: không bị hủy vì localization khi có ghế; pose sai vẫn bị báo.
- Rủi ro: ca lạc "bản đồ tưởng robot xa tường phía trước hơn thực tế" ở chỗ trống (xem README stage2).
  Theo dõi `blocked_ratio` trên đường dài.

### P2. Lớp hợp nhất camera + LiDAR vào local costmap (M2). Trạng thái: **XONG offline**, chưa thử trên robot
Nguồn: Lu, Hershberger, Smart 2014 (layered costmaps); Macenski 2020 (STVL); Nav2 Voxel/Obstacle Layer.
- [x] `stage3_navigation/launch/camera.launch.py` (`realsense2_camera` 4.57): `publish_tf:=false`, độ sâu
      424×240 @15 fps, decimation 2 (~25k điểm), point cloud **không texture** (giữ đủ FOV 87°), màu tắt.
      `navigation.launch.py` thêm `camera:=false` (driver) và `camera_layer:=auto|fusion|voxel|none`.
- [x] Bước 1 (đối chứng): `config/camera_voxel.yaml`, `nav2_costmap_2d::VoxelLayer` **tách khỏi** lớp LiDAR,
      gộp bằng `max`, độ cao 0.05–0.70 m.
- [x] Bước 2: plugin `stage3_navigation::FusionLayer` (`src/fusion_layer.cpp`) + lõi không ROS
      `CameraFusionGrid` (`src/camera_fusion.cpp`), `config/camera_fusion.yaml`. Đã chọn:
  - LiDAR vẫn do `obstacle_layer` vẽ. `fusion_layer` chỉ giữ bằng chứng camera theo ô (khóa theo tọa độ `odom`,
    không phụ thuộc cửa sổ cuộn).
  - Camera-only: 3 frame liên tiếp. Gần điểm LiDAR (0.10 m): xác nhận ngay, không vẽ (vị trí theo LiDAR).
  - Xóa: camera "nhìn xuyên" ô ở mọi độ cao đã thấy vật, 2 frame liên tiếp (lưới bin phương vị 1° × góc ngẩng 2°,
    khoảng cách gần nhất mỗi bin). Lỗ độ sâu / bị che / quá gần / ngoài FOV → không xóa, hết hạn sau 10 s (TTL).
  - Không có cloud 1 s → costmap không current → controller dừng.
  - Trạng thái: `/local_costmap/fusion_layer/status`.
- [x] Test:
  - gtest `test_camera_fusion` 16 test (từng dòng bảng luật, nhiễu nhấp nháy, điểm thưa, lỗ độ sâu, bị che, quá
    gần, ngoài FOV, lọc thân robot, tải). Xử lý **~2 ms / frame 25k điểm**.
  - Benchmark offline trong `controller_server` thật: `offline_sim.py camera:=true` (camera giả lập 3D) +
    4 kịch bản `cam_*` (`run_benchmark.py --camera-layer`). Kết quả 2026-10-20 (54 lần chạy, số liệu trong
    `stage3_navigation/doc/benchmark/m2_camera_2026-10-20/`):

| Kịch bản (CustomDWB, 3 lần) | `none` (chỉ LiDAR) | `fusion` |
|---|---|---|
| Mặt bàn 0.45–0.75 m trên đường | **3/3 đâm** | 3/3 dừng, hở 0.10 m |
| Hộp thấp 0.12 m trên đường | **3/3 đâm** | 3/3 dừng, hở 0.10 m |
| Hộp cao 0.8 m (cả hai thấy) | 3/3 dừng | 3/3 dừng |
| Hộp thấp cạnh đường | 3/3 tới đích | 3/3 tới đích, hở 0.16 m |
| `straight`, `obstacle_near_path` | 6/6 | 6/6 (không vật cản ma) |

- `voxel` (đối chứng): cùng kết quả như `fusion` trên các kịch bản này (camera giả lập không nhiễu, nên chưa thấy
  khác biệt về lọc nhiễu).
- Thời gian tính CustomDWB p99 ~3 ms ở cả ba cấu hình; chu kỳ lệnh p99 ~102 ms (10 Hz giữ được).
- "Dừng" = `holding position` rồi progress checker abort sau 10 s (hành vi đã biết, P4 sẽ xử lý đi vòng).
- [ ] Robot (bạn chạy, xem mục "Thử P2 trên robot" bên dưới): mặt ghế / mặt bàn thấp / vật thấp 0.1 m hiện trong
      `/local_costmap/costmap`; vật mất ≤ 1 s sau khi dời; sàn không tạo vật cản ma.
- Còn thiếu: **chiều cao robot kể cả laptop** (đang giả định 0.6 m → `max_obstacle_height` 0.70);
  độ nghiêng camera thật (CAD: nằm ngang).

#### Thử P2 trên robot (bạn chạy)
Lần đầu cách ly motor (`navigation_enabled:=false`), không cần bánh chạm đất.
1. `ls /dev/video* ; rs-enumerate-devices -s` (nếu có) để chắc D435 được nhận.
2. `ros2 launch stage3_navigation navigation.launch.py camera:=true monitor:=false`
3. `ros2 topic hz /camera/camera/depth/color/points` (kỳ vọng ~15 Hz);
   `ros2 topic echo /local_costmap/fusion_layer/status` (kỳ vọng `OK`, `proc_ms_mean` vài ms).
4. RViz: thêm PointCloud2 `/camera/camera/depth/color/points`, xem `/local_costmap/costmap`.
   - Sàn trống: không có ô đỏ nào (nếu có → camera nghiêng; tăng `min_obstacle_height` 0.08 hoặc giảm `max_range`).
   - Đặt hộp thấp ~0.1 m và ghế (mặt ghế) trước robot 0.6–2 m: ô hiện sau < 0.5 s.
   - Dời đi: ô mất < 1 s (vật còn trong tầm nhìn).
5. Bag: `ros2 bag record -o ~/p2_camera /camera/camera/depth/color/points /scan_filtered /tf /tf_static
   /local_costmap/costmap /local_costmap/fusion_layer/status /odometry/filtered`.
6. Sau đó mới thử kê bánh rồi chạy chậm (`navigation_enabled:=true auto_follow:=true`, E-stop sẵn sàng).

### P3. YOLO nhận diện vật (M3) + lọc scan cho AMCL (M4). Trạng thái: chưa làm
Nguồn: Qi 2018 (Frustum PointNets); Hähnel 2003 và Fox 1999 (lọc tia vật động trước khi định vị);
DynaSLAM 2018 (che vật động bằng mạng nhận diện).
- [ ] Package mới (ví dụ `stage3_perception`, Python): `object_detector`. YOLOv8n ONNX (`da1/da1/yolov8n.onnx`) trên
      ảnh màu, dùng ảnh độ sâu đã căn chỉnh (aligned depth). Khung 2D → frustum → cụm điểm gần nhất, bỏ sàn →
      vật 3D (tâm, bán kính, lớp) trong frame `odom`. Lớp: person, chair, couch, dining table, suitcase, bench,
      potted plant…
- [ ] Tracker: ghép với vật gần nhất; xác nhận khi thấy ≥ 3 frame; TTL ~5 s. Publish `vision_msgs/Detection3DArray`
      (`/perception/objects`) + MarkerArray.
- [ ] `FusionLayer` nhận thêm `/perception/objects`: vẽ footprint của vật (kể cả phần độ sâu thưa); có thể tăng
      biên an toàn cho `person`.
- [ ] M4 `scan_object_filter`: đặt NaN cho các tia `/scan_filtered` rơi vào vùng góc ±3° và khoảng cách ±0.2 m của
      vật đã xác nhận (tính trong frame robot, **không phụ thuộc pose map**). Giới hạn xóa tối đa 50% số tia →
      `/scan_loc` cho AMCL (`scan_topic`) và monitor. Costmap vẫn dùng `/scan_filtered` gốc.
- [ ] Đo: thời gian suy luận YOLO trên CPU (Ryzen 5 7535HS), tần số 5–10 Hz, sai số vị trí vật ≤ 0.1 m.
- Rủi ro: vị trí camera và LiDAR lấy từ CAD (cần chừa biên góc); YOLO bỏ sót vật lạ (M1/M2 vẫn đỡ);
  YOLO nhận nhầm (giới hạn bằng ≥ 3 frame và 50%).

### P4. DWB tự đi vòng vật cản chắn đường, không replanning. Trạng thái: chưa làm
Nguồn: Fox, Burgard, Thrun 1997 (DWA); Fox et al. 1998 (hybrid: kết hợp map với cảm biến);
Rösmann et al. 2017 (TEB, nhiều đường vòng theo topology). Hiện tại CustomDWB chỉ nhìn trước ~0.26 m và luôn bị
kéo về đường A\*, nên khi bị chắn thì chỉ đứng chờ.
- [ ] Trong CustomDWB: khi đoạn đường A\* nằm trong local costmap bị chắn (ô ≥ INSCRIBED), tìm một **đường vòng
      cục bộ** (A\* nhỏ trên local costmap) từ robot tới điểm nhập lại trên đường A\* phía sau vật cản. Dùng đường
      này làm tham chiếu cho `PathTracker` / cà rốt. **Đường toàn cục giữ nguyên**, không gọi planner_server.
- [ ] Không có đường vòng → giữ hành vi cũ (`holding position`).
- [ ] Cần xem xét lại: local costmap 2×2 m có thể quá nhỏ để vòng (thử 3×3 m; vấn đề 3×3 trước đây là của Nav2 DWB);
      progress checker 10 s có thể abort giữa lúc vòng; vùng mù phía sau.
- [ ] Test: thêm kịch bản `obstacle_blocking_path` vào `test_dwa_core` và benchmark offline; sau đó lên robot,
      chạy chậm.

## 4. Quy trình mỗi bước
1. Code + gtest offline → `colcon build` → `colcon test`. Riêng `stage2_slam`:
   `colcon build --packages-select stage2_slam --cmake-args -DBUILD_TESTING=ON`, vì cache cũ đang để `OFF`.
2. Cập nhật README liên quan + file này.
3. Lên robot: mình đưa lệnh để bạn tự chạy. Thứ tự: cách ly motor → kê bánh khỏi mặt đất → chạy chậm trên sàn.
   Luôn có E-stop.

## 5. Nhật ký
| Ngày | Việc |
|---|---|
| 2026-10-20 | Chốt kế hoạch. P1 xong offline (16 gtest). Cài onnxruntime |
| 2026-10-20 | P2 xong offline: `FusionLayer` + `VoxelLayer` đối chứng + `camera.launch.py`. 16 gtest, benchmark 54 lần, `colcon test` stage3 125 / 0 lỗi |

## 6. Nguồn
- Fox, Burgard, Thrun 1999 – Markov Localization for Mobile Robots in Dynamic Environments, JAIR 11.
  https://arxiv.org/pdf/1106.0222
- Thrun, Burgard, Fox 2005 – *Probabilistic Robotics*, ch. 6 (beam model, likelihood field).
- Hähnel, Schulz, Burgard 2003 – Mobile Robot Mapping in Populated Environments, Advanced Robotics 17.
  https://aif.bit.uni-bonn.de/rhino/research/papers/haehnel-populated.pdf
- Meyer-Delius, Beinhofer, Burgard 2012 – Occupancy Grid Models for Robot Mapping in Changing Environments, AAAI.
  https://ojs.aaai.org/index.php/AAAI/article/view/8377
- Pomerleau et al. 2014 – Long-term 3D map maintenance in dynamic environments, ICRA.
  https://homepages.loria.fr/FColas/website/publications/2014_Pomerleau_ICRA_Long-term.pdf
- Lu, Hershberger, Smart 2014 – Layered Costmaps for Context-Sensitive Navigation, IROS.
  http://ieeexplore.ieee.org/document/6942636/
- Macenski, Tsai, Feinberg 2020 – Spatio-temporal voxel layer, IJARS 17(2). https://index.ros.org/p/spatio_temporal_voxel_layer/
- Qi et al. 2018 – Frustum PointNets for 3D Object Detection from RGB-D Data, CVPR. https://arxiv.org/pdf/1711.08488
- Bescos et al. 2018 – DynaSLAM. https://ar5iv.labs.arxiv.org/html/1806.05620
- Fox, Burgard, Thrun 1997 – The Dynamic Window Approach to Collision Avoidance, IEEE RAM 4(1).
  https://www.ri.cmu.edu/publications/the-dynamic-window-approach-to-collision-avoidance
- Fox, Burgard, Thrun, Cremers 1998 – A Hybrid Collision Avoidance Method for Mobile Robots, ICRA.
  https://www.ri.cmu.edu/publications/a-hybrid-collision-avoidance-method-for-mobile-robots
- Rösmann, Hoffmann, Bertram 2017 – Integrated online trajectory planning and optimization in distinctive
  topologies, Robotics and Autonomous Systems 88.
- Nav2 docs: AMCL https://docs.nav2.org/configuration/packages/configuring-amcl.html ·
  Voxel Layer https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html ·
  Obstacle Layer https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html
- Intel D435i specs (FOV 87°×58°): https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html
