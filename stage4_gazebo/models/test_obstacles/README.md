# Thư viện vật cản thử nghiệm (`stage4_gazebo/models/test_obstacles`)

6 hộp tĩnh dùng chung cho **mọi world Gazebo Fortress** của `stage4_gazebo` (House, Classroom = world sinh từ
`maps/map.yaml`, Warehouse, Office, `empty_floor`, world sinh từ map bất kỳ) để thử điều hướng với **vật cản mới
không có trong map tĩnh**, gồm cả vật cản nằm **dưới mặt phẳng quét LiDAR 2D**.

- Không world nào include các model này, không map nào bị sửa → mặc định **không có vật cản nào xuất hiện**.
- Chỉ xuất hiện khi người dùng chủ động chèn (GUI) hoặc gọi `spawn_obstacle.py`.
- Benchmark (`stage5_benchmark/scripts/gazebo_episode.py`, vật cản tên `d_<id>_<k>`) và Fast 2D không dùng thư viện này.

## 1. Hình học robot (sim, `urdf/robot_sim.urdf.xacro`)

| Đại lượng | Giá trị |
|---|---|
| Mặt phẳng quét LiDAR | **0.347 m** trên sàn |
| Đáy thân (collision) | 0.1135 m |
| Bán kính thân | 0.20 m |

- LiDAR: `base_footprint→base_link` 0.1135 + `lidar_joint` 0.23365 m. `gpu_lidar` có **1 tia dọc, góc 0** → chỉ thấy vật cắt đúng mặt phẳng z = 0.347 m.
- Collision gầm (sim): 2 cầu caster r 0.0375 m ở x ±0.068; 2 cầu bánh r 0.0535 m ở y ±0.17. Khe giữa caster và bánh chỉ 0.079 m.
- Robot thật (CAD): gầm thấp nhất là tấm gá motor ≈ 0.034 m, càng caster ≈ 0.047 m (sim không có collision cho phần này).

## 2. Sáu model

| Model | D × R × C (m) | Loại |
|---|---|---|
| `tall_small` | 0.18 × 0.18 × 0.55 | cao |
| `tall_wide` | 0.35 × 0.22 × 0.60 | cao |
| `tall_long` | 0.70 × 0.18 × 0.50 | cao |
| `low_small` | 0.18 × 0.18 × 0.08 | thấp |
| `low_wide` | 0.40 × 0.25 × 0.10 | thấp |
| `low_long` | 0.80 × 0.15 × 0.07 | thấp |

- Giữ nguyên kích thước đề xuất, vì đã kiểm tra đạt:
  - cao: 0.50–0.60 m > 0.347 m (thừa ≥ 0.15 m) → LiDAR thấy;
  - thấp: 0.07–0.10 m < 0.347 m → LiDAR không thấy;
  - thấp: 0.07–0.10 m > tâm cầu bánh 0.0535 m và tâm cầu caster 0.0375 m → bánh/caster không leo được;
  - mọi cạnh ngang ≥ 0.15 m > khe 0.079 m → không lọt giữa caster và bánh.
- Hộp thấp < 0.1135 m nên thân sim đi **qua phía trên**; robot bị chặn bởi bánh/caster. Trên robot thật, tấm gá motor (0.034 m) cũng chạm.
- Màu: cao = đỏ đất nung, thấp = vàng cam.
- `static` (robot không đẩy được), collision = visual (cùng một hộp), không mesh, không tải mạng.
- Gốc model ở **tâm mặt đáy** → spawn z = 0. Trục dài của hộp = trục x của model (yaw xoay trục này).
- Tên entity trong Entity Tree luôn có tiền tố `test_obstacle_` (ví dụ `test_obstacle_low_wide_2`).

## 3. Cấu hình dùng chung

- Model nằm ở `stage4_gazebo/models/test_obstacles/<tên>/{model.config, model.sdf}`, được cài vào `share/stage4_gazebo/models/test_obstacles` (với `--symlink-install` là symlink về source).
- `IGN_GAZEBO_RESOURCE_PATH` được thêm thư mục trên ở 2 chỗ, giữ nguyên các đường dẫn cũ:
  - `launch/sim_world.launch.py`: mọi launch `s0`…`s3` đều đi qua file này;
  - env-hook `env-hooks/stage4_gazebo_models.dsv.in`: `source install/setup.bash` là đủ, kể cả khi gọi thẳng `ign gazebo`.
- `config/gui.config` có thêm plugin GUI **Resource Spawner** (có sẵn trong Fortress, không phải plugin tự viết).

Sau khi pull: `colcon build --symlink-install --packages-select stage4_gazebo`, mở terminal mới rồi `source install/setup.bash`.

## 4. Chèn bằng GUI

1. Launch world như bình thường, ví dụ:
   - House: `ros2 launch stage4_gazebo s3_navigation.launch.py env:=house`
   - Classroom: `ros2 launch stage4_gazebo s3_navigation.launch.py env:=map`
   - Lệnh `s0_control` / `s1_odom` / `s2_slam` cũng dùng được.
2. Panel **Resource Spawner** (cột phải; không thấy thì vào menu ⋮ → *Resource Spawner*) → mục *Local resources* → chọn đường dẫn `.../share/stage4_gazebo/models/test_obstacles`.
3. Bấm một trong 6 model → click vào vị trí trên sàn để đặt.
4. Di chuyển/xoay: chọn vật → thanh công cụ *Translate* (phím `T`) / *Rotate* (phím `R`) → kéo trục.
   - Chỉ xoay quanh trục z (vòng xanh dương), để hộp đứng thẳng trên sàn.
5. Nhân bản: chọn vật → `Ctrl+C`, `Ctrl+V`, rồi click chỗ đặt. Xoá: chuột phải → *Remove*, hoặc phím `Delete`.
6. Chạy thử điều hướng (2D Goal Pose trong RViz).

## 5. Dòng lệnh (`scripts/spawn_obstacle.py`)

```bash
ros2 run stage4_gazebo spawn_obstacle.py --models                                   # 6 loại + kích thước
ros2 run stage4_gazebo spawn_obstacle.py --model tall_small --x 1.0 --y 2.0 --yaw 0.0
ros2 run stage4_gazebo spawn_obstacle.py --model low_long --x 2 --y 0 --yaw-deg 90  # thêm bản nữa: chạy lại
ros2 run stage4_gazebo spawn_obstacle.py --model low_wide --x 0 --y 1 --world aws_small_house
ros2 run stage4_gazebo spawn_obstacle.py --list
ros2 run stage4_gazebo spawn_obstacle.py --remove test_obstacle_tall_small_1
ros2 run stage4_gazebo spawn_obstacle.py --remove-all
```

- `x, y` theo frame world Gazebo, trùng frame `map` của map sim. Lấy tọa độ bằng *Publish Point* trong RViz, hoặc đọc pose trong Gazebo.
- `--yaw` (rad) hoặc `--yaw-deg` (độ) xoay trục dài của hộp.
- Tên tự sinh `test_obstacle_<model>_<k>` (k nhỏ nhất chưa dùng). `--name` để tự đặt tên (tự thêm tiền tố).
- World được tự dò qua service `/world/<w>/create`. Nếu có nhiều world thì bắt buộc `--world`.
- Terminal phải cùng `IGN_PARTITION` với simulation (mặc định cả hai đều rỗng).
- `--remove` / `--remove-all` chỉ xoá model có tiền tố `test_obstacle_`, gồm cả bản chèn từ GUI. Robot `ace492`, tường, đồ đạc và vật cản benchmark không bao giờ bị xoá.
- Dùng CLI `ign service` (UserCommands), không cần ROS graph; mọi world của dự án đều nạp `UserCommands`.

## 6. Kiểm chứng (2026-11-27, headless, partition/domain riêng)

| Kiểm tra | House | Classroom |
|---|---|---|
| Không vật cản mặc định | PASS | PASS |
| 6 model tìm thấy + spawn đúng pose | PASS | PASS |
| LiDAR thấy 3 hộp cao | PASS | PASS |
| LiDAR không thấy 3 hộp thấp | PASS | PASS |
| Hộp chặn robot (cả 6) | PASS | PASS |
| Di chuyển/xoay, nhiều bản, xoá 1, xoá hết | PASS | PASS |

- LiDAR: hộp đặt cách LiDAR 1.0 m, đọc `/sim/scan` thật.
  - Hộp cao: tia trước đo 0.870–0.895 m, kỳ vọng mặt hộp 0.890–0.910 m (sai lệch ≤ 0.026 m).
  - Hộp thấp: tia trước vẫn đo tường phía sau (3.7 m ở House, 6.3 m ở Classroom).
- Chặn: lái thẳng 0.15 m/s vào hộp, theo dõi ~465 pose. Hình collision của robot lấn vào hộp tối đa 0.1 mm (chỉ chạm), robot không leo (z = 0), hộp không dịch. Không có vật cản thì robot đi được 1.33 m.
  - Thân tròn trượt dọc hộp sang bên: robot bị chặn, không xuyên qua.
- `s3_navigation env:=house` và `env:=map` vẫn chạy như cũ: `/scan_filtered` có dữ liệu, `controller_server` / `map_server` / `amcl` active, 0 vật cản thử nghiệm, 0 lỗi.
- `colcon test stage4_gazebo`: 11 / 0 failures. Không sửa world, map, tham số nav2/DWA/A*/costmap, benchmark.
- **Chưa kiểm** thao tác chuột trong GUI: máy này không chạy được GUI offscreen (Ogre segfault cả với `gui.config` gốc). Mới kiểm là plugin Resource Spawner nạp được.

## 7. Hạn chế

- Hộp thấp **chỉ LiDAR không thấy**. Với `camera:=true` (lớp fusion, dải cao 0.05–0.70 m) camera depth thấy cả 6 hộp. Muốn thử giới hạn quan sát của LiDAR thì để `camera:=false` (mặc định).
- Robot sim không có collision cho tấm gá motor / càng caster (robot thật ≈ 0.034 / 0.047 m). Trong sim, hộp thấp chạm bánh/caster chứ không chạm thân như trên xe thật.
- Resource Spawner của Fortress tự truy vấn danh sách model Fuel (mạng). Offline thì phần đó rỗng; mục *Local resources* vẫn chạy. Thư viện không tải gì.
- Tên khi chèn bằng GUI (`test_obstacle_tall_small`, bản sao có hậu tố) theo cơ chế đổi tên của Gazebo, chưa kiểm trên GUI.
- Vật cản chèn bằng GUI/CLI mất khi tắt simulation (cố ý; world gốc không đổi).
