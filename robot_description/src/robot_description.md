# Phiếu thông số để dựng `mobile_robot_description` (ROS 2 Humble)

Điền trực tiếp vào các bảng. Ghi **CHƯA ĐO** nếu chưa có số liệu, **KHÔNG CÓ** nếu xe không dùng bộ phận đó. Ưu tiên số đo trên robot thật. Gửi lại file cùng CAD/STL và ảnh đánh dấu vị trí cảm biến nếu có.

**Quy ước khi điền:** chiều dài `mm`, khối lượng `kg`, góc `độ`; mình sẽ đổi sang `m` và `rad` trong URDF. Nhìn từ phía sau xe theo hướng xe chạy: `+X` về trước, `+Y` sang trái, `+Z` lên trên. Nếu cách đặt trục của bạn khác, ghi rõ ở phần 1.

## 1. Mốc đo chung

Đề xuất chọn **O tại điểm giữa hai tâm bánh motor**, trên đường nối hai tâm bánh. Mọi tọa độ `(x,y,z)` dưới đây đo từ O; `+X` hướng về đầu xe, `+Y` sang trái, `+Z` lên trên. Nếu bạn dùng mốc khác, ghi rõ trước khi đo. `base_footprint` dự kiến là hình chiếu O xuống sàn.

| Mục | Bạn điền |
| --- | --- |
| Tên robot/package mong muốn |  |
| Hướng đầu xe trong ảnh; có dùng quy ước +X trước, +Y trái? |  |
| Mốc O đề xuất có phù hợp? Nếu không, mô tả mốc riêng |  |
| Độ cao O so với mặt sàn (mm) |  |


## 2. Base dưới, base trên và trụ nối

Kích thước `D × R × C` lần lượt theo X, Y, Z. Tọa độ tâm là tâm hình học của từng bộ phận.

| Bộ phận | D × R × C hoặc đường kính × cao (mm) | Tâm (x,y,z) mm | Khối lượng (kg) | Tên CAD/STL hoặc ảnh | Ghi chú |
| --- | --- | --- | --- | --- | --- |
| Base dưới / tấm đáy |đường kính 400, độ dày 5| gốc tọa độ |  |  |  |
| Base trên / tấm trên |đường kính 400, độ dày 5  | gốc tọa độ |  |  |  |
| Trụ đỡ 1 | đường kính 12, độ cao 100 | ví trí (120, 100)  |  |  |  |
| Trụ đỡ 2 | đường kính 12, độ cao 100 | ví trí (-120, 100) |  |  |  |
| Trụ đỡ 3 | đường kính 12, độ cao 100 | ví trí (-120, -100) |  |  |  |
| Trụ đỡ 4 | đường kính 12, độ cao 100 | ví trí (120, -100) |  |  |  |
| Trụ khác (nêu số lượng) |  |  |  |  |  |

| Khoảng cách khác | Giá trị (mm) |
| --- | --- |
| Mặt dưới base dưới tới mặt sàn |  |
| Mặt trên base dưới tới mặt dưới base trên |  |

## 3. Bánh motor và giá/trụ đỡ bánh

Tâm bánh là tâm trục quay. Nếu “trụ đỡ bánh” là **giá giữ motor**, điền ở đây; nếu là **trụ nối hai base**, điền mục 2.

| Bộ phận | Đường kính (mm) | Bề rộng/chiều dài (mm) | Tâm/trục (x,y,z) mm | Khối lượng (kg) | Tên CAD/STL hoặc ảnh |
| --- | --- | --- | --- | --- | --- |
| Bánh motor trái |  |  |  |  |  |
| Bánh motor phải |  |  |  |  |  |
| Motor/moay-ơ trái (nếu hiện hình riêng) |  |  |  |  |  |
| Motor/moay-ơ phải (nếu hiện hình riêng) |  |  |  |  |  |
| Giá/trụ đỡ bánh trái |  |  |  |  |  |
| Giá/trụ đỡ bánh phải |  |  |  |  |  |

| Thông số bánh motor | Giá trị |
| --- | --- |
| Khoảng cách giữa hai tâm bánh trái/phải theo Y (mm) |  |
| Chiều cao tâm bánh so với sàn (mm) |  |
| Hướng trục quay bánh (thường song song Y) |  |
| Kích thước tiết diện giá/trụ đỡ bánh (mm) |  |

## 4. Bánh caster trước và sau

Với caster xoay tự do, trục xoay đứng có thể **lệch khỏi tâm bánh lăn**; nếu đo được hãy ghi cả hai. Tọa độ đều so với O.

| Bộ phận | Loại/model | Đường kính bánh (mm) | Bề rộng bánh (mm) | Tâm bánh lăn (x,y,z) mm | Trục xoay đứng (x,y,z) mm | Tên CAD/STL hoặc ảnh |
| --- | --- | --- | --- | --- | --- | --- |
| Caster trước |  |  |  |  |  |  |
| Caster sau |  |  |  |  |  |  |

| Thông số caster | Caster trước | Caster sau |
| --- | --- | --- |
| Chiều cao từ sàn đến mặt gá (mm) |  |  |
| Độ lệch giữa trục xoay đứng và tâm bánh (mm) |  |  |
| Kích thước đế gá / khoảng cách lỗ bắt vít (mm) |  |  |
| Caster luôn chạm sàn khi xe đứng cân bằng? |  |  |

## 5. Camera, IMU và LiDAR

Tọa độ là tâm hoặc điểm gá đã mô tả, so với O. Góc `(roll,pitch,yaw)` theo độ; nếu chưa biết góc, ghi hướng nhìn và gửi ảnh lắp thật.

| Thiết bị | Model | D × R × C hoặc Ø × C (mm) | Tâm/điểm gá (x,y,z) mm | (roll,pitch,yaw) độ | Hướng nhìn / trục in trên bo | Tên CAD/STL hoặc ảnh |
| --- | --- | --- | --- | --- | --- | --- |
| Camera | D435? |  |  |  |  |  |
| IMU | GY-BNO055? |  |  |  |  |  |
| LiDAR | RPLiDAR C1? |  |  |  |  |  |
| Giá camera |  |  |  |  |  |  |
| Giá IMU (nếu có) |  |  |  |  |  |  |
| Giá LiDAR |  |  |  |  |  |  |

| Mốc phụ và frame cảm biến | Bạn điền |
| --- | --- |
| Tâm thấu kính camera so với điểm gá (mm), nếu biết |  |
| Chiều cao mặt phẳng quét LiDAR so với sàn (mm) |  |
| Trục X/Y/Z in trên IMU hướng về phía nào của xe? |  |
| `header.frame_id` của `/scan` |  |
| `header.frame_id` của camera ảnh/depth |  |
| `header.frame_id` của IMU |  |
| Driver nào đã tự publish TF? Ghi parent → child |  |

## 6. Tệp mô hình và ảnh — cần để làm STL đúng hình

- Có CAD gốc không? [ ] STEP  [ ] SolidWorks  [ ] Fusion 360  [ ] STL rời  [ ] Không có
- Đường dẫn/tên tệp CAD hoặc nơi sẽ gửi:
- Tệp nào là: khung, vỏ, bánh trái, bánh phải, bánh tự do, giá LiDAR, giá camera, hộp điện?:
- Đơn vị xuất mesh CAD/STL: [ ] mm  [ ] m  [ ] chưa biết
- Gốc tọa độ và hướng trục của từng STL đã biết chưa?:
- Có được phép dùng mesh công khai của D435/RPLiDAR (nếu tìm được đúng bản) không?:
- Những ảnh nào thể hiện rõ thước đo hoặc bản vẽ có kích thước?:
- Chi tiết nào phải giống xe thật khi xem RViz; chi tiết nào có thể dùng hình hộp/trụ?:
- Màu mong muốn của khung, bánh và cảm biến:

> Ảnh JPG giúp đối chiếu ngoại hình nhưng không chứa kích thước và hình học 3D đủ tin cậy để suy ra STL chính xác. Nếu chưa có CAD, mình có thể dựng mesh đơn giản theo kích thước bạn đo, rồi thay mesh CAD sau.

## 7. Trạng thái ROS 2 hiện có — giúp ghép vào hệ thống

- Workspace và tên package hiện có (ví dụ `~/ros2_ws/src/...`):
- Hệ chạy: [ ] máy Ubuntu  [ ] Jetson  [ ] Docker  [ ] máy khác:
- Kết quả `ros2 topic list` liên quan đến `/joint_states`, `/odom`, `/scan`, camera, IMU:
- Kết quả `ros2 topic echo /scan --once` (chỉ cần dòng `header.frame_id`):
- Kết quả `ros2 topic echo <imu_topic> --once` (chỉ cần `header.frame_id`):
- Có node nào đang phát `odom → base_link` hoặc `map → odom` chưa? Tên node:
- Có cấu hình Nav2/SLAM/robot_localization/ros2_control hiện tại không? Tên tệp:
- Tên joint bánh và frame đang dùng ở node encoder/driver ZLAC8015D (nếu có):

## 8. Chỉ nếu muốn mô phỏng động lực học bằng Gazebo

- Khối lượng và trọng tâm từng bộ phận (kg, tọa độ mm):
- Mô men quán tính từ CAD (nếu có):
- Vị trí và tính chất bánh tự do / điểm tì sàn:
- Giới hạn tốc độ bánh, gia tốc, ma sát hoặc thông số motor/driver:
- Bạn đang dùng phiên bản Gazebo nào (Fortress, Harmonic, khác)?:

## 9. Ghi chú / thông tin chưa chắc chắn

- Những giá trị chỉ ước lượng từ ảnh:
- Những bộ phận chưa lắp lên xe:
- Mong muốn đặc biệt về mô hình hoặc thứ tự công việc:

---

**Tối thiểu để bắt đầu:** mốc đo mục 1, các bảng bộ phận mục 2–5 và ảnh tổng thể. **Để có STL đúng hình:** thêm CAD gốc hoặc số đo chi tiết ở mục 6. Mục 7–8 có thể bổ sung dần tùy mục tiêu RViz, Nav2 hay Gazebo.
