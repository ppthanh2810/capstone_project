# SAC–DWA — RESEARCH & TRAINING MASTER PLAN

**Phiên bản 1.0 | Đồ án tốt nghiệp — Navigation System**  
**Repository:** [ppthanh2810/capstone_project](https://github.com/ppthanh2810/capstone_project)  
**Phạm vi:** Điều hướng robot di động trong môi trường tĩnh, biết trước một phần; thích nghi trọng số DWA và thời gian dự đoán bằng SAC.

## Cách sử dụng tài liệu

Tài liệu tổng hợp các quyết định đã thống nhất trong quá trình trao đổi, dùng làm tài liệu theo dõi nghiên cứu và đặc tả đầu vào cho Claude Code. Mỗi bước có: **lý thuyết — từ khóa — ví dụ/tại sao — quyết định hoặc công việc — đầu ra nghiệm thu**. Đây không phải báo cáo kết quả thực nghiệm. Bước 8–10 trình bày kế hoạch và kiến thức cần thiết, chưa phải các thông số đã được chốt hoặc kết quả đã đạt.

## A. Bảng theo dõi tiến độ 1–10

| Bước | Nội dung | Trạng thái | Đầu ra cần có |
|---:|---|---|---|
| 1 | Problem Definition | Đã chốt | Bài toán, phạm vi, hai phương pháp |
| 2 | Research Hypothesis | Đã chốt | H1, H2, H3, H5, H6 |
| 3 | System Characterization | Giao Claude Code; chưa nghiệm thu | Improved DWA, cấu hình robot thống nhất |
| 4 | Scenario Taxonomy | Đã chốt định hướng; thông số cụ thể giao Claude | 10 loại điều hướng + S1 an toàn |
| 5 | Scenario Generation | Đã chốt phương pháp; chưa triển khai | Generator, bộ kiểm tra, scheduler |
| 6 | Metrics and Protocol | Đã chốt nguyên tắc; một số ngưỡng chờ pilot | Metric, episode termination, validation, BM1–BM5 |
| 7 | Dataset Splitting | Đã chốt quy mô; chưa sinh dữ liệu | 2.140 scenario cố định + online + S1 riêng |
| 8 | Baseline Evaluation | Chưa thực hiện | DWA cố định được tối ưu và benchmark |
| 9 | Optimization and Training | Chưa thực hiện | SAC tổng quát, model thích nghi, kết quả mô phỏng |
| 10 | Real-world Validation | Chưa thực hiện | Kết quả thật, sim-to-real, kết luận |

**Tiến độ thiết kế:** 6/6 bước đã được thảo luận và thống nhất về định hướng (1, 2, 4, 5, 6, 7). Bước 3 tạm giao Claude. Không hiểu tỷ lệ này là 100% khối lượng đồ án hay đã hoàn thành code, dữ liệu và training.

---

## 1. Problem Definition — Định nghĩa bài toán

### Lý thuyết và từ khóa

- **Problem Definition:** xác định chính xác vấn đề, mục tiêu, đầu vào, đầu ra và giới hạn nghiên cứu.
- **DWA/DWB:** lấy mẫu vận tốc, dự đoán quỹ đạo trong một khoảng thời gian, loại quỹ đạo không an toàn và chấm điểm các quỹ đạo còn lại.
- **Adaptive Parameter Learning:** RL học cách điều chỉnh tham số planner thay vì thay thế planner.
- **Partially Known / Stale Map:** bản đồ tĩnh đã lưu không chứa mọi vật cản tồn tại trong môi trường thực thi. Vật cản mới ở đây đứng yên trong suốt episode.
- **Prediction Horizon / Simulation Time (`s_time`):** thời gian dự đoán quỹ đạo của DWA, không phải thời gian chạy episode.

### Ví dụ — Tại sao cần?

Một bộ trọng số phù hợp với phòng rộng có thể không phù hợp khi robot gặp khe hẹp hoặc hộp chắn đường. Horizon ngắn có thể phản ứng cục bộ tốt nhưng nhìn chưa đủ xa; horizon dài có thể dự đoán tốt hơn nhưng tốn tính toán hoặc vượt phạm vi costmap. SAC sẽ học cách chọn kết hợp tham số phù hợp; đây là giả thuyết, không phải đảm bảo.

### Quyết định đã chốt

**Bài toán:** Cải thiện khả năng điều hướng và tránh vật cản của DWA bằng SAC điều chỉnh thích nghi năm trọng số và thời gian dự đoán trong môi trường tĩnh biết trước một phần.

| Tham số | M0 — Optimized DWA | M1 — SAC-DWA |
|---|---|---|
| P — Path | Cố định | Thích nghi |
| G — Goal | Cố định | Thích nghi |
| O — Obstacle | Cố định | Thích nghi |
| H — Heading | Cố định | Thích nghi |
| S — Speed | Cố định | Thích nghi |
| T_sim / s_time | Cố định | Thích nghi |
| Oscillation weight (nếu có trong code cuối) | Cố định | Cố định |

Biểu diễn định hướng: `J = wP·P + wG·G + wO·O + wH·H + wS·S + wosc·Osc` (xác nhận tên và công thức critic theo code cuối).

Action của SAC: `a_t = [wP, wG, wO, wH, wS, T_sim]`.

```text
LiDAR + Costmap + Path + Odometry (state)
                    ↓
               SAC policy
                    ↓
       5 weights + prediction horizon
                    ↓
    Improved DWA + hard safety constraints
                    ↓
             v*, omega* → Robot
```

Ba nhóm môi trường: N — không có vật cản mới (map và world khớp); B — vật cản mới ngoài map; C — môi trường tĩnh phức tạp (khe hẹp, nhiều vật cản).

Giới hạn: không vật cản động, không RL trực tiếp xuất vận tốc, không thay A* hoặc DWA bằng mạng neural, không nghiên cứu cải tiến SLAM/localization như đóng góp chính. Hai phương pháp dùng chung navigation, robot model, replanning và safety. Chỉ so sánh hai phương pháp; không làm ablation weights-only/horizon-only để giảm khối lượng đồ án. Vì vậy không được kết luận thành phần nào đóng góp nhiều hơn.

**Đầu ra:** đặc tả bài toán và hai phương pháp đối chứng. **Trạng thái: đã chốt.**

## 2. Research Hypothesis — Giả thuyết nghiên cứu

### Lý thuyết và từ khóa

**Research Hypothesis** là dự đoán có thể kiểm chứng; **Baseline** là phương pháp đối chứng; **Fair Comparison** là giữ các yếu tố ngoài biến nghiên cứu giống nhau. Giả thuyết không phải kết luận định sẵn.

### Ví dụ — Tại sao cần?

Nếu SAC-DWA đến đích nhiều hơn DWA nhưng va chạm nhiều hơn, không thể chỉ nhìn Success Rate rồi tuyên bố tốt hơn. Nếu chỉ so với DWA mặc định chưa tinh chỉnh, cải thiện có thể do baseline yếu.

### Quyết định đã chốt

| ID | Giả thuyết | Chỉ số ưu tiên |
|---|---|---|
| H1 | Duy trì hiệu suất điều hướng danh định | SR, CR |
| H2 | Nâng cao khả năng tránh vật cản tĩnh mới | SR, CR, STR |
| H3 | Nâng cao hiệu quả trong môi trường tĩnh phức tạp | SR, CR, STR |
| H5 | Duy trì lợi ích khi chuyển mô phỏng sang robot thật | SR, CR, SSR, STR |
| H6 | Giảm chi phí tinh chỉnh khi chuyển môi trường | Adaptation Episodes, Time, Human Tuning Time |

H4 (generalization) không còn là giả thuyết chính độc lập nhưng vẫn kiểm tra zero-shot để làm cơ sở cho fine-tuning và H6. H1 ở đây tập trung vào thành công và va chạm; không mặc nhiên chứng minh tốc độ hoặc độ mượt tương đương nếu không phân tích các log phụ. Ngưỡng khác biệt có ý nghĩa và điều kiện đạt mục tiêu thích nghi cần xác định trước final test.

**Đầu ra:** bảng giả thuyết, biến nghiên cứu và chỉ số tương ứng. **Trạng thái: đã chốt.**

## 3. System Characterization — Chuẩn hóa hệ thống

### Lý thuyết và từ khóa

**Robot Footprint**, **Kinematic Model**, **Velocity/Acceleration Limits**, **Latency**, **Braking**, **LiDAR FOV**, **Local Costmap**, **Inflation**, **Carrot Selection**, **Path Blockage Detection**, **Safe Stop**, **Global Replanning**, **Regression Test**, **Single Source of Truth**.

System characterization là xác định cách robot thực sự vận hành và giới hạn của nó. Các đặc tính này quyết định thế nào là một khe có thể đi qua, horizon nào hợp lệ và mô phỏng phải phản ánh điều gì. Đồng nhất không có nghĩa simulator giống thật tuyệt đối, mà là các tham số quan trọng được đo, dùng chung và kiểm chứng.

### Ví dụ — Tại sao cần?

Nếu footprint trong A* nhỏ hơn footprint collision checking của DWA, A* có thể tạo đường qua khe mà DWA không thể đi. SAC bị phạt vì lỗi kiến trúc, không phải vì chọn trọng số sai. Tương tự, nếu motor thật phản ứng chậm hơn mô phỏng, quỹ đạo được dự đoán an toàn có thể không còn an toàn.

### Checklist Claude Code

- [ ] Audit code hiện tại, xác định cấu hình và luồng dữ liệu thực tế; ghi commit trước/sau.
- [ ] Hoàn thiện Carrot Selection: điểm cục bộ hợp lệ, không nằm trong vật cản.
- [ ] Hoàn thiện Path Blockage Detection: phát hiện tuyến bị chặn.
- [ ] Hoàn thiện Safe Stop: dừng khi không còn quỹ đạo an toàn hoặc dữ liệu quan trọng không hợp lệ.
- [ ] Hoàn thiện Global Replanning: sử dụng thông tin vật cản đã quan sát để tìm đường thay thế; xử lý không còn đường.
- [ ] Đồng nhất footprint, giới hạn vận tốc/gia tốc, braking/ramp/latency, LiDAR, costmap, tần số và hệ tọa độ.
- [ ] Kiểm tra obstacle marking/clearing, unknown cells, dữ liệu cũ, giới hạn dự đoán và chi phí tính toán.
- [ ] Xác định thông số hình học generator, độ khó và episode thresholds từ cấu hình cuối cùng (bước 4.3, 4.4, 6.2).
- [ ] Regression test N/B/C/S1; đóng băng phiên bản Improved DWA dùng chung cho M0 và M1.

Không cần mô hình điện động cơ chi tiết; giữ mô hình motor đơn giản nhưng đo/kiểm tra những đặc tính ảnh hưởng đến điều hướng. Không cho SAC thay đổi các ràng buộc an toàn cứng.

**Đầu ra:** cấu hình robot chuẩn, Improved DWA đã nghiệm thu, log đo, Git commit và báo cáo regression. **Trạng thái: giao Claude Code; chưa xác nhận hoàn thành.**

## 4. Scenario Taxonomy — Phân loại môi trường và kịch bản

### Lý thuyết và từ khóa

- **Layout:** hình dạng phòng, tường, hành lang và cấu trúc cố định.
- **Static Map (M):** bản đồ mà hệ thống lập kế hoạch biết trước.
- **World (W):** môi trường thực thi chứa các vật cản thật.
- **Scenario:** layout/world/map + start, goal, hướng xuất phát và điều kiện nhiệm vụ.
- **Episode:** một lần thực hiện scenario; cùng scenario có thể chạy nhiều episode.
- **Procedural Generation / BARN-inspired:** sinh nhiều môi trường có kiểm soát bằng thuật toán; BARN dùng Cellular Automata để tạo bố trí vật cản đa dạng.
- **Difficulty Classification:** phân tầng độ khó từ hình học và đặc tính nhiệm vụ; không dùng kết quả SAC để tự gán nhãn độ khó ban đầu.

### Ví dụ — Tại sao cần?

Cùng một phòng có thể tạo ba nhiệm vụ: đi bình thường; xuất hiện hộp mới chắn đường A*; hoặc nhiều hộp tạo khe hẹp. Nếu chỉ training trong một căn phòng, policy có thể học một số bố trí quen thuộc thay vì quy luật thích nghi.

### 4.1. Layout Selection — Đã chốt

Training tổng quát: sinh môi trường đa dạng theo ý tưởng BARN, kết hợp bố trí hình học có chủ đích (phòng, hành lang, góc rẽ, nhiều tuyến). Không dùng House và phòng học để pre-train. Hai map mục tiêu của repo là phòng học trong `maps/` và House: kiểm tra zero-shot, fine-tune độc lập từ cùng checkpoint tổng quát, rồi final test trên scenario chưa học. Tên file và hình học map cụ thể cần Claude xác nhận.

### 4.2. Danh mục scenario — Đã chốt

| Nhóm | ID | Tình huống | Năng lực kiểm tra |
|---|---|---|---|
| Nominal | N1 | Đường thẳng | Điều hướng cơ bản |
| Nominal | N2 | Góc rẽ | Quay, bám đường |
| Nominal | N3 | Nhiều góc rẽ | Điều hướng nhiều đoạn |
| New obstacle | B1 | Một vật cản mới | Đi vòng |
| New obstacle | B2 | Vật cản mới kích thước lớn | Chọn quỹ đạo thay thế |
| New obstacle | B3 | Chặn tuyến, còn đường thay thế | Phục hồi và replanning |
| Complex | C1 | Khe hẹp | Điều hướng với clearance nhỏ |
| Complex | C2 | Nhiều vật cản | Xử lý liên tiếp |
| Complex | C3 | Zigzag | Đổi hướng nhiều lần |
| Complex | C4 | BARN-like | Thích nghi với bố trí đa dạng |
| Safety | S1 | Không còn tuyến khả thi | Safe Stop |

N1–C4 dùng cho training và test; S1 đánh giá riêng. Không ép mọi kịch bản xuất hiện trên cả hai map nếu hình học không phù hợp. Các nhóm chính là nhãn thiết kế; giữ thêm nhãn known/unknown đối với C để phân tích tránh lẫn tác động hình học và thông tin bản đồ.

### 4.3. Thông số sinh môi trường — Giao Claude sau bước 3

Kích thước phòng, chiều rộng hành lang, số lượng/kích thước/hình dạng/góc xoay vật cản, vị trí start/goal, mật độ, khoảng hở, góc rẽ. Nên định nghĩa tương đối theo footprint khi phù hợp, sau đó chuyển sang mét từ cấu hình thực. Không đóng đinh số đo giả định trước khi xác minh robot.

### 4.4. Độ khó và Curriculum — Đã chốt nguyên tắc

Các yếu tố độ khó: clearance, obstacle density trên vùng nhiệm vụ, số góc rẽ/độ cong đường, detour ratio (độ dài đường vòng so với đường tham chiếu), visibility/occlusion. Claude xác định công thức/ngưỡng Easy–Medium–Hard sau bước 3 và pilot; không bắt buộc mọi loại có đủ cả ba mức.

| Giai đoạn | Easy | Medium | Hard |
|---|---:|---:|---:|
| 1 — Học cơ bản | 80% | 20% | 0% |
| 2 — Học tránh vật cản | 30% | 50% | 20% |
| 3 — Học tổng quát | 33% | 34% | 33% |

**Curriculum Learning**: học từ dễ đến khó nhưng vẫn duy trì dữ liệu dễ ở giai đoạn sau. Tỷ lệ là xác suất lấy mẫu episode, không phải tỷ lệ gradient update. SAC tiếp tục học cùng model, không reset neural network. Unsolvable không đưa vào training điều hướng thông thường; dành cho kiểm tra Safe Stop.

**Đầu ra:** Scenario Taxonomy 10+1, chiến lược layout, quy tắc độ khó và curriculum. **Trạng thái: đã chốt định hướng; giá trị kỹ thuật giao Claude.**

## 5. Scenario Generation — Sinh và kiểm tra môi trường

### Lý thuyết và từ khóa

**Rule-based Generation** tạo tình huống cụ thể; **BARN-style Generation** tạo bố trí đa dạng kiểu BARN; **Offline Generation** sinh trước và lưu dữ liệu; **Online Generation** sinh mới khi training; **Hybrid Generation** kết hợp hai nguồn; **Stratified Sampling** lấy mẫu theo loại và độ khó; **Feasibility Checking** kiểm tra đường khả thi; **Random Seed / Reproducibility** giúp tái lập.

Offline/online ở đây nói về cách sinh scenario, không phải offline RL/online RL: SAC vẫn thu thập transition bằng tương tác với môi trường mô phỏng.

### Ví dụ — Tại sao cần?

Sinh ngẫu nhiên hộp ở một phòng robot không đi qua có thể không tạo giá trị training. Sinh vật cản dựa trên tuyến A* ban đầu giúp đảm bảo robot thật sự gặp tình huống cần học; thêm bố trí ngẫu nhiên kiểu BARN để tránh quá khớp với các mẫu cố định.

### 5.1. Hybrid Generation — Đã chốt

70% episode lấy từ bộ offline đã kiểm tra; 30% lấy từ generator online. Dùng ở cả ba giai đoạn curriculum; fine-tuning trên từng map cũng dùng cấu hình này làm mặc định đã thống nhất. Validation/test 100% kịch bản offline, khóa trước.

```text
Chọn difficulty theo curriculum
 → Chọn scenario type hợp lệ, kiểm soát độ bao phủ
 → Chọn nguồn offline (70%) / online (30%)
 → Lấy hoặc sinh scenario và kiểm tra tính khả thi
 → Reset simulator → SAC tương tác → Ghi log → Episode tiếp theo
```

Có thể dùng buffer môi trường online được sinh/kiểm tra trước khi reset để giảm chi phí. Ghi tỷ lệ thực tế theo nguồn, loại, độ khó; giới hạn số lần thử sinh, báo lỗi và dùng fallback cùng nhóm nếu generator không sinh được, không âm thầm thay đổi phân phối.

### 5.2. Tách Map và World

`W = M ⊕ Δ` (Δ là tập vật cản mới được thêm vào world).

| Nhóm | Quy tắc |
|---|---|
| N | M = W |
| B | W có vật cản mới không nằm trong M |
| C | Có biến thể vật cản đã biết và chưa biết |
| S1 | M có đường ban đầu nhưng W bị chặn hết tuyến khả thi, hoặc đặc tả vô nghiệm phù hợp |

A* ban đầu dùng M; replanning chỉ sử dụng dữ liệu hệ thống thực sự quan sát được. LiDAR mô phỏng quan sát W với giới hạn sensor. Ground truth W dành cho generator, evaluator và reward nếu cần; không cấp thông tin vật cản chưa quan sát cho policy hoặc planner.

### 5.3. Pipeline generator đề xuất

1. Sinh layout hoặc tải map mục tiêu; chọn start, goal và hướng.
2. Lập đường A* trên M; tạo vật cản theo quy tắc và/hoặc BARN-inspired.
3. Kiểm tra start/goal không va chạm, đường tồn tại trong W với footprint phù hợp (A*/BFS trên grid đã xét footprint là kiểm tra hình học, không chứng minh đầy đủ khả thi động học).
4. Xác nhận đúng nhãn: B3 còn tuyến thay thế nhưng tuyến ban đầu bị chặn; S1 không có đường phù hợp; loại kịch bản sai mục tiêu.
5. Tính difficulty và lưu `scenario_id`, seed, layout ID, map/world, vật cản, start/goal, type, difficulty, split, config/version/hash.
6. Dùng cùng mô tả scenario cho Fast 2D Simulator và Gazebo. Kiểm tra tương ứng trước training lớn.

**Đầu ra:** generator, feasibility/difficulty checker, dataset manifest, curriculum sampler và cơ chế replay. **Trạng thái: phương pháp đã chốt; code chưa triển khai.**

## 6. Metrics and Protocol — Chỉ số và giao thức đánh giá

### Lý thuyết và từ khóa

**Metric**, **Episode Termination**, **Validation**, **Final Test**, **Checkpoint**, **Early Stopping**, **Zero-shot Transfer**, **Fine-tuning**, **Adaptation Cost**, **Paired Evaluation**. Reward hướng dẫn quá trình học; metric độc lập để đánh giá thuật toán. Validation dùng chọn model; final test chỉ dùng đánh giá sau khi đã đóng băng.

### Ví dụ — Tại sao cần?

Robot đứng yên khi không còn đường là hành vi an toàn; đứng yên dù còn đường và không phục hồi được là mắc kẹt. Nếu tính cả hai là lỗi đến đích, kết quả sẽ không phản ánh đúng năng lực. Nếu vừa học vừa test cùng dữ liệu thì kết quả bị thiên lệch.

### 6.1. Năm chỉ số chính — Đã chốt

Gọi Nf là số episode khả thi hợp lệ, Nu là số episode vô nghiệm hợp lệ.

| Metric | Công thức / cách đo | Mục tiêu |
|---|---|---|
| Success Rate (SR) | N_success / Nf × 100% | Tăng |
| Collision Rate (CR) | N_episode có collision / (Nf + Nu) × 100% | Giảm |
| Safe Stop Rate (SSR) | N_safe_stop / Nu × 100% | Tăng |
| Stuck Rate (STR) | N_stuck / Nf × 100% | Giảm |
| Adaptation Cost (AC) | Số episode/transition bổ sung, wall-clock/compute time, human tuning time để đạt cùng mục tiêu | Giảm |

CR báo cáo cả trên các tập riêng và tổng hợp; một episode có nhiều va chạm vẫn tính một episode collision. Trong validation điều hướng chỉ có Nf thì Nu = 0; S1 đánh giá riêng, không pha để làm giảm CR của tập khả thi. Ghi thêm time, path length, clearance, computation time, oscillation, replan count để phân tích lỗi, không bắt buộc là chỉ số chính. Tính SR cân bằng theo nhóm khi chọn checkpoint (cách tính cụ thể chốt trước chạy).

### 6.2. Episode Termination — Đã chốt sáu nhãn

| Nhãn | Điều kiện |
|---|---|
| SUCCESS | Đến goal hợp lệ, không va chạm |
| COLLISION | Thân robot tiếp xúc vật cản thực; ưu tiên nhãn này nếu va chạm đã xảy ra |
| STUCK | Không tiến triển trong ngưỡng quy định, tồn tại đường nhưng không phục hồi |
| SAFE_STOP | Vô nghiệm, xử lý đường bị chặn và dừng an toàn đúng quy trình |
| TIMEOUT | Hết thời gian nhiệm vụ, không thuộc trạng thái kết thúc khác |
| SYSTEM_ERROR | Lỗi hệ thống ngăn episode tiếp tục |

Claude xác định goal tolerance, stuck window, timeout, vận tốc dừng, thời gian giữ dừng và cách phát hiện va chạm từ bước 3. Lỗi hạ tầng ngoại sinh ghi riêng và chạy lại theo quy tắc chung; lỗi do thuật toán không tùy ý loại bỏ. Ghi các sự kiện trung gian để phân biệt dừng tạm thời/replan thành công với STUCK cuối cùng.

### 6.3. Curriculum Transition — Đã chốt nguyên tắc

| Validation | GĐ1 | GĐ2 | GĐ3 |
|---|---:|---:|---:|
| SR Easy | ≥95% | ≥95% | ≥95% |
| SR Medium | ≥80% | ≥90% | ≥90% |
| SR Hard | — | ≥70% | ≥85% |
| CR | ≤2% | ≤2% | ≤2% |
| Lần đạt liên tiếp | 3 | 3 | 3 |

Tất cả validation mô phỏng (bao gồm fine-tuning) thống nhất **CR ≤ 2%**, không giảm thành 1% ở giai đoạn cuối. Đây là ngưỡng tỷ lệ quan sát, không phải bảo đảm an toàn; không áp dụng như mức va chạm cho phép trên robot thật. Các ngưỡng SR còn là mục tiêu khởi đầu, xác nhận theo pilot và baseline trước training chính thức. Báo cáo thêm CR theo nhóm/mức khó để phát hiện rủi ro bị che khuất bởi số trung bình.

Training một khoảng → checkpoint → chạy validation cố định với policy đóng băng (không gradient, không đưa dữ liệu validation vào replay buffer) → kiểm tra điều kiện → chuyển giai đoạn nếu đạt ba lần liên tiếp. Có ngân sách tối thiểu/tối đa và tần suất validation; giá trị cụ thể chưa chốt.

### 6.4. Evaluation Protocol — BM1–BM5 đã chốt

| Benchmark | Mục tiêu | Thiết kế |
|---|---|---|
| BM1 General Test | Hiệu quả tổng quát | DWA tối ưu và SAC general trên 300 scenario test độc lập |
| BM2 Zero-shot | Hiệu quả trước thích nghi | DWA general và SAC general trên Classroom/House, không cập nhật |
| BM3 Adapted Simulation | Hiệu quả sau thích nghi và AC | DWA retune và SAC fine-tune từ cùng bản general tương ứng; final test độc lập |
| BM4 Initial Real | Xác định sim-to-real gap | Chạy kiểm tra an toàn có giám sát, ghi log để hiệu chỉnh |
| BM5 Final Real | Kết luận thực nghiệm | Đóng băng tham số/model, chạy 60 episode cuối |

Cùng navigation architecture, robot, safety, start/goal, vật cản, timeout, kịch bản thích nghi và điều kiện đánh giá. Fine-tuning từng map độc lập; DWA được quyền retune weights + s_time trên dữ liệu thích nghi tương ứng, không dùng baseline yếu. Cùng mục tiêu hiệu suất và giới hạn ngân sách; ghi riêng số episode, transition, wall-clock, compute và thời gian người. Nếu không đạt mục tiêu trong giới hạn, ghi nhận chưa đạt. Ghi cả chi phí pre-training SAC ban đầu và chi phí tối ưu DWA ban đầu; không đánh đồng chi phí ban đầu và chi phí chuyển môi trường.

BM2 phục vụ phát triển dùng tập validation/phát triển của map mục tiêu, không mở final test. Để có so sánh trước–sau trên cùng tập test, có thể lưu checkpoint trước/sau rồi chạy cả hai trên final test sau khi đóng băng toàn bộ quyết định; đây là quy tắc triển khai bổ sung để tránh data leakage, không phải một bộ test dùng để tinh chỉnh.

### 6.5. Training Termination / Model Selection — Đã chốt

Dừng khi đạt toàn bộ mục tiêu (GĐ3 đạt ba validation liên tiếp), early stopping khi không cải thiện hoặc hết ngân sách. Nếu chưa đạt, báo cáo chưa đạt; không kéo dài vô hạn hoặc hạ mục tiêu hậu nghiệm. Chọn model bằng validation: loại checkpoint CR >2%; ưu tiên SR cân bằng cao, nếu tương đương thì STR thấp; bắt buộc vượt kiểm tra Safe Stop riêng trước triển khai thật. Không chọn chỉ theo total reward. Nếu không có checkpoint đáp ứng an toàn, không đưa lên robot thật.

Lưu checkpoint `sac_stage1`, `sac_stage2`, `sac_general_best`, `sac_general_last`, `sac_classroom_best`, `sac_house_best`, `sac_real_best` (nếu cần), cùng config, seed, training steps, validation metrics, normalizer và trạng thái cần thiết để tiếp tục training (optimizer/replay buffer nếu triển khai yêu cầu).

**Đầu ra:** bộ metric, episode evaluator, validation gate, checkpoint selector, giao thức BM1–BM5. **Trạng thái: nguyên tắc đã chốt; ngưỡng kỹ thuật, ngân sách, ngưỡng SR chờ pilot.**

## 7. Dataset Splitting — Phân chia dữ liệu

### Lý thuyết và từ khóa

**Train Set** dùng cập nhật model/tinh chỉnh; **Validation Set** dùng kiểm tra tiến độ và chọn checkpoint; **Test Set** giữ độc lập cho đánh giá cuối; **Data Leakage** xảy ra nếu thông tin test ảnh hưởng training hoặc tuning; **Holdout**, **Random Seed**, **Scenario Group Split**, **OOD/Zero-shot**.

### Ví dụ — Tại sao cần?

Nếu cùng một layout và gần như cùng vị trí hộp chỉ đổi seed được đưa vào cả train và test, kết quả có thể đo khả năng ghi nhớ thay vì tổng quát hóa. Cần chia theo nhóm layout/biến thể liên quan, không chỉ theo tên file hoặc seed. Trên House và phòng học layout cố định, tách theo nhóm bố trí vật cản và nhiệm vụ start/goal khác nhau.

### 7.1. Quy mô cân bằng — Đã chốt

| Dataset | Offline Train/Fine-tune | Validation | Final Test | Tổng |
|---|---:|---:|---:|---:|
| General | 900 | 300 | 300 | 1.500 |
| Classroom | 120 | 100 | 100 | 320 |
| House | 120 | 100 | 100 | 320 |
| Tổng | 1.140 | 500 | 500 | **2.140** |

Không bao gồm scenario online và các scenario S1 riêng. 70/30 là tỷ lệ lấy mẫu từ offline/online trong training, không phải tỷ lệ chia train/validation/test. Số scenario cũng khác số episode và số training steps.

### 7.2. Phân tầng 900 scenario general offline (phân bổ khởi đầu đã đề xuất)

| ID | Easy | Medium | Hard | Tổng |
|---|---:|---:|---:|---:|
| N1 | 60 | 30 | 0 | 90 |
| N2 | 40 | 30 | 20 | 90 |
| N3 | 30 | 30 | 30 | 90 |
| B1 | 40 | 30 | 20 | 90 |
| B2 | 30 | 30 | 30 | 90 |
| B3 | 20 | 30 | 40 | 90 |
| C1 | 20 | 30 | 40 | 90 |
| C2 | 20 | 30 | 40 | 90 |
| C3 | 20 | 30 | 40 | 90 |
| C4 | 20 | 30 | 40 | 90 |
| Tổng | 300 | 300 | 300 | 900 |

Claude cần kiểm tra tổ hợp type/difficulty hợp lý rồi điều chỉnh nếu cần, ghi lại thay đổi. Đề xuất triển khai 300 general validation và 300 general test cân bằng theo độ khó (100 mỗi mức) và bao phủ các loại hợp lệ. Từng map mục tiêu 100 val/100 test phân tầng theo các loại có thể dựng thật. Số S1 riêng chưa chốt; tách cả S1 dùng nghiệm thu/chọn model và S1 final test để giữ độc lập.

### 7.3. Quy tắc chống rò rỉ và kiểm soát dữ liệu

- Khóa manifest, seed namespace, layout family/instance và cấu hình val/test trước tuning chính thức; online generator không được trùng hoặc tạo bản sao gần của holdout theo quy tắc đã định.
- Không dùng House/Classroom trong pre-training; trên từng map tách các tập fine-tuning, validation, final test.
- Mỗi model được đánh giá cùng kịch bản với đối chứng. Không loại kịch bản vì một phương pháp không vượt qua.
- Pilot simulator và kiểm tra tỷ lệ sinh hợp lệ trước khi sinh toàn bộ 2.140 scenario. Các con số là ngân sách thiết kế, không phải chuẩn bắt buộc hay đảm bảo hội tụ SAC.

### 7.4. Dataset D — Robot thật

| ID | Kịch bản đại diện |
|---|---|
| R1 | Điều hướng bình thường |
| R2 | Một vật cản mới |
| R3 | Chặn tuyến, cần replan |
| R4 | Lối đi hẹp |
| R5 | Nhiều vật cản |
| R6 | Vô nghiệm — Safe Stop |

6 kịch bản × 5 lần lặp × 2 phương pháp = **60 episode final real test**, gồm 30 mỗi phương pháp (25 khả thi + 5 vô nghiệm). Chưa gồm BM4 và các đợt hiệu chỉnh. Đây là quy mô kiểm chứng kỹ thuật ban đầu, mẫu nhỏ đối với kết luận thống kê về độ an toàn.

**Đầu ra:** manifests General/Classroom/House, danh sách S1 độc lập, protocol thực tế. **Trạng thái: quy mô đã chốt; dataset chưa sinh.**

## 8. Baseline Evaluation — Đánh giá DWA truyền thống

### Lý thuyết và từ khóa

**Baseline**, **Parameter Tuning**, **Bayesian Optimization (BO)**, **Grid/Random Search**, **Pilot Experiment**, **Fair Budget**, **Regression**, **Failure Analysis**. Baseline tốt là đối chứng được tối ưu hợp lý, không phải cấu hình yếu để làm nổi bật RL. BO là một lựa chọn đề xuất: dựa trên kết quả thử trước để chọn bộ tham số tiếp theo, không phải phương pháp đã bắt buộc chốt.

### Ví dụ — Tại sao cần?

Nếu DWA mặc định chỉ đạt SR 60%, nhưng một bộ weights và s_time cố định được tinh chỉnh đạt 90%, thì việc SAC đạt 85% không chứng minh ưu thế so với DWA đã tối ưu.

### Kế hoạch thực hiện (chưa chốt chi tiết)

1. Nghiệm thu Improved DWA và kiểm chứng simulator trước (cùng DwaCore, cách tạo quỹ đạo, sensor/costmap, replanning; so kết quả pilot 2D với Gazebo).
2. Chạy DWA hiện tại trên tập phát triển, ghi các failure mode.
3. Xác định khoảng tìm kiếm 5 weights và s_time theo code, footprint, costmap, deadline; giữ Oscillation và safety chung.
4. Tối ưu bộ tham số cố định trên dữ liệu train/development, chọn bằng validation. BO hoặc phương pháp hệ thống phù hợp ngân sách; không dùng final test để chọn tham số.
5. Đóng băng `dwa_general_best`; xác nhận mục tiêu SR và ngân sách ở bước 6 từ pilot; lưu kết quả theo nhóm và mức khó. BM1–BM3 so sánh với SAC sau khi model sẵn sàng.

Khi sang map mới, retune DWA trên dữ liệu thích nghi tương ứng để đối chứng H6. Không đổi weights theo từng scenario trong final test.

**Đầu ra:** cấu hình DWA cố định tối ưu, báo cáo baseline, log thất bại, ngân sách và miền tham số. **Trạng thái: chưa thực hiện.**

## 9. Optimization and Training — Thiết kế và huấn luyện SAC-DWA

### Lý thuyết và từ khóa

- **MDP / POMDP:** trạng thái môi trường, hành động, phần thưởng và chuyển trạng thái; robot chỉ quan sát một phần thế giới, nên state đưa vào policy thực chất là observation.
- **Actor-Critic:** Actor (mạng policy) sinh hành động; Critic (mạng Q) đánh giá lợi ích dài hạn của hành động trong trạng thái.
- **SAC (Soft Actor-Critic):** RL off-policy cho action liên tục, tối ưu reward tích lũy kết hợp entropy để khuyến khích khám phá. Replay Buffer lưu transition `(s, a, r, s_next, done)`; hai critic và target networks giúp cập nhật ổn định hơn, không bảo đảm hội tụ.
- **Reward Shaping:** tạo tín hiệu học, không đồng nhất reward với metric cuối.
- **Pre-training / Transfer Learning / Fine-tuning / Zero-shot:** học tổng quát, chuyển checkpoint, học bổ sung hoặc chạy trực tiếp không cập nhật.
- **Sim Fidelity / Fast 2D / Gazebo:** huấn luyện số lượng lớn trong simulator nhanh, kiểm chứng toàn stack bằng Gazebo; phải xác nhận độ tương đồng đủ dùng trước training lớn.

### Ví dụ — Tại sao cần?

LiDAR thấy hộp chắn, path hướng thẳng vào hộp, carrot ở phía sau hộp. Actor xuất sáu tham số; DWA dùng tham số đó để chọn quỹ đạo hợp lệ. Nếu robot tiến triển an toàn, reward phản ánh lợi ích; nếu mắc kẹt/va chạm, reward phản ánh hậu quả. Critic học từ transition và Actor được cập nhật để chọn tham số có lợi hơn trong tình huống tương tự. SAC không nhận quyền bỏ qua kiểm tra collision.

### 9.1. Thiết kế RL cần chốt sau bước 3 và 8

| Thành phần | Định hướng đề xuất, chưa chốt số |
|---|---|
| Observation | LiDAR sectors, path/carrot, costmap đặc trưng, odometry, trạng thái blockage/replan, action trước nếu hữu ích; chỉ thông tin thật có thể đo |
| Action | 5 weights + T_sim liên tục; ánh xạ action vào miền hợp lệ, giữ Oscillation cố định |
| Reward | Tiến triển đến goal, thành công, phạt va chạm/mắc kẹt, thời gian, clearance, dao động/thay đổi tham số nếu cần; không dùng trực tiếp J của DWA làm mục tiêu duy nhất |
| Safety | Hard collision/braking/footprint/limits độc lập RL; action clipping, horizon hợp lệ, fallback khi policy lỗi |
| Update rate | SAC chậm hơn hoặc bằng DWA; chọn sau đo latency, không chốt tần số trước |
| Training budget | Số transition, seed, batch size, learning rate, replay buffer, validation interval, patience; xác định qua pilot |

Cần kiểm tra việc thay đổi horizon không làm thiên lệch critic do số điểm rollout khác nhau; kiểm tra độ nhạy và scale của weights, không tự thay đổi công thức J giữa M0 và M1. Định nghĩa chính xác `s_time` trong code, giới hạn rollout trong costmap và deadline điều khiển. Reward dùng ground truth nếu cần trong training nhưng policy inference không được nhìn dữ liệu không có trên robot thật.

### 9.2. Quy trình training và transfer đã thống nhất

```text
Fast 2D simulator đã kiểm chứng với Gazebo
  ↓
SAC pre-training: 900 offline + 30% online episodes
  ↓
Curriculum GĐ1 → GĐ2 → GĐ3 (validation gate)
  ↓
SAC_GENERAL_BEST
  ├─ BM1: General Test
  ├─ Classroom: zero-shot → fine-tune riêng → validation → final test
  └─ House:     zero-shot → fine-tune riêng → validation → final test
  ↓
SAC_CLASSROOM_BEST → robot thật (BM4)
```

Tỷ lệ chính xác của dòng pre-training: 70% episode lấy từ 900 scenario offline, 30% episode online. Hai nhánh Classroom/House bắt đầu từ cùng general checkpoint, không học nối tiếp qua hai map. DWA có quy trình retune tương ứng. Nếu zero-shot đã đạt yêu cầu thì có thể không cần fine-tuning. Đóng băng model trước final test, không cập nhật khi chạy benchmark.

**Đầu ra:** Gymnasium environment hoặc giao diện tương đương, observation/action/reward spec, SAC training pipeline, checkpoints, learning curves, kết quả BM1–BM3. **Trạng thái: chiến lược đã thống nhất; thiết kế RL chi tiết và training chưa thực hiện.**

## 10. Real-world Validation — Kiểm chứng trên robot thật

### Lý thuyết và từ khóa

**Sim-to-Real Gap**, **Calibration**, **Domain Randomization**, **Digital Twin**, **Real-world Adaptation**, **Paired Runs**, **Confidence Interval**, **External Validity**. Mô phỏng khác robot thật về trễ, tăng tốc/phanh, sai số localization, LiDAR, costmap và timing. Fine-tune không sửa được mọi lỗi hệ thống; cần tìm nguyên nhân trước.

### Ví dụ — Tại sao cần?

SAC-DWA né hộp trong simulator nhưng robot thật dừng muộn do motor ramp/latency cao hơn. Giải pháp trước tiên là đo trễ/phanh và cập nhật simulator; nếu cần fine-tune lại bằng mô hình đã hiệu chỉnh. Không cho policy khám phá ngẫu nhiên trực tiếp trên phần cứng chỉ để khắc phục lỗi mô phỏng.

### Kế hoạch đã thống nhất ở mức phương pháp

1. BM4: chạy robot thật có giám sát, giới hạn an toàn, E-stop hoạt động, dùng vật cản tĩnh không phải người, ghi rosbag/video và các sự kiện. Nếu va chạm hoặc mất an toàn, dừng và kiểm tra nguyên nhân.
2. So sánh với scenario mô phỏng tương ứng; đo trễ, odometry, LiDAR, localization/costmap, braking và tính toán. Hiệu chỉnh mô phỏng và domain randomization theo khoảng đo được.
3. Nếu cần, thực hiện 1–2 đợt hiệu chỉnh/fine-tuning trong simulator rồi triển khai lại. Đây là giới hạn ngân sách dự kiến, không đảm bảo hội tụ. DWA cũng được hiệu chỉnh/tune trên dữ liệu tương ứng để so sánh công bằng.
4. BM5: đóng băng DWA và SAC policy; 6 kịch bản × 5 lần × 2 phương pháp = 60 episode cuối, bố trí/start/goal chung, điều kiện lặp lại, thứ tự chạy cân bằng hoặc ngẫu nhiên hóa có kiểm soát. Không sử dụng các episode final test để học.
5. Tổng hợp SR, CR, SSR, STR, AC; báo cáo số đếm, mẫu số, kết quả theo nhóm và khoảng bất định nếu phù hợp. Mẫu nhỏ: không tuyên bố an toàn tuyệt đối dù không thấy collision. Phân tích từng giả thuyết là được hỗ trợ, chưa đủ bằng chứng hoặc không được hỗ trợ.

H6 chỉ được hỗ trợ nếu hai phương pháp được đánh giá cùng mục tiêu/điều kiện và có dữ liệu chi phí thực tế. Nếu SAC cần ít thời gian con người nhưng tốn nhiều thời gian máy, ghi rõ sự đánh đổi thay vì gộp thành kết luận thắng tuyệt đối.

**Đầu ra:** log thực nghiệm, bảng so sánh sim/real, kết quả BM4/BM5, phân tích sai lệch và giới hạn nghiên cứu. **Trạng thái: chưa thực hiện.**

---

## B. Quy trình thực thi từ đầu đến cuối

```text
[1–2] Problem + Hypotheses (đã chốt)
            ↓
[3] Claude: chuẩn hóa robot + Improved DWA + nghiệm thu
            ↓
[4–5] Claude: đặc tả số + Scenario Generator + feasibility/difficulty
            ↓
[6–7] Evaluator + manifest + pilot + khóa splits
            ↓
[8] Baseline DWA tối ưu + xác nhận ngưỡng SR/ngân sách
            ↓
[9] Thiết kế state/action/reward + SAC pre-training (3 curriculum stages)
            ↓
BM1: General Test → BM2: Zero-shot Classroom/House
            ↓
Retune DWA / Fine-tune SAC độc lập theo map → BM3: Final Simulation
            ↓
[10] BM4: Initial Real → Calibration + fine-tune nếu cần → BM5: Final Real
            ↓
Kết luận H1/H2/H3/H5/H6
```

### Các cổng nghiệm thu trước khi thực hiện tiếp

| Gate | Điều kiện |
|---|---|
| G1 — System ready | Navigation, safety và robot model đã regression; phiên bản chung được đóng băng |
| G2 — Data ready | Generator đúng taxonomy, feasible, không leakage, tỷ lệ bao phủ đủ |
| G3 — Simulation ready | Fast 2D đủ tương đồng với Gazebo qua pilot, chi phí training khả thi |
| G4 — Baseline ready | DWA đã được tune công bằng, mục tiêu SR và ngân sách đã chốt |
| G5 — Policy ready | Validation và S1 nghiệm thu, đáp ứng deadline; không có lỗi an toàn chưa xử lý |
| G6 — Experiment ready | Holdout độc lập, thiết bị an toàn và protocol ghi log hoàn chỉnh |

### Những thông số còn mở — Không tự coi là đã chốt

- Kích thước map và footprint, cấu hình robot cuối, ngưỡng khả thi/độ khó, ngưỡng episode, miền action/horizon và tần số SAC.
- Reward cụ thể, observation dimension, số training steps, số seed training, validation interval, early-stopping patience, thời gian tính toán.
- Số lượng S1 riêng, mục tiêu SSR, ngưỡng hiệu suất tương đương và ngân sách thích nghi để đo H6.
- Mức độ tương đồng 2D/Gazebo, khả năng dựng đủ scenario trên hai map và tài nguyên máy tính.

Các nội dung này phải được xác nhận từ code, đo đạc hoặc pilot. Không dùng final test để chọn các giá trị. Những bổ sung về manifest, kiểm soát leakage, cổng nghiệm thu và cách tổ chức kiểm tra S1 trong tài liệu này là đề xuất thực thi để làm rõ các quyết định đã chốt, không phải kết quả mới.

## C. Nguồn lý thuyết tham khảo đã dùng trong thảo luận

Các nguồn dưới đây là tài liệu nền cho phương pháp; chúng không quy định tỷ lệ 70/30, quy mô 2.140 hay các ngưỡng curriculum của đồ án. Những giá trị đó là lựa chọn thiết kế đã thống nhất, cần pilot kiểm tra.

| Nguồn | Vai trò |
|---|---|
| [Fox et al., The Dynamic Window Approach to Collision Avoidance](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/fox_dynam_window_collision.pdf) | DWA và ràng buộc chuyển động |
| [Perille et al., Benchmarking Metric Ground Navigation (BARN)](https://arxiv.org/abs/2008.13315), [generator](https://github.com/dperille/jackal-map-creation) | Sinh môi trường và đánh giá độ khó |
| [MRPB 1.0: Local Planning Benchmark for Mobile Robots](https://arxiv.org/abs/2011.00491) | Phân loại môi trường/kịch bản local planning |
| [APPL: Adaptive Planner Parameter Learning](https://arxiv.org/abs/2105.07620) | Hướng học tham số planner thay vì thay thế toàn bộ planner |
| [Haarnoja et al., Soft Actor-Critic](https://arxiv.org/abs/1801.01290) | SAC, off-policy actor-critic và maximum entropy |
| [Xu et al., Benchmarking Reinforcement Learning Techniques for Autonomous Navigation](https://arxiv.org/abs/2210.04839) | Đa dạng môi trường training, kiểm tra tổng quát hóa |
| [Kadian et al., Sim2Real Predictivity](https://arxiv.org/abs/1912.06321) | Kiểm chứng tính dự báo của simulator và sai lệch sim-to-real |

Tài liệu này tổng hợp nội dung trao đổi, không thực hiện lại việc đọc toàn bộ các paper hoặc kiểm tra repository hiện tại. Các link dùng để tra cứu thêm; cần kiểm tra nguồn gốc và chi tiết khi viết trích dẫn chính thức cho báo cáo.
