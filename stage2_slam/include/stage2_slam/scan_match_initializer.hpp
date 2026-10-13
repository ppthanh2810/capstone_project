// ScanMatchInitializer: bounded correlative scan matching (CSM) quanh pose người dùng chọn,
// dùng để hiệu chỉnh /initialpose trước khi gửi cho AMCL.
//
// Phần lõi KHÔNG phụ thuộc ROS. Node ROS 2 (scan_match_initializer_node) lo
// subscribe /map, /scan_filtered, TF và publish kết quả.
//
// Quy ước:
//   - Tất cả đơn vị SI (m, rad). Pose 2D trong frame map (REP-105).
//   - Scan đã được chuẩn hóa bởi scan_rotate (+pi) trước đó: angle_min/angle_increment dùng
//     NGUYÊN TRẠNG, không bù thêm góc. Vị trí/hướng LiDAR so với base lấy từ TF (sensor_in_base).
//   - Ô OccupancyGrid: -1 unknown, 0..100 xác suất chiếm chỗ; row-major, ô (0,0) ở góc origin.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace stage2_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

double normalizeAngle(double a);
// a ⊕ b (b biểu diễn trong frame của a)
Pose2D compose(const Pose2D & a, const Pose2D & b);
// a^-1 ⊕ b
Pose2D between(const Pose2D & a, const Pose2D & b);

// ---------------------------------------------------------------------------
// Map
// ---------------------------------------------------------------------------
struct GridMap
{
  int width{0};
  int height{0};
  double resolution{0.0};
  Pose2D origin;               // pose của góc ô (0,0) trong frame map (có thể có yaw)
  std::vector<int8_t> data;    // width*height, row-major
};

struct ScoreFieldConfig
{
  int occupied_threshold{65};   // ô >= ngưỡng coi là vật cản
  double sigma{0.05};           // m, độ rộng Gaussian của likelihood field
  double max_distance{0.5};     // m, xa hơn -> score 0
  double unknown_score{0.0};    // score cho điểm rơi vào ô unknown
};

// Likelihood field: score(cell) = exp(-d^2 / (2 sigma^2)), d = khoảng cách từ tâm ô tới tâm ô chiếm
// chỗ gần nhất (exact Euclidean distance transform). Ô unknown -> unknown_score, ngoài bản đồ -> 0.
class ScoreField
{
public:
  // Trả về nullopt (kèm lý do) nếu map không hợp lệ.
  static std::optional<ScoreField> build(
    const GridMap & map, const ScoreFieldConfig & config, std::string * error = nullptr);

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}
  const Pose2D & origin() const {return origin_;}
  std::size_t occupiedCells() const {return occupied_cells_;}

  // Toạ độ ô liên tục (đơn vị ô) của một điểm trong frame map.
  void worldToGrid(double x, double y, double & gx, double & gy) const;
  // Score theo ô gần nhất (dùng trong tìm kiếm).
  double scoreAtGrid(double gx, double gy) const;
  // Score nội suy song tuyến (dùng khi refine).
  double scoreAtGridBilinear(double gx, double gy) const;
  double scoreAtWorld(double x, double y) const;

  // Hệ số đổi một độ dời (dx, dy) trong frame map sang độ dời ô.
  double cosOrigin() const {return cos_o_;}
  double sinOrigin() const {return sin_o_;}

private:
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Pose2D origin_;
  double cos_o_{1.0};
  double sin_o_{0.0};
  std::size_t occupied_cells_{0};
  std::vector<float> score_;
  float cellScore(int cx, int cy) const;
};

// ---------------------------------------------------------------------------
// Scan
// ---------------------------------------------------------------------------
struct ScanData
{
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<float> ranges;
};

struct PreprocessConfig
{
  double min_range{0.10};      // m (dùng max với scan.range_min)
  double max_range{12.0};      // m (dùng min với scan.range_max)
  std::size_t max_points{400}; // giảm mẫu đều nếu nhiều hơn
};

struct PreprocessResult
{
  std::vector<Point2D> points;  // trong frame base (base_footprint)
  std::size_t total_beams{0};
  std::size_t valid_beams{0};   // hữu hạn và trong khoảng đo
  double max_point_range{0.0};  // m, khoảng cách xa nhất của điểm hợp lệ tới LiDAR
};

// Đổi LaserScan -> điểm 2D trong frame base. Không cộng thêm offset góc nào ngoài sensor_in_base.
PreprocessResult scanToPoints(
  const ScanData & scan, const Pose2D & sensor_in_base, const PreprocessConfig & config);

// ---------------------------------------------------------------------------
// Matcher
// ---------------------------------------------------------------------------
enum class MatchStatus
{
  ACCEPTED,
  REJECTED,
  AMBIGUOUS,
  NO_SCAN,
  NO_MAP,
  TF_ERROR,
  TIMEOUT,
  AMCL_INACTIVE,
  INVALID_INPUT,
};

const char * toString(MatchStatus status);

struct SearchConfig
{
  double linear_window{0.5};       // m, ± theo X và Y quanh guess
  double angular_window{0.349066}; // rad, ± theo yaw (20 deg)
  double linear_step{0.025};       // m
  // rad; <= 0 -> tự tính theo tầm xa của scan: acos(1 - r^2 / (2 d^2)) (như Cartographer),
  // kẹp trong [min_auto_angular_step, max_auto_angular_step]
  double angular_step{0.0};
  double min_auto_angular_step{0.004};
  double max_auto_angular_step{0.035};
  std::size_t max_candidates{4000000};
  double time_budget{3.0};         // s, quá thời gian -> TIMEOUT
};

// Tinh chỉnh pattern search trên score nội suy song tuyến. TẮT mặc định: benchmark tổng hợp cho
// thấy không cải thiện (làm tăng sai số vị trí trung bình) so với trọng tâm vùng đỉnh trên lưới, vì
// điểm chạm thật nằm bất kỳ đâu trong ô chiếm chỗ còn nội suy kéo điểm về tâm ô.
struct RefineConfig
{
  bool enabled{false};
  double min_linear_step{0.002};   // m
  double min_angular_step{0.001};  // rad
  int max_iterations{200};
};

struct ValidationConfig
{
  std::size_t min_valid_points{60};
  double min_score{0.65};             // score trung bình [0,1] tại pose tốt nhất (chọn trên tập tuning)
  double ambiguity_ratio{0.90};       // cực đại cạnh tranh / tốt nhất vượt ngưỡng -> AMBIGUOUS
  double peak_separation_linear{0.25};   // m, cực đại cách xa hơn mới tính là "độc lập"
  double peak_separation_angular{0.17};  // rad (~10 deg)
  // Cực đại cạnh tranh chỉ tính là đỉnh độc lập khi score nhỏ nhất trên đoạn thẳng (trong lưới ứng
  // viên) nối nó với đỉnh tốt nhất < valley_ratio * score của nó (có "thung lũng" ngăn cách).
  // Sườn/đỉnh dải của cùng một đỉnh (hành lang, tường dài) không phải nhập nhằng: độ bất định đó đã
  // nằm trong covariance. Ứng viên trên biên cửa sổ bị bỏ qua (không kiểm chứng được là cực đại).
  double valley_ratio{0.90};
  bool reject_at_window_boundary{true};
  double inlier_score{0.5};           // điểm có score >= ngưỡng này tính là inlier (chẩn đoán)
};

struct CovarianceConfig
{
  double min_std_linear{0.05};   // m
  double max_std_linear{0.50};   // m
  double min_std_angular{0.035}; // rad (~2 deg)
  double max_std_angular{0.35};  // rad (~20 deg)
  double score_fraction{0.90};   // dùng ứng viên có score >= fraction * best để ước lượng độ trải
};

struct MatcherConfig
{
  ScoreFieldConfig score_field;
  PreprocessConfig preprocess;
  SearchConfig search;
  RefineConfig refine;
  ValidationConfig validation;
  CovarianceConfig covariance;
};

struct MatchResult
{
  MatchStatus status{MatchStatus::INVALID_INPUT};
  std::string reason;
  Pose2D guess;
  Pose2D pose;                  // pose tốt nhất (sau refine)
  Pose2D coarse_pose;           // pose tốt nhất trên lưới
  double score{0.0};            // score trung bình tại pose (nearest-cell)
  double guess_score{0.0};      // score tại guess
  double second_score{0.0};     // cực đại độc lập lớn thứ hai (0 nếu không có)
  Pose2D second_pose;
  double ambiguity{0.0};        // second_score / score
  double inlier_fraction{0.0};
  std::size_t valid_points{0};
  std::size_t candidates{0};
  double angular_step{0.0};
  double elapsed{0.0};          // s
  bool at_window_boundary{false};
  // Hiệp phương sai 3x3 (x, y, yaw), row-major. Heuristic từ độ trải của các ứng viên gần đỉnh,
  // kẹp trong [min_std, max_std]. KHÔNG phải hiệp phương sai đã hiệu chuẩn thống kê.
  std::array<double, 9> covariance{};
};

// Tìm pose khớp nhất trong cửa sổ quanh guess. points ở frame base.
MatchResult matchScan(
  const ScoreField & field, const std::vector<Point2D> & points, std::size_t total_beams,
  const Pose2D & guess, const MatcherConfig & config);

// Score trung bình (nearest-cell) của tập điểm tại một pose.
double scorePose(const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & pose);
double scorePoseBilinear(
  const ScoreField & field, const std::vector<Point2D> & points, const Pose2D & pose);

// ---------------------------------------------------------------------------
// Session: gom nhiều scan cho một yêu cầu khởi tạo, kiểm tra TF/tuổi dữ liệu/chuyển động/nhất quán.
// ---------------------------------------------------------------------------
struct SessionConfig
{
  std::size_t num_scans{3};         // số scan phải khớp và nhất quán
  double scan_max_age{0.5};         // s, now - stamp
  double timeout{5.0};              // s kể từ lúc nhận guess
  double max_motion_linear{0.03};   // m, robot phải gần như đứng yên giữa các scan (theo odom)
  double max_motion_angular{0.03};  // rad
  double max_spread_linear{0.05};   // m, độ lệch giữa các pose khớp
  double max_spread_angular{0.035}; // rad
  std::size_t max_tf_failures{3};   // lỗi TF liên tiếp -> TF_ERROR (lỗi lẻ tẻ: bỏ qua scan đó)
};

struct ScanSample
{
  double stamp{0.0};                       // s
  ScanData scan;
  std::optional<Pose2D> sensor_in_base;    // TF base_frame <- laser frame tại stamp
  std::optional<Pose2D> odom_pose;         // TF odom <- base_frame tại stamp (kiểm tra đứng yên)
  std::string tf_error;
};

struct SessionResult
{
  MatchStatus status{MatchStatus::INVALID_INPUT};
  std::string reason;
  MatchResult final_match;          // match của scan cuối (pose dùng để publish)
  double stamp{0.0};                // stamp của scan cuối
  std::vector<MatchResult> matches;
  std::size_t stale_scans{0};
  bool publish{false};              // true chỉ khi ACCEPTED
};

class InitializationSession
{
public:
  InitializationSession(
    const Pose2D & guess, double start_time, const SessionConfig & session,
    const MatcherConfig & matcher);

  // Thêm một scan. Trả về kết quả khi phiên kết thúc (đủ scan hoặc lỗi), nullopt khi còn chờ.
  std::optional<SessionResult> addScan(
    const ScanSample & sample, double now, const ScoreField * field);
  // Gọi định kỳ: TIMEOUT / NO_SCAN khi quá hạn.
  std::optional<SessionResult> checkTimeout(double now) const;

  const Pose2D & guess() const {return guess_;}
  std::size_t collected() const {return matches_.size();}

private:
  SessionResult finish(MatchStatus status, const std::string & reason) const;

  Pose2D guess_;
  double start_time_;
  SessionConfig session_;
  MatcherConfig matcher_;
  std::vector<MatchResult> matches_;
  std::vector<Pose2D> odom_poses_;
  double last_stamp_{0.0};
  std::size_t stale_scans_{0};
  std::size_t tf_failures_{0};
  std::string last_tf_error_;
};

}  // namespace stage2_slam
