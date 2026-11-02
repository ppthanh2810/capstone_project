#pragma once

// Mô hình "hộp đen" của driver ZLAC8015D (tầng firmware), không phụ thuộc ROS.
// Tầng ROS (động học, làm tròn RPM, đảo dấu bánh phải, watchdog) nằm ở zlac_sim_node.cpp,
// chép theo stage0_ps4_control/src/zlac8015d_move.cpp.
//
// Quy ước "raw" giống thanh ghi của driver thật: lệnh/feedback theo chiều dương của từng motor.
// Trên xe, motor phải lắp ngược nên raw phải > 0 = bánh phải quay lùi; trong URDF sim trục joint
// phải được chọn cho khớp quy ước này (xem urdf/robot_sim.urdf.xacro).

#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace stage4_gazebo
{

// Cách hiểu thanh ghi thời gian tăng/giảm tốc 0x2080..0x2083 (TBD – cần đo trên xe):
//  TimeToTarget: mỗi lần đổi lệnh, motor đi tới tốc độ đích trong đúng accel/decel_time.
//                Khớp nhận xét log robot (500 ms -> trễ ~0.7-0.8 s, 200 ms -> ~0.2 s).
//  Rate:         độ dốc cố định ramp_reference_rpm / accel_time (kiểu "thời gian từ 0 tới N rpm").
enum class RampMode { TimeToTarget, Rate };

bool parseRampMode(const std::string & text, RampMode & mode);

struct ZlacModelParams
{
  double accel_time_s = 0.5;        // driver thật: accel_time_ms mặc định 500
  double decel_time_s = 0.5;
  RampMode ramp_mode = RampMode::TimeToTarget;
  double ramp_reference_rpm = 3000.0;  // chỉ dùng ở RampMode::Rate
  double command_delay_s = 0.0;     // trễ từ setRpm tới khi firmware nhận lệnh
  double speed_loop_tau_s = 0.0;    // vòng tốc độ bên trong xấp xỉ bậc 1; 0 = bám tức thời theo ramp
  int max_rpm = 3000;               // Controller::setRpm kẹp ±3000
};

// Trả về chuỗi lỗi rỗng nếu hợp lệ.
std::string validateParams(const ZlacModelParams & p);

// Ramp + vòng tốc độ của một motor.
class ZlacMotorModel
{
public:
  explicit ZlacMotorModel(const ZlacModelParams & params);

  void setTarget(double rpm);
  // Tiến thêm dt giây, trả về tốc độ đặt (raw rpm) gửi xuống vật lý.
  double update(double dt);
  void reset();

  double target() const {return target_;}
  double ramped() const {return ramped_;}
  double output() const {return output_;}

private:
  ZlacModelParams p_;
  double target_ = 0.0;
  double ramped_ = 0.0;   // sau khâu ramp
  double output_ = 0.0;   // sau vòng tốc độ
  double rate_ = 0.0;     // rpm/s của đoạn ramp hiện tại
};

// Firmware hai kênh: hàng đợi lệnh (trễ) + ramp từng bánh.
class ZlacDriverModel
{
public:
  explicit ZlacDriverModel(const ZlacModelParams & params);

  // Giống Controller::setRpm: lệnh nguyên, kẹp ±max_rpm, có hiệu lực sau command_delay.
  void setRpm(double now_s, int left_raw, int right_raw);
  // Tiến tới thời điểm now_s (sim time). Trả về {left_raw_rpm, right_raw_rpm} đặt cho vật lý.
  std::pair<double, double> update(double now_s);
  void reset();

  const ZlacMotorModel & left() const {return left_;}
  const ZlacMotorModel & right() const {return right_;}

private:
  struct PendingCommand
  {
    double apply_time;
    int left;
    int right;
  };

  ZlacModelParams p_;
  ZlacMotorModel left_;
  ZlacMotorModel right_;
  std::deque<PendingCommand> queue_;
  double last_time_ = 0.0;
  bool has_time_ = false;
};

// ---- Feedback giống thanh ghi của driver thật ----
constexpr double kEncoderCpr = 4096.0;         // zlac8015d.hpp CPR
constexpr double kTravelInOneRev = 0.336;      // zlac8015d.hpp TRAVEL_IN_ONE_REV (không phải 2*pi*r)

// Tốc độ joint (rad/s) -> RPM như getRpm(): thanh ghi int16 đơn vị 0.1 rpm.
double feedbackRpm(double joint_velocity_rad_s);
// Góc joint (rad, tính từ lúc spawn) -> tick encoder int32.
int32_t encoderTicks(double joint_position_rad);
// Như getWheelsTravelled(): ticks / CPR * TRAVEL_IN_ONE_REV.
double ticksToTravelled(int32_t ticks);
// Lệnh raw rpm -> tốc độ joint rad/s.
double rpmToRadPerSec(double rpm);

}  // namespace stage4_gazebo
