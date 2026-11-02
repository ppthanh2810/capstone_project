// zlac_sim: thay zlac8015d_move (Modbus) trong Gazebo.
// Tầng ROS chép từ stage0_ps4_control/src/zlac8015d_move.cpp (động học, lround RPM, đảo dấu bánh phải,
// watchdog 100 ms / 0.5 s, feedback 20 Hz). Khác: timer theo clock của node (sim time) và
// Modbus được thay bằng ZlacDriverModel + joint bánh trong Gazebo.
//
//   /cmd_vel_out ─► động học ─► ZlacDriverModel (trễ + ramp) ─► /sim/.../cmd_vel (Float64, rad/s) ─► JointController
//   /wheel_feedback ◄─ RPM 0.1 + tick encoder 4096 ◄─ /sim/wheel_joint_states (vị trí/vận tốc joint thật)

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "stage4_gazebo/zlac_driver_model.hpp"

using namespace std::chrono_literals;
using stage4_gazebo::ZlacDriverModel;
using stage4_gazebo::ZlacModelParams;

class ZlacSim : public rclcpp::Node
{
public:
  ZlacSim()
  : Node("zlac_sim"),
    model_(loadModelParams())
  {
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0535);
    wheel_distance_ = declare_parameter<double>("wheel_distance", 0.34);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    left_joint_ = declare_parameter<std::string>("left_joint", "left_wheel_joint");
    right_joint_ = declare_parameter<std::string>("right_joint", "right_wheel_joint");
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    const double control_rate = declare_parameter<double>("control_rate", 200.0);
    const double feedback_rate = declare_parameter<double>("feedback_rate", 20.0);
    const auto left_cmd_topic =
      declare_parameter<std::string>("left_cmd_topic", "/sim/left_wheel/cmd_vel");
    const auto right_cmd_topic =
      declare_parameter<std::string>("right_cmd_topic", "/sim/right_wheel/cmd_vel");
    const auto joint_state_topic =
      declare_parameter<std::string>("joint_state_topic", "/sim/wheel_joint_states");

    if (!(wheel_radius_ > 0.0) || !(wheel_distance_ > 0.0) || !(command_timeout_ > 0.0) ||
      !(control_rate > 0.0) || !(feedback_rate > 0.0))
    {
      throw std::invalid_argument("wheel_radius, wheel_distance, command_timeout and rates must be > 0");
    }

    motion_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_out", 1, std::bind(&ZlacSim::motionCallback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&ZlacSim::jointCallback, this, std::placeholders::_1));

    feedback_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_feedback", 10);
    left_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(left_cmd_topic, 10);
    right_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(right_cmd_topic, 10);
    if (publish_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }
    if (publish_debug_) {
      debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/sim/zlac/debug", 10);
    }

    // Timer theo clock của node: với use_sim_time:=true chạy theo /clock của Gazebo
    auto period = [](double rate) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
      };
    control_timer_ = rclcpp::create_timer(this, get_clock(), period(control_rate), [this] {controlStep();});
    feedback_timer_ = rclcpp::create_timer(this, get_clock(), period(feedback_rate), [this] {readFeedback();});
    watchdog_timer_ = rclcpp::create_timer(this, get_clock(), 100ms, [this] {checkWatchdog();});

    last_command_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "zlac_sim started | accel %.0f ms, decel %.0f ms, ramp %s, delay %.3f s, tau %.3f s",
      params_.accel_time_s * 1000.0, params_.decel_time_s * 1000.0,
      params_.ramp_mode == stage4_gazebo::RampMode::TimeToTarget ? "time_to_target" : "rate",
      params_.command_delay_s, params_.speed_loop_tau_s);
  }

  ~ZlacSim() override
  {
    // giống destructor driver thật: đặt 0 rồi disable -> bánh dừng ngay
    try {
      publishWheelCommand(0.0, 0.0);
    } catch (...) {
    }
  }

private:
  ZlacModelParams loadModelParams()
  {
    // cùng tên và miền hợp lệ như zlac8015d_move
    const int accel_time_ms = static_cast<int>(declare_parameter<int>("accel_time_ms", 500));
    const int decel_time_ms = static_cast<int>(declare_parameter<int>("decel_time_ms", 500));
    if (accel_time_ms < 50 || accel_time_ms > 2000 || decel_time_ms < 50 || decel_time_ms > 2000) {
      throw std::invalid_argument("accel_time_ms and decel_time_ms must be in [50, 2000] ms");
    }
    params_.accel_time_s = accel_time_ms / 1000.0;
    params_.decel_time_s = decel_time_ms / 1000.0;

    const auto ramp_mode = declare_parameter<std::string>("ramp_mode", "time_to_target");
    if (!stage4_gazebo::parseRampMode(ramp_mode, params_.ramp_mode)) {
      throw std::invalid_argument("ramp_mode must be 'time_to_target' or 'rate'");
    }
    params_.ramp_reference_rpm = declare_parameter<double>("ramp_reference_rpm", 3000.0);
    params_.command_delay_s = declare_parameter<double>("command_delay", 0.0);
    params_.speed_loop_tau_s = declare_parameter<double>("speed_loop_tau", 0.0);

    const std::string error = stage4_gazebo::validateParams(params_);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    return params_;
  }

  // ============================================================
  // MOTION CONTROL (giống zlac8015d_move::motionCallback)
  // ============================================================

  void motionCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double x = msg->linear.x;
    double z = msg->angular.z;

    double left = (x - wheel_distance_ / 2.0 * z) / wheel_radius_;
    double right = (x + wheel_distance_ / 2.0 * z) / wheel_radius_;

    double left_rpm = left * rpm_factor_;
    double right_rpm = -right * rpm_factor_;

    if (!std::isfinite(left_rpm) || !std::isfinite(right_rpm)) {
      RCLCPP_WARN(get_logger(), "Set RPM failed: non-finite command");
      return;
    }

    model_.setRpm(
      now().seconds(),
      static_cast<int>(std::lround(left_rpm)),
      static_cast<int>(std::lround(right_rpm)));

    last_command_time_ = now();
    command_received_ = true;
    watchdog_stopped_ = false;
  }

  // ============================================================
  // WATCHDOG (giống zlac8015d_move::checkWatchdog)
  // ============================================================

  void checkWatchdog()
  {
    if (!command_received_) return;

    double elapsed = (now() - last_command_time_).seconds();

    if (elapsed > command_timeout_ && !watchdog_stopped_) {
      model_.setRpm(now().seconds(), 0, 0);
      watchdog_stopped_ = true;
      RCLCPP_WARN(get_logger(), "Command timeout. Motors stopped.");
    }
  }

  // ============================================================
  // FIRMWARE + VẬT LÝ
  // ============================================================

  void controlStep()
  {
    const auto [left_raw, right_raw] = model_.update(now().seconds());
    publishWheelCommand(left_raw, right_raw);

    if (publish_debug_) {
      // [t, target_l, target_r, ramp_out_l, ramp_out_r, joint_rpm_l, joint_rpm_r] — đã chuẩn hóa dấu
      // (hai bánh tiến = dương), joint_rpm chưa lượng tử
      std_msgs::msg::Float64MultiArray debug;
      debug.data = {
        now().seconds(),
        model_.left().target(), -model_.right().target(),
        left_raw, -right_raw,
        left_vel_ * rpm_factor_, -right_vel_ * rpm_factor_};
      debug_pub_->publish(debug);
    }
  }

  void publishWheelCommand(double left_raw_rpm, double right_raw_rpm)
  {
    // raw rpm > 0 = chiều dương của joint (URDF sim chọn trục cho khớp quy ước motor thật)
    std_msgs::msg::Float64 left;
    std_msgs::msg::Float64 right;
    left.data = stage4_gazebo::rpmToRadPerSec(left_raw_rpm);
    right.data = stage4_gazebo::rpmToRadPerSec(right_raw_rpm);
    left_cmd_pub_->publish(left);
    right_cmd_pub_->publish(right);
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool got_left = false;
    bool got_right = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size() || i >= msg->velocity.size()) {
        break;
      }
      if (msg->name[i] == left_joint_) {
        left_pos_ = msg->position[i];
        left_vel_ = msg->velocity[i];
        got_left = true;
      } else if (msg->name[i] == right_joint_) {
        right_pos_ = msg->position[i];
        right_vel_ = msg->velocity[i];
        got_right = true;
      }
    }
    if (got_left && got_right) {
      have_joints_ = true;
    }
  }

  // ============================================================
  // MOTOR FEEDBACK (giống zlac8015d_move::readFeedback)
  // ============================================================

  void readFeedback()
  {
    if (!have_joints_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Read feedback failed: no wheel joint states from Gazebo yet");
      return;
    }

    // getRpm(): int16 0.1 rpm; getWheelsTravelled(): tick int32 / 4096 * 0.336
    double left_rpm_raw = stage4_gazebo::feedbackRpm(left_vel_);
    double right_rpm_raw = stage4_gazebo::feedbackRpm(right_vel_);
    double left_travelled_raw = stage4_gazebo::ticksToTravelled(stage4_gazebo::encoderTicks(left_pos_));
    double right_travelled_raw = stage4_gazebo::ticksToTravelled(stage4_gazebo::encoderTicks(right_pos_));

    double left_rpm = left_rpm_raw;
    double right_rpm = -right_rpm_raw;
    double left_travelled = left_travelled_raw;
    double right_travelled = -right_travelled_raw;

    std_msgs::msg::Float64MultiArray feedback;
    feedback.data = {left_rpm, right_rpm, left_travelled, right_travelled};
    feedback_pub_->publish(feedback);

    if (publish_joint_states_) {
      // driver thật không có /joint_states; ở sim chỉ để robot_state_publisher vẽ bánh quay
      // (bridge thẳng joint_state 1 kHz của Gazebo sẽ làm /tf phát 1 kHz)
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      js.name = {left_joint_, right_joint_};
      js.position = {left_pos_, right_pos_};
      js.velocity = {left_vel_, right_vel_};
      joint_state_pub_->publish(js);
    }
  }

  // ============================================================
  // PARAMETERS
  // ============================================================

  static constexpr double pi_ = 3.14159265358979323846;
  static constexpr double rpm_factor_ = 30.0 / pi_;

  ZlacModelParams params_;
  ZlacDriverModel model_;

  double wheel_radius_ = 0.0535;
  double wheel_distance_ = 0.34;
  double command_timeout_ = 0.5;
  std::string left_joint_;
  std::string right_joint_;
  bool publish_joint_states_ = true;
  bool publish_debug_ = true;

  bool command_received_ = false;
  bool watchdog_stopped_ = false;
  rclcpp::Time last_command_time_;

  bool have_joints_ = false;
  double left_pos_ = 0.0;
  double right_pos_ = 0.0;
  double left_vel_ = 0.0;
  double right_vel_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZlacSim>());
  rclcpp::shutdown();
  return 0;
}
