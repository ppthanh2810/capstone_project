// cloud_sim_adapter: thay driver realsense2_camera (D435, camera.launch.py của stage3) trong Gazebo.
//
//   /sim/camera/depth/points (depth_camera Gazebo, frame camera_link: X trước, Y trái, Z lên)
//     ─► /camera/camera/depth/color/points (PointCloud2, frame camera_depth_optical_frame: Z trước, X phải, Y xuống)
//
// Quy ước trục của Gazebo đã kiểm bằng vật đặt ở vị trí biết trước (test/s3_check.py camera): điểm của
// depth_camera Fortress nằm trong frame của link (X là hướng nhìn), KHÔNG phải optical. Chỉ đổi frame_id
// thì dữ liệu sai trục -> phải xoay từng điểm:
//   x_opt = -y_link,  y_opt = -z_link,  z_opt = x_link      (= nghịch đảo rpy (-pi/2, 0, -pi/2) của URDF)
// Đầu ra giống realsense2_camera với pointcloud.stream_filter 0 (không texture): chỉ trường x, y, z float32,
// unorganized (height 1), bỏ điểm không hữu hạn (driver thật không phát điểm độ sâu 0).
// Stamp giữ của Gazebo (sim time lúc chụp).

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class CloudSimAdapter : public rclcpp::Node
{
public:
  CloudSimAdapter()
  : Node("cloud_sim_adapter")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    const auto input_topic = declare_parameter<std::string>("input_topic", "/sim/camera/depth/points");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/camera/camera/depth/color/points");

    // realsense2_camera 4.x: point cloud publisher QoS mặc định = SensorDataQoS-like (best effort ở một số bản);
    // FusionLayer subscribe bằng SensorDataQoS nên reliable/best effort đều nhận được. Dùng reliable keep last 5.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(rclcpp::KeepLast(5)));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&CloudSimAdapter::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "CLOUD SIM ADAPTER | %s (camera_link X trước) -> %s (frame %s, Z trước)",
      input_topic.c_str(), output_topic.c_str(), frame_id_.c_str());
  }

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr in)
  {
    const size_t n = static_cast<size_t>(in->width) * in->height;
    if (n == 0) return;

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = in->header.stamp;
    out.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(n);

    sensor_msgs::PointCloud2ConstIterator<float> ix(*in, "x"), iy(*in, "y"), iz(*in, "z");
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    size_t kept = 0;
    for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      *ox = -y;
      *oy = -z;
      *oz = x;
      ++ox; ++oy; ++oz; ++kept;
    }
    mod.resize(kept);
    out.height = 1;
    out.width = static_cast<uint32_t>(kept);
    out.is_dense = true;
    pub_->publish(std::move(out));
  }

  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
