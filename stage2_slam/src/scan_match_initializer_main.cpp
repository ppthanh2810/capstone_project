#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stage2_slam/scan_match_initializer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stage2_slam::ScanMatchInitializerNode>());
  rclcpp::shutdown();
  return 0;
}
