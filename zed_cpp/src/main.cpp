/** Robotic-StereoVision License **/

#include <rclcpp/rclcpp.hpp>

#include "zed.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoboticStereoVision::vision::ZedNode>("zed_pub");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}