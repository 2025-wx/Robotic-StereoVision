/** Robotic-StereoVision License **/

#include <rclcpp/rclcpp.hpp>

#include "target.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoboticStereoVision::target_tf::TargetNode>(
      "target_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
