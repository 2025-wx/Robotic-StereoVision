/** Robotic-StereoVision License **/

#include <rclcpp/rclcpp.hpp>

#include "grabbing.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<RoboticStereoVision::grabbing::GrabbingNode>(
      "grabbing_node", node_options);
  node->InitMoveGroup();
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
