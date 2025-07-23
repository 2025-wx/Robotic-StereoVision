/** Robotic-StereoVision License **/

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "zed_interfaces/msg/trk.hpp"
#include "zed_interfaces/srv/set_pos.hpp"

namespace RoboticStereoVision {
namespace target_tf {

class TargetNode : public rclcpp::Node {
 public:
  explicit TargetNode(const std::string &name);
  ~TargetNode() override = default;

 private:
  void DetSubCallback(const zed_interfaces::msg::Trk::SharedPtr msg);
  void TargetService(
      const std::shared_ptr<zed_interfaces::srv::SetPos::Request> request,
      const std::shared_ptr<zed_interfaces::srv::SetPos::Response> response);

  std::deque<zed_interfaces::msg::Obj> objects_class_1_;
  std::mutex mutex_;

  rclcpp::Subscription<zed_interfaces::msg::Trk>::SharedPtr det_sub_;
  rclcpp::Service<zed_interfaces::srv::SetPos>::SharedPtr target_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace target_tf
}  // namespace RoboticStereoVision
