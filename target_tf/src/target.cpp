/** Robotic-StereoVision License **/

#include "target.hpp"

#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace RoboticStereoVision {
namespace target_tf {

namespace {
constexpr const char *kTargetServer = "target_server_name";
constexpr const char *kTargetDefaultServer = "target_server";
constexpr const char *kZedSubscriber = "zed_subscriber_name";
constexpr const char *kZedDefaultSubscriber = "/zed/detections";
constexpr int kTargetLabelId = 1;
constexpr size_t kMaxTargetHistory = 10;
}  // namespace

TargetNode::TargetNode(const std::string &name)
    : Node(name), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
  std::string sub_topic_name;
  std::string target_server_name;

  // 参数声明与获取
  this->declare_parameter(kZedSubscriber, std::string(kZedDefaultSubscriber));
  this->get_parameter_or(kZedSubscriber, sub_topic_name,
                         std::string(kZedDefaultSubscriber));
  this->declare_parameter(kTargetServer, std::string(kTargetDefaultServer));
  this->get_parameter_or(kTargetServer, target_server_name,
                         std::string(kTargetDefaultServer));

  // 订阅器
  det_sub_ = this->create_subscription<zed_interfaces::msg::Trk>(
      sub_topic_name, 10,
      std::bind(&TargetNode::DetSubCallback, this, std::placeholders::_1));

  // 服务端
  target_server_ = this->create_service<zed_interfaces::srv::SetPos>(
      target_server_name,
      std::bind(&TargetNode::TargetService, this, std::placeholders::_1,
                std::placeholders::_2));
}

void TargetNode::DetSubCallback(const zed_interfaces::msg::Trk::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->objects.empty()) {
    return;
  }

  std::optional<zed_interfaces::msg::Obj> min_obj;
  float min_z = std::numeric_limits<float>::max();

  for (const auto &obj : msg->objects) {
    if (obj.label_id == kTargetLabelId) {
      float z = obj.position[2];
      if (z < min_z) {
        min_z = z;
        min_obj = obj;
      }
    }
  }

  if (min_obj.has_value()) {
    if (objects_class_1_.size() >= kMaxTargetHistory) {
      objects_class_1_.pop_front();
    }
    objects_class_1_.push_back(min_obj.value());
  }
}

void TargetNode::TargetService(
    const std::shared_ptr<zed_interfaces::srv::SetPos::Request> request,
    const std::shared_ptr<zed_interfaces::srv::SetPos::Response> response) {

  (void)request; 

  std::lock_guard<std::mutex> lock(mutex_);

  if (objects_class_1_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No target objects available.");
    response->success = false;
    return;
  }

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  for (const auto &obj : objects_class_1_) {
    sum_x += obj.position[0];
    sum_y += obj.position[1];
    sum_z += obj.position[2];
  }

  size_t count = objects_class_1_.size();
  float avg_x = sum_x / count;
  float avg_y = sum_y / count;
  float avg_z = sum_z / count;

  geometry_msgs::msg::PointStamped target_in_camera;
  target_in_camera.header.stamp = this->now();
  target_in_camera.header.frame_id = "zed_camera_link";
  target_in_camera.point.x = avg_x;
  target_in_camera.point.y = avg_y;
  target_in_camera.point.z = avg_z;

  try {
    geometry_msgs::msg::PointStamped target_in_base;
    tf_buffer_.transform(target_in_camera, target_in_base, "base_link",
                         tf2::durationFromSec(0.1));

    response->position = {static_cast<float>(target_in_base.point.x),
                          static_cast<float>(target_in_base.point.y),
                          static_cast<float>(target_in_base.point.z)};
    response->success = true;

    RCLCPP_INFO(this->get_logger(),
                "Target position [%.3f, %.3f, %.3f] has been transformed to "
                "base_link frame.",
                response->position[0], response->position[1],
                response->position[2]);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF conversion failed: %s", ex.what());
    response->position = {0.0f, 0.0f, 0.0f};
    response->success = false;
  }
}

}  // namespace target_tf
}  // namespace RoboticStereoVision
