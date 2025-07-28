/** Robotic-StereoVision License **/

#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "lebai_interfaces/srv/set_gripper.hpp"
#include "std_srvs/srv/empty.hpp"
#include "zed_interfaces/msg/trk.hpp"
#include "zed_interfaces/srv/set_pos.hpp"

namespace RoboticStereoVision {
namespace grabbing {

class GrabbingNode : public rclcpp::Node {
 public:
  explicit GrabbingNode(const std::string &name);
  ~GrabbingNode() override;

 private:
  void KeyCallback(const std_msgs::msg::String::SharedPtr msg);
  void RoboticInit();
  void InitMoveGroup();
  void MoveToInitial();
  void MoveToTarget();
  void TargetGrabbing();
  void StopMotion();

  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::Client<zed_interfaces::srv::SetPos>::SharedPtr target_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr robotic_enable_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr robotic_disable_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr robotic_emergency_stop_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr robotic_turn_off_;

  rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedPtr gripper_force_;
  rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedPtr
      gripper_position_;
};

}  // namespace grabbing
}  // namespace RoboticStereoVision
