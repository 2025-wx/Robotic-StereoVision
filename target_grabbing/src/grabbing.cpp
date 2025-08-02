/** Robotic-StereoVision License **/

#include "grabbing.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace RoboticStereoVision {
namespace grabbing {

namespace {
constexpr const char *kTargetClient = "target_client_name";
constexpr const char *kTargetDefaultClient = "target_position";
constexpr const char *kMoveGroup = "move_group_name";
constexpr const char *kDefaultMoveGroup = "robotic";
// constexpr const char *kMoveGroup = "move_group_name";
// constexpr const char *kDefaultMoveGroup = "robotic";
}  // namespace

GrabbingNode::GrabbingNode(const std::string &name,
                           const rclcpp::NodeOptions &options)
    : Node(name, options) {
  // std::string move_group_name;
  std::string target_client_name;
  //   std::string move_group_name;
  this->declare_parameter(kMoveGroup, std::string(kDefaultMoveGroup));
  this->get_parameter_or(kMoveGroup, move_group_name_,
                         std::string(kDefaultMoveGroup));
  this->declare_parameter(kTargetClient, std::string(kTargetDefaultClient));
  this->get_parameter_or(kTargetClient, target_client_name,
                         std::string(kTargetDefaultClient));

  RoboticInit();

  target_client_ =
      this->create_client<zed_interfaces::srv::SetPos>(target_client_name);

  // move_group_ =
  //     std::make_shared<moveit::planning_interface::MoveGroupInterface>(
  //         shared_from_this(), move_group_name);

  // move_group_->setPoseReferenceFrame("base_link");

  key_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/key_input", 10,
      std::bind(&GrabbingNode::KeyCallback, this, std::placeholders::_1));
}

GrabbingNode::~GrabbingNode() {
  StopMotion();
}

void GrabbingNode::InitMoveGroup() {
  try {
    RCLCPP_INFO(this->get_logger(),
                "Initializing MoveGroupInterface with group '%s'...",
                move_group_name_.c_str());
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), move_group_name_);
    move_group_->setPoseReferenceFrame("base_link");
    RCLCPP_INFO(this->get_logger(),
                "MoveGroupInterface initialized successfully.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize MoveGroupInterface: %s", e.what());
  }
}

void GrabbingNode::RoboticInit() {
  robotic_enable_ =
      this->create_client<std_srvs::srv::Empty>("/system_service/enable");
  while (!robotic_enable_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for robotic init service /system_service/enable...");
  }
  auto request_enable = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result_enable = robotic_enable_->async_send_request(request_enable);
  if (result_enable.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Robotic init service call succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Robotic init service call failed or timed out!");
  }

  gripper_force_ = this->create_client<lebai_interfaces::srv::SetGripper>(
      "/io_service/set_gripper_force");
  while (!gripper_force_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for gripper force init service "
                "/io_service/set_gripper_force...");
  }
  auto request_force =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_force->val = 0.0;
  auto result_force = gripper_force_->async_send_request(request_force);
  if (result_force.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(),
                "Gripper force init service call succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Gripper force init service call failed or timed out!");
  }

  gripper_position_ = this->create_client<lebai_interfaces::srv::SetGripper>(
      "/io_service/set_gripper_position");
  while (!gripper_position_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for gripper position init service "
                "/io_service/set_gripper_position...");
  }
  auto request_position =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_position->val = 0.0;
  auto result_position =
      gripper_position_->async_send_request(request_position);
  if (result_position.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(),
                "Gripper position init service call succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Gripper position init service call failed or timed out!");
  }
}

void GrabbingNode::KeyCallback(const std_msgs::msg::String::SharedPtr msg) {
  char key = msg->data[0];
  if (key == 'q') {
    MoveToInitial();
  } else if (key == 'w') {
    MoveToTarget();
  } else if (key == 'e') {
    TargetGrabbing();
  } else if (key == 'r') {
    StopMotion();
  }
}

void GrabbingNode::MoveToInitial() {
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.25;
  initial_pose.position.y = 0.0;
  initial_pose.position.z = 0.5;
  initial_pose.orientation.w = 1.0;

  move_group_->setPoseTarget(initial_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(this->get_logger(),
                "Initial pose planning succeeded, starting movement...");
    // move_group_->execute(plan);
    bool executed =
        (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
      RCLCPP_WARN(this->get_logger(), "Move to initial position failed!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Move to initial position succeeded!");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Path planning for initial position failed!");
  }
}

void GrabbingNode::MoveToTarget() {
  if (!target_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Service %s unavailable!",
                 kTargetDefaultClient);
    return;
  }

  auto request_target =
      std::make_shared<zed_interfaces::srv::SetPos::Request>();
  auto result_target = target_client_->async_send_request(request_target);
  if (result_target.wait_for(std::chrono::seconds(3)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call the service or timed out!");
    return;
  }

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = result_target.get()->position[0];
  target_pose.position.y = result_target.get()->position[1];
  target_pose.position.z = result_target.get()->position[2];
  target_pose.orientation.w = 1.0;

  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(),
                "Target point acquired, starting execution...");
    bool executed =
        (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
      RCLCPP_WARN(this->get_logger(), "Move to target position failed!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Move to target position succeeded!");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Path planning for target position failed!");
  }
}

void GrabbingNode::TargetGrabbing() {
  auto request_force =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_force->val = 0.3;
  auto result_force = gripper_force_->async_send_request(request_force);
  if (result_force.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Gripper force service call succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Gripper force service call failed or timed out!");
  }

  auto request_position =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_position->val = 0.3;
  auto result_position =
      gripper_position_->async_send_request(request_position);
  if (result_position.wait_for(std::chrono::seconds(3)) ==
      std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Gripper position service call succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Gripper position service call failed or timed out!");
  }
}

void GrabbingNode::StopMotion() {
  RCLCPP_WARN(this->get_logger(), "Emergency stop triggered!");
  move_group_->stop();
}

}  // namespace grabbing
}  // namespace RoboticStereoVision
