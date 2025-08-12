/** Robotic-StereoVision License **/

#include "grabbing.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace RoboticStereoVision {
namespace grabbing {

namespace {
constexpr const char *kTargetClient = "target_client_name";
constexpr const char *kTargetDefaultClient = "target_position";
constexpr const char *kMoveGroup = "move_group_name";
constexpr const char *kDefaultMoveGroup = "robotic";
constexpr const char *kKeyPublisher = "key_publisher";
constexpr const char *kDefaultKeyPublisher = "/key_input";
}  // namespace

GrabbingNode::GrabbingNode(const std::string &name,
                           const rclcpp::NodeOptions &options)
    : Node(name, options) {
  std::string key_publisher_name;
  std::string target_client_name;
  this->declare_parameter(kMoveGroup, std::string(kDefaultMoveGroup));
  this->get_parameter_or(kMoveGroup, move_group_name_,
                         std::string(kDefaultMoveGroup));
  this->declare_parameter(kTargetClient, std::string(kTargetDefaultClient));
  this->get_parameter_or(kTargetClient, target_client_name,
                         std::string(kTargetDefaultClient));
  this->declare_parameter(kKeyPublisher, std::string(kDefaultKeyPublisher));
  this->get_parameter_or(kKeyPublisher, key_publisher_name,
                         std::string(kDefaultKeyPublisher));

  target_client_ =
      this->create_client<zed_interfaces::srv::SetPos>(target_client_name);

  key_pub_ =
      this->create_publisher<std_msgs::msg::String>(key_publisher_name, 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GrabbingNode::ReadAndPublishKey, this));

  key_sub_ = this->create_subscription<std_msgs::msg::String>(
      key_publisher_name, 10,
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
  auto result_enable = robotic_enable_->async_send_request(
      request_enable,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        if (future.valid()) {
          RCLCPP_INFO(this->get_logger(),
                      "Robotic init service call succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Robotic init service call failed!");
        }
      });

  gripper_force_ = this->create_client<lebai_interfaces::srv::SetGripper>(
      "/io_service/set_gripper_force");
  while (!gripper_force_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for gripper force init service "
                "/io_service/set_gripper_force...");
  }
  auto request_force =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_force->val = 50.0;
  auto result_force = gripper_force_->async_send_request(
      request_force,
      [this](rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedFuture
                 future) {
        if (future.valid()) {
          RCLCPP_INFO(this->get_logger(),
                      "Gripper force init service call succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Gripper force init service call failed!");
        }
      });

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
  auto result_position = gripper_position_->async_send_request(
      request_position,
      [this](rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedFuture
                 future) {
        if (future.valid()) {
          RCLCPP_INFO(this->get_logger(),
                      "Gripper position init service call succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Gripper position init service call failed!");
        }
      });
}

void GrabbingNode::ReadAndPublishKey() {
  char c;
  if (ReadKeyboardNonBlocking(c)) {
    std_msgs::msg::String msg;
    msg.data = std::string(1, c);
    key_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published key: '%c'", c);
  }
}

bool GrabbingNode::ReadKeyboardNonBlocking(char &c) {
  struct termios oldt, newt;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  int nread = read(STDIN_FILENO, &c, 1);

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  return nread == 1;
}

void GrabbingNode::KeyCallback(const std_msgs::msg::String::SharedPtr msg) {
  char key = msg->data[0];
  if (key == 'q') {
    MoveToInitial();
  } else if (key == 'w') {
    MoveToTarget();
  } else if (key == 'e') {
    GripperClosed();
  } else if (key == 'r') {
    GripperOpened();
  } else if (key == 'd') {
    StopMotion();
  }
}

void GrabbingNode::MoveToInitial() {
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 0.25;
  initial_pose.position.y = 0.0;
  initial_pose.position.z = 0.5;
  initial_pose.orientation.w = 1.0;

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  const moveit::core::JointModelGroup *joint_model_group =
      current_state->getJointModelGroup(move_group_->getName());

  bool found_ik = current_state->setFromIK(joint_model_group, initial_pose);
  if (!found_ik) {
    RCLCPP_ERROR(this->get_logger(), "IK solution not found for initial pose!");
    return;
  }

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);

  move_group_->setJointValueTarget(joint_group_positions);

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
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result_target) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call the service!");
    return;
  }

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = result_target.get()->position[0];
  target_pose.position.y = result_target.get()->position[1];
  target_pose.position.z = result_target.get()->position[2];
  target_pose.orientation.w = 1.0;

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
  const moveit::core::JointModelGroup *joint_model_group =
      current_state->getJointModelGroup(move_group_->getName());

  bool found_ik = current_state->setFromIK(joint_model_group, target_pose);
  if (!found_ik) {
    RCLCPP_ERROR(this->get_logger(), "IK solution not found for target pose!");
    return;
  }

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);

  move_group_->setJointValueTarget(joint_group_positions);

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

void GrabbingNode::GripperClosed() {
  auto request_position =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_position->val = 0.0;
  auto result_position = gripper_position_->async_send_request(
      request_position,
      [this](rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedFuture
                 future) {
        if (future.valid()) {
          RCLCPP_INFO(this->get_logger(),
                      "Gripper position init service call succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Gripper position init service call failed!");
        }
      });
}

void GrabbingNode::GripperOpened() {
  auto request_position =
      std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
  request_position->val = 100.0;
  auto result_position = gripper_position_->async_send_request(
      request_position,
      [this](rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedFuture
                 future) {
        if (future.valid()) {
          RCLCPP_INFO(this->get_logger(),
                      "Gripper position init service call succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Gripper position init service call failed!");
        }
      });
}

void GrabbingNode::StopMotion() {
  RCLCPP_WARN(this->get_logger(), "Emergency stop triggered!");
  move_group_->stop();
}

}  // namespace grabbing
}  // namespace RoboticStereoVision
