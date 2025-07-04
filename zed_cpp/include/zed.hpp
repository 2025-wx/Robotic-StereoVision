/** Robotic-StereoVision License **/

#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "utils.h"
#include "zed_interfaces/msg/obj.hpp"
#include "zed_interfaces/msg/trk.hpp"

namespace RoboticStereoVision {
namespace vision {

const std::vector<std::string> RSV_CLASSES = {"Subsurface buoy",
                                              "Mooring line"};

class ZedNode : public rclcpp::Node {
 public:
  explicit ZedNode(const std::string &name);
  ~ZedNode() override;

 private:
  void ZedInit();

  void zed_timer_callback();

  rclcpp::TimerBase::SharedPtr zed_timer_;
  rclcpp::Publisher<zed_interfaces::msg::Trk>::SharedPtr det_pub_;
  sl::Objects objs;
  std::vector<std::vector<int>> colors;
  std::string class_name;
  // std_msgs::msg::String det_msg;

  sl::Camera zed;

  sl::Mat left_sl;
  cv::Mat left_cv;
  cv::Mat res;
  sl::InitParameters init_parameters;
  sl::ObjectDetectionParameters detection_params;
  sl::CustomObjectDetectionRuntimeParameters customObjectTracker_rt;

  constexpr bool enable_tracking = true;
};
}  // namespace vision
}  // namespace RoboticStereoVision