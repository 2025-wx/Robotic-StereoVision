/** Robotic-StereoVision License **/

#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>

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

  bool show_window_;
  int grab_failure_count_ = 0;
  std::string zed_publisher_name;
  std::string onnx_path_;

  sl::Camera zed;
  sl::Mat left_sl;
  cv::Mat left_cv;
  sl::Objects objs;
  std::string class_name;
  sl::InitParameters init_parameters;
  rclcpp::TimerBase::SharedPtr zed_timer_;
  sl::ObjectDetectionParameters detection_params;
  rclcpp::Publisher<zed_interfaces::msg::Trk>::SharedPtr det_pub_;
  sl::CustomObjectDetectionRuntimeParameters customObjectTracker_rt;
};
}  // namespace vision
}  // namespace RoboticStereoVision