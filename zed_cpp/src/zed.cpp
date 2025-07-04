/** Robotic-StereoVision License **/

#include "zed.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <vector>

#include "utils.h"
#include "zed_interfaces/msg/obj.hpp"
#include "zed_interfaces/msg/trk.hpp"

namespace RoboticStereoVision {
namespace vision {

namespace {
constexpr const char *kZedPublisher = "zed_publisher_name";
constexpr const char *kZedDefaultPublisher = "/zed/detections";
}  // namespace

ZedNode::ZedNode(const std::string &name) : Node(name) {
  std::string zed_publisher_name;
  this->get_parameter_or(kZedPublisher, zed_publisher_name,
                         std::string(kZedDefaultPublisher));

  det_pub_ =
      this->create_publisher<zed_interfaces::msg::Trk>(zed_publisher_name, 10);
  ZedInit();
}

ZedNode::~ZedNode() {
  if (zed_timer_ != nullptr) {
    zed_timer_.reset();
    cv::destroyAllWindows();
  }
}

void ZedNode::ZedInit() {
  init_parameters.sdk_verbose = true;
  init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
  init_parameters.coordinate_units = sl::UNIT::METER;
  init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

  const sl::ERROR_CODE open_ret = zed.open(init_parameters);
  if (open_ret != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera!");
    rclcpp::shutdown();
  }

  zed.enablePositionalTracking();

  constexpr bool enable_tracking = true;
  detection_params.enable_tracking = enable_tracking;
  detection_params.enable_segmentation = true;
  detection_params.detection_model =
      sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
  detection_params.custom_onnx_file.set("yolov8.onnx");
  detection_params.custom_onnx_dynamic_input_shape = sl::Resolution(320, 320);

  const sl::ERROR_CODE od_ret = zed.enableObjectDetection(detection_params);
  if (od_ret != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable object detection!");
    zed.close();
    rclcpp::shutdown();
  }

  const sl::CameraConfiguration camera_config =
      zed.getCameraInformation().camera_configuration;
  const sl::Resolution pc_resolution(
      std::min((int)camera_config.resolution.width, 720),
      std::min((int)camera_config.resolution.height, 404));
  const sl::CameraConfiguration camera_info =
      zed.getCameraInformation(pc_resolution).camera_configuration;

  customObjectTracker_rt.object_detection_properties
      .detection_confidence_threshold = 20.f;
  printf(
      "Custom Object Detection runtime parameters: confidence threshold set to "
      "%2.1f for all classes\n",
      customObjectTracker_rt.object_detection_properties
          .detection_confidence_threshold);

  customObjectTracker_rt.object_class_detection_properties[0U]
      .detection_confidence_threshold = 60.f;
  printf(
      "Custom Object Detection runtime parameters: Label 0, confidence "
      "threshold set to %2.1f\n",
      customObjectTracker_rt.object_class_detection_properties[0U]
          .detection_confidence_threshold);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .min_box_width_normalized = 0.01f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, min box width set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .min_box_width_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .max_box_width_normalized = 0.5f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, max box width set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .max_box_width_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .min_box_height_normalized = 0.01f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, min box height set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .min_box_height_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .max_box_height_normalized = 0.5f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, max box height set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .max_box_height_normalized);

  cv::namedWindow("ZED", cv::WINDOW_NORMAL);
  cv::resizeWindow("ZED", 1536, 864);
  printf("ZED camera opened successfully!\n");

  zed_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(30.0)),
                              std::bind(&ZedNode::zed_timer_callback, this));
}

void ZedNode::zed_timer_callback() {
  if (zed.grab() != sl::ERROR_CODE::SUCCESS)
    return;

  zed.retrieveImage(left_sl, sl::VIEW::LEFT);
  left_cv = slMat2cvMat(left_sl);

  zed.retrieveCustomObjects(objs, customObjectTracker_rt);

  auto det_msg = zed_interfaces::msg::Trk();

  res = left_cv.clone();
  cv::Mat mask{left_cv.clone()};

  for (sl::ObjectData const& obj : objs.object_list) {
    if (!renderObject(obj, true))
      continue;

    size_t const idx_color{obj.id % colors.size()};
    cv::Scalar const color{cv::Scalar(
        colors[idx_color][0U], colors[idx_color][1U], colors[idx_color][2U])};

    cv::Rect const rect{
        static_cast<int>(obj.bounding_box_2d[0U].x),
        static_cast<int>(obj.bounding_box_2d[0U].y),
        static_cast<int>(obj.bounding_box_2d[1U].x - obj.bounding_box_2d[0U].x),
        static_cast<int>(obj.bounding_box_2d[2U].y -
                         obj.bounding_box_2d[0U].y)};
    cv::rectangle(res, rect, color, 2);

    char text[256U];
    class_name = "Unknown";

    if (obj.raw_label >= 0 &&
        obj.raw_label < static_cast<int>(RSV_CLASSES.size())) {
      class_name = RSV_CLASSES[obj.raw_label];
    }

    float distance = -1.0f;

    if (!std::isnan(obj.position.z)) {
      distance = obj.position.z;
      sprintf(text, "%s - %.1f%% - Dist: %.2fm", class_name.c_str(),
              obj.confidence, distance);
    } else {
      sprintf(text, "%s - %.1f%%", class_name.c_str(), obj.confidence);
    }

    // det_msg.data += std::string(text) + "\n";
    zed_interfaces::msg::Obj trk_data;
    trk_data.label = class_name;
    trk_data.label_id = obj.raw_label;
    trk_data.confidence = obj.confidence;
    trk_data.position[0] = obj.position.x;
    trk_data.position[1] = obj.position.y;
    trk_data.position[2] = obj.position.z;

    det_msg.objects.push_back(trk_data);

    if (obj.mask.isInit() && obj.mask.getWidth() > 0U &&
        obj.mask.getHeight() > 0U) {
      const cv::Mat obj_mask = slMat2cvMat(obj.mask);
      mask(rect).setTo(color, obj_mask);
    }

    int baseLine{0};
    const cv::Size label_size{
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine)};
    const int x{rect.x};
    const int y{std::min(rect.y + 1, res.rows)};
    cv::rectangle(
        res, cv::Rect(x, y, label_size.width, label_size.height + baseLine),
        {255, 255, 0}, -1);
    cv::putText(res, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 4);
    cv::addWeighted(res, 1.0, mask, 0.4, 0.0, res);
  }

  cv::resizeWindow("ZED", 1200, 700);
  cv::imshow("ZED", left_cv);
  det_pub_->publish(det_msg);
}
}  // namespace vision
}  // namespace RoboticStereoVision