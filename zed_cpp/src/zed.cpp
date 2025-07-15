/** Robotic-StereoVision License **/

#include "zed.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <thread>

#include "utils.h"
#include "zed_interfaces/msg/obj.hpp"
#include "zed_interfaces/msg/trk.hpp"

namespace RoboticStereoVision {
namespace vision {

namespace {
constexpr const char *kShowWindow = "show_window";
constexpr bool kZedDefaultShowWindow = true;
constexpr const char *kZedPublisher = "zed_publisher_name";
constexpr const char *kZedDefaultPublisher = "/zed/detections";
constexpr const char *kOnnxPath = "onnx_path";
constexpr const char *kZedDefaultOnnxPath =
    "/home/nxsuper/rs_ws/src/model_yolov8/yolov8.onnx";
}  // namespace

ZedNode::ZedNode(const std::string &name) : Node(name) {
  this->declare_parameter(kShowWindow, kZedDefaultShowWindow);
  this->get_parameter_or(kShowWindow, show_window_, kZedDefaultShowWindow);
  this->declare_parameter(kZedPublisher, std::string(kZedDefaultPublisher));
  this->get_parameter_or(kZedPublisher, zed_publisher_name,
                         std::string(kZedDefaultPublisher));
  this->declare_parameter(kOnnxPath, std::string(kZedDefaultOnnxPath));
  this->get_parameter_or(kOnnxPath, onnx_path_,
                         std::string(kZedDefaultOnnxPath));

  det_pub_ =
      this->create_publisher<zed_interfaces::msg::Trk>(zed_publisher_name, 10);

  ZedInit();

  camera_thread_ = std::thread(&ZedNode::CameraThreadFunc, this);
  inference_thread_ = std::thread(&ZedNode::InferenceThreadFunc, this);
}

ZedNode::~ZedNode() {
  running_ = false;
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }
  if (show_window_) {
    cv::destroyAllWindows();
  }
  zed.close();
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

  detection_params.enable_tracking = true;
  detection_params.enable_segmentation = true;
  detection_params.detection_model =
      sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
  detection_params.custom_onnx_file.set(onnx_path_.c_str());
  detection_params.custom_onnx_dynamic_input_shape = sl::Resolution(320, 320);

  const sl::ERROR_CODE od_ret = zed.enableObjectDetection(detection_params);
  if (od_ret != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable object detection!");
    zed.close();
    rclcpp::shutdown();
    return;
  }

  customObjectTracker_rt.object_detection_properties
      .detection_confidence_threshold = 20.f;
  printf(
      "Custom Object Detection runtime parameters: confidence threshold set "
      "to "
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
      "Custom Object Detection runtime parameters: Label 1, min box width "
      "set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .min_box_width_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .max_box_width_normalized = 0.5f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, max box width "
      "set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .max_box_width_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .min_box_height_normalized = 0.01f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, min box height "
      "set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .min_box_height_normalized);

  customObjectTracker_rt.object_class_detection_properties[1U]
      .max_box_height_normalized = 0.5f;
  printf(
      "Custom Object Detection runtime parameters: Label 1, max box height "
      "set "
      "to %.2f\n",
      customObjectTracker_rt.object_class_detection_properties[1U]
          .max_box_height_normalized);

  if (show_window_) {
    cv::namedWindow("ZED", cv::WINDOW_NORMAL);
    cv::resizeWindow("ZED", 1536, 864);
  }
}

void ZedNode::CameraThreadFunc() {
  while (running_) {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      sl::Mat left_sl;
      zed.retrieveImage(left_sl, sl::VIEW::LEFT);
      cv::Mat frame = slMat2cvMat(left_sl);

      if (!frame.empty()) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (frame_queue_.size() < 5) {
          frame_queue_.push(frame.clone());
        }
      }
    } else {
      grab_failure_count_++;
      RCLCPP_WARN(this->get_logger(), "ZED grab failed (%d times)",
                  grab_failure_count_);
      if (grab_failure_count_ >= 30) {
        RCLCPP_ERROR(this->get_logger(),
                     "ZED grab failed too many times. Reinitializing.");
        zed.close();
        ZedInit();
        grab_failure_count_ = 0;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void ZedNode::InferenceThreadFunc() {
  using clock = std::chrono::steady_clock;
  const std::chrono::milliseconds frame_duration(40);
  auto last_time = clock::now();

  while (running_) {
    auto now = clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
    if (elapsed < frame_duration) {
      std::this_thread::sleep_for(frame_duration - elapsed);
      continue;
    }
    last_time = clock::now();

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!frame_queue_.empty()) {
        frame = frame_queue_.front();
        frame_queue_.pop();
      }
    }

    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    left_cv = frame;
    zed.retrieveCustomObjects(objs, customObjectTracker_rt);

    auto det_msg = zed_interfaces::msg::Trk();
    cv::Mat res, mask;
    if (show_window_) {
      res = left_cv.clone();
      mask = left_cv.clone();
    }

    for (const sl::ObjectData &obj : objs.object_list) {
      if (!renderObject(obj, true))
        continue;

      cv::Rect rect;
      cv::Scalar color;

      if (show_window_) {
        size_t idx_color = obj.id % CLASS_COLORS.size();
        color =
            cv::Scalar(CLASS_COLORS[idx_color][0U], CLASS_COLORS[idx_color][1U],
                       CLASS_COLORS[idx_color][2U]);
        rect = cv::Rect(static_cast<int>(obj.bounding_box_2d[0U].x),
                        static_cast<int>(obj.bounding_box_2d[0U].y),
                        static_cast<int>(obj.bounding_box_2d[1U].x -
                                         obj.bounding_box_2d[0U].x),
                        static_cast<int>(obj.bounding_box_2d[2U].y -
                                         obj.bounding_box_2d[0U].y));
        cv::rectangle(res, rect, color, 2);
      }

      zed_interfaces::msg::Obj trk_data;
      trk_data.label_id = obj.raw_label;
      trk_data.label =
          (obj.raw_label >= 0 && obj.raw_label < RSV_CLASSES.size())
              ? RSV_CLASSES[obj.raw_label]
              : "Unknown";
      trk_data.confidence = obj.confidence;
      trk_data.position[0] = obj.position.x;
      trk_data.position[1] = obj.position.y;
      trk_data.position[2] = obj.position.z;
      det_msg.objects.push_back(trk_data);

      if (show_window_ && obj.mask.isInit()) {
        const cv::Mat obj_mask = slMat2cvMat(obj.mask);
        mask(rect).setTo(color, obj_mask);

        char text[256U];
        float distance =
            (!std::isnan(obj.position.z)) ? -obj.position.z : -1.0f;
        sprintf(text, "%s - %.1f%% - Dist: %.2fm", trk_data.label.c_str(),
                trk_data.confidence, distance);

        int baseLine;
        auto label_size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        int x = rect.x;
        int y = std::min(rect.y + 1, res.rows);
        cv::rectangle(
            res, cv::Rect(x, y, label_size.width, label_size.height + baseLine),
            {255, 255, 0}, -1);
        cv::putText(res, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
      }
    }

    if (show_window_) {
      cv::addWeighted(res, 1.0, mask, 0.4, 0.0, res);
      cv::imshow("ZED", res);
      cv::waitKey(1);
    }

    det_pub_->publish(det_msg);
  }
}

}  // namespace vision
}  // namespace RoboticStereoVision