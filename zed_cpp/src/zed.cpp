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
// #include "zed_interfaces/msg/obj.hpp"
// #include "zed_interfaces/msg/trk.hpp"

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
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
}

void ZedNode::InferenceThreadFunc() {
  using clock = std::chrono::steady_clock;
  const std::chrono::milliseconds frame_duration(40);
  auto last_time = clock::now();
  int frame_index = 0;  
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
    // ===== Image preprocessing =====
    cv::Mat gs_frame, hsv, eroded, inRange_hsv;
    // Gaussian blur
    cv::GaussianBlur(frame, gs_frame, cv::Size(5, 5), 0);
    // Convert to HSV color space
    cv::cvtColor(gs_frame, hsv, cv::COLOR_BGR2HSV);
    // Erosion processing
    cv::erode(hsv, eroded, cv::Mat(), cv::Point(-1, -1), 2);
    // White color threshold range (adjustable)
    cv::Scalar lower_white(0, 0, 200);
    cv::Scalar upper_white(180, 30, 255);
    // Generate binary image using thresholding
    cv::inRange(eroded, lower_white, upper_white, inRange_hsv);
    // Save binary result to file
    std::string filename = "binary_result_" + std::to_string(frame_index++) + ".jpg";

    auto det_msg = zed_interfaces::msg::Trk();
    cv::Mat res = left_cv.clone();

    for (const sl::ObjectData & obj : objs.object_list) {
      if (!renderObject(obj, true))
        continue;

      cv::Rect rect;
      cv::Scalar color;
      float center_x = 0.0f; 
      float center_y = 0.0f;
      if (show_window_) {
        size_t const idx_color{obj.id % CLASS_COLORS.size()};
        color = cv::Scalar(CLASS_COLORS[idx_color][0U],
                           CLASS_COLORS[idx_color][1U],
                           CLASS_COLORS[idx_color][2U]);
        rect = cv::Rect(static_cast<int>(obj.bounding_box_2d[0U].x),
                        static_cast<int>(obj.bounding_box_2d[0U].y),
                        static_cast<int>(obj.bounding_box_2d[1U].x -
                                         obj.bounding_box_2d[0U].x),
                        static_cast<int>(obj.bounding_box_2d[2U].y -
                                         obj.bounding_box_2d[0U].y));

        auto top_left = obj.bounding_box_2d[0U];    // Top-left vertex
        auto bottom_right = obj.bounding_box_2d[2U]; // Bottom-right vertex
        // Calculate the center point coordinates (average of x and y respectively)
        center_x = (top_left.x + bottom_right.x) / 2.0f;
        center_y = (top_left.y + bottom_right.y) / 2.0f;
        if (center_x >= 0 && center_x < inRange_hsv.cols &&
        center_y >= 0 && center_y < inRange_hsv.rows) {
        
        if (inRange_hsv.at<uchar>(center_y, center_x) == 0) {

            bool found = false;
            const int max_search_radius = 20; 
            for (int r = 1; r <= max_search_radius && !found; ++r) {
                for (int dx = -r; dx <= r && !found; ++dx) {
                    int dy = r - abs(dx);
                    int nx = center_x + dx;
                    int ny = center_y - dy;
                    if (nx >= 0 && nx < inRange_hsv.cols && ny >= 0 && ny < inRange_hsv.rows) {
                        if (inRange_hsv.at<uchar>(ny, nx) != 0) {
                            center_x = nx;
                            center_y = ny;
                            found = true;
                            break;
                        }
                    }
                    if (dy != 0) {
                        ny = center_y + dy;
                        if (nx >= 0 && nx < inRange_hsv.cols && ny >= 0 && ny < inRange_hsv.rows) {
                            if (inRange_hsv.at<uchar>(ny, nx) != 0) {
                                center_x = nx;
                                center_y = ny;
                                found = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
        cv::rectangle(res, rect, color, 2);
      }

      char text[256U];
      class_name = "Unknown";
      if (obj.raw_label >= 0 &&
          obj.raw_label < static_cast<int>(RSV_CLASSES.size())) {
        class_name = RSV_CLASSES[obj.raw_label];
      }

      float distance = -1.0f;
      if (!std::isnan(obj.position.z)) {
        distance = -obj.position.z;
        if (show_window_) {
          sprintf(text, "%s - %.1f%% - Dist: %.2fm", class_name.c_str(),
                  obj.confidence, distance);
        }
      }

      zed_interfaces::msg::Obj trk_data;
      trk_data.label_id = obj.raw_label;
      trk_data.label = class_name;
      trk_data.confidence = obj.confidence;
      trk_data.position[0] = static_cast<float>(center_x);
      trk_data.position[1] = static_cast<float>(center_y);
      trk_data.position[2] = obj.position.z;
      det_msg.objects.push_back(trk_data);

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
    }

    if (show_window_) {
      cv::imshow("ZED", res);
      cv::waitKey(1);
    }

    det_pub_->publish(det_msg);
  }
}


}  // namespace vision
} // namespace RoboticStereoVision