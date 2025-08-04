void ZedNode::InferenceThreadFunc() {
  using clock = std::chrono::steady_clock;
  const std::chrono::milliseconds frame_duration(40);
  auto last_time = clock::now();
  int frame_index = 0;

  // 新增：声明点云 Mat
  sl::Mat point_cloud;

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

    // 获取检测对象
    zed.retrieveCustomObjects(objs, customObjectTracker_rt);

    // 获取点云
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

    // ===== 图像预处理 =====
    cv::Mat gs_frame, hsv, eroded, inRange_hsv;
    cv::GaussianBlur(frame, gs_frame, cv::Size(5, 5), 0);
    cv::cvtColor(gs_frame, hsv, cv::COLOR_BGR2HSV);
    cv::erode(hsv, eroded, cv::Mat(), cv::Point(-1, -1), 2);
    cv::Scalar lower_white(0, 0, 200);
    cv::Scalar upper_white(180, 30, 255);
    cv::inRange(eroded, lower_white, upper_white, inRange_hsv);

    auto det_msg = zed_interfaces::msg::Trk();
    cv::Mat res = left_cv.clone();

    for (const sl::ObjectData & obj : objs.object_list) {
      if (!renderObject(obj, true))
        continue;

      cv::Rect rect;
      cv::Scalar color;
      float center_x = 0.0f; 
      float center_y = 0.0f;

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

      auto top_left = obj.bounding_box_2d[0U];    
      auto bottom_right = obj.bounding_box_2d[2U]; 
      center_x = (top_left.x + bottom_right.x) / 2.0f;
      center_y = (top_left.y + bottom_right.y) / 2.0f;

      // 二值化修正
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
              if (nx >= 0 && nx < inRange_hsv.cols &&
                  ny >= 0 && ny < inRange_hsv.rows &&
                  inRange_hsv.at<uchar>(ny, nx) != 0) {
                center_x = nx;
                center_y = ny;
                found = true;
                break;
              }
              if (dy != 0) {
                ny = center_y + dy;
                if (nx >= 0 && nx < inRange_hsv.cols &&
                    ny >= 0 && ny < inRange_hsv.rows &&
                    inRange_hsv.at<uchar>(ny, nx) != 0) {
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

      // ===== 使用点云反投影获取三维坐标 =====
      sl::float4 point3D;
      point_cloud.getValue((int)center_x, (int)center_y, &point3D);
      float world_x = point3D.x;
      float world_y = point3D.y;
      float world_z = point3D.z;

      cv::rectangle(res, rect, color, 2);
      cv::circle(res, cv::Point((int)center_x, (int)center_y), 4, {0,255,0}, -1);

      char text[256U];
      class_name = "Unknown";
      if (obj.raw_label >= 0 &&
          obj.raw_label < static_cast<int>(RSV_CLASSES.size())) {
        class_name = RSV_CLASSES[obj.raw_label];
      }

      float distance = -1.0f;
      if (std::isfinite(world_z)) {
        distance = world_z; 
        sprintf(text, "%s - %.1f%% - Dist: %.2fm",
                class_name.c_str(), obj.confidence, distance);
      } else {
        sprintf(text, "%s - %.1f%% - Dist: N/A",
                class_name.c_str(), obj.confidence);
      }

      // 填充 ROS 消息 (用反投影坐标替代 obj.position)
      zed_interfaces::msg::Obj trk_data;
      trk_data.label_id = obj.raw_label;
      trk_data.label = class_name;
      trk_data.confidence = obj.confidence;
      trk_data.position[0] = world_x;
      trk_data.position[1] = world_y;
      trk_data.position[2] = world_z;

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
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
    }

    if (show_window_) {
      cv::imshow("ZED", res);
      cv::waitKey(1);
    }

    det_pub_->publish(det_msg);
  }
}
