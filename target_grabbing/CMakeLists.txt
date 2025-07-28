cmake_minimum_required(VERSION 3.8)
project(target_grabbing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(lebai_interfaces REQUIRED)

include_directories(
  include
)

add_executable(grabbing_node src/main.cpp src/grabbing.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_target_dependencies(grabbing_node
  rclcpp
  std_msgs
  tf2_geometry_msgs
  moveit_ros_planning_interface
  zed_interfaces
  lebai_interfaces
)

install(TARGETS grabbing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
