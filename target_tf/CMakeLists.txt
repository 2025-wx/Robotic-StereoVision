cmake_minimum_required(VERSION 3.8)
project(target_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)

include_directories(
  include
  ${TERMBITS_INCLUDE_DIRS}
)

link_directories(
  ${TERMBITS_LIBRARY_DIRS} 
  ${catkin_LIB_DIRS}
)

set(TERMBITS_LIBRARY_DIRS /usr/lib)
set(TERMBITS_INCLUDE_DIRS /usr/lib/include)

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

add_executable(target_server src/main.cpp src/target.cpp)
ament_target_dependencies(target_server rclcpp tf2_ros tf2_geometry_msgs zed_interfaces)

install(TARGETS
  target_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
