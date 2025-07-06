cmake_minimum_required(VERSION 3.8)
project(zed_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZED REQUIRED)
find_package(CUDA REQUIRED)
find_package(zed_interfaces REQUIRED)

set(TERMBITS_LIBRARY_DIRS /usr/lib)
set(TERMBITS_INCLUDE_DIRS /usr/lib/include)

include_directories(
  include
  ${TERMBITS_INCLUDE_DIRS}
)

link_directories(
  ${TERMBITS_LIBRARY_DIRS} 
  ${catkin_LIB_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(ZED_LIBS
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

add_executable(zed_pub src/main.cpp src/zed.cpp)
ament_target_dependencies(zed_pub rclcpp std_msgs OpenCV zed_interfaces)

# Optional: Add the macro to enable window display
option(SHOW_WINDOW "Enable OpenCV display window" ON)

if(SHOW_WINDOW)
  target_compile_definitions(zed_pub PRIVATE SHOW_WINDOW)
endif()

target_include_directories(zed_pub PUBLIC
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

target_link_libraries(zed_pub
    ${ZED_LIBS}
)

install(TARGETS
  zed_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
