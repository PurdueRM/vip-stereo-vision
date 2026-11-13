cmake_minimum_required(VERSION 3.8)
project(stereo_pipeline)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Input node
add_executable(replay_video_file_input_node
  src/replay_video_file_input_node.cpp
)

ament_target_dependencies(replay_video_file_input_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(replay_video_file_input_node
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

# Stereo vision node
add_executable(stereo_vision_node
  src/stereo_vision_node.cpp
)

ament_target_dependencies(stereo_vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(stereo_vision_node
  ${OpenCV_LIBRARIES}
)

# Output node
add_executable(write_video_file_output_node
  src/write_video_file_output_node.cpp
)

ament_target_dependencies(write_video_file_output_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(write_video_file_output_node
  ${OpenCV_LIBRARIES}
)

add_executable(armor_isolation_node
  src/armor_isolation_node.cpp
)

ament_target_dependencies(armor_isolation_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(armor_isolation_node
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS
  replay_video_file_input_node
  stereo_vision_node
  write_video_file_output_node
  armor_isolation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and configuration files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
