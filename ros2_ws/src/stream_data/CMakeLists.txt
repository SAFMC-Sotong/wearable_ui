cmake_minimum_required(VERSION 3.8)
project(stream_data)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create executable
add_executable(stream_data src/stream_data.cpp)
target_include_directories(stream_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  stream_data
  rclcpp
  mavros_msgs
)

# Install executable
install(TARGETS stream_data
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if they exist
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.launch.py"
)

# Package dependencies
ament_package()