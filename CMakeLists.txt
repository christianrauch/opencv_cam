cmake_minimum_required(VERSION 3.5)
project(opencv_cam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

add_library(CameraNode SHARED src/CameraNode.cpp)
rclcpp_components_register_nodes(CameraNode "CameraNode")

ament_target_dependencies(
  CameraNode
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "camera_info_manager"
  "OpenCV"
)
install(TARGETS CameraNode
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
