cmake_minimum_required(VERSION 3.5)
project(kinova_vision)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)

# System Dependencies
find_package(PkgConfig)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

# Create executables and libraries
add_library(kinova_vision SHARED src/vision.cpp)
target_include_directories(kinova_vision PRIVATE ${ament_INCLUDE_DIRS})
ament_target_dependencies(kinova_vision rclcpp sensor_msgs image_transport camera_calibration_parsers camera_info_manager tf2)
target_link_libraries(kinova_vision 
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

add_executable(kinova_vision_node src/vision_node.cpp)
target_include_directories(kinova_vision_node PRIVATE ${ament_INCLUDE_DIRS})
ament_target_dependencies(kinova_vision_node rclcpp sensor_msgs image_transport camera_calibration_parsers camera_info_manager tf2)
target_link_libraries(kinova_vision_node kinova_vision)

# Install directives
install(
  TARGETS
    kinova_vision
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(
  TARGETS
    kinova_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY launch/calibration
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.ini"
)

# Export information to other packages
ament_package()
