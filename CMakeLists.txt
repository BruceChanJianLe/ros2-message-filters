cmake_minimum_required(VERSION 3.8)
project(ros2-message-filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# BUILD
include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  message_filters
  tf2
  tf2_ros
)

# RCLCPP NODE
add_executable(rclcpp_node_mf
  src/rclcpp_message_filters.cpp
  src/rclcpp_message_filters_node.cpp
)

ament_target_dependencies(
  rclcpp_node_mf
  ${dependencies}
)

# RCLCPP LIFECYCLE NODE
add_executable(rclcpp_lifecycle_node_mf
  src/rclcpp_lifecycle_message_filters.cpp
  src/rclcpp_lifecycle_message_filters_node.cpp
)

ament_target_dependencies(
  rclcpp_lifecycle_node_mf
  ${dependencies}
)

# INSTALL
install(
  TARGETS rclcpp_node_mf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS rclcpp_lifecycle_node_mf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  bags
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
