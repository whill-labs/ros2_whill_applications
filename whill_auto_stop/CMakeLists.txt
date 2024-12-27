cmake_minimum_required(VERSION 3.8)
project(whill_auto_stop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(twist_filter_node src/twist_filter_node.cpp)
ament_target_dependencies(twist_filter_node
  geometry_msgs
  rclcpp
  std_msgs
)
install(TARGETS twist_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(laserscan_filter_node src/laserscan_filter_node.cpp)
ament_target_dependencies(laserscan_filter_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
install(TARGETS laserscan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # uncurstifyの代わりにclang-formatを使う
  find_package(ament_cmake_clang_format REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
