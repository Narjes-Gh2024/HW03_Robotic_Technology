cmake_minimum_required(VERSION 3.8)
project(robot_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(slam_node
  src/slam_node.cpp
  src/occupancy_grid_map.cpp
  src/scan_matcher.cpp
  src/motion_model.cpp
)

ament_target_dependencies(slam_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_node
  Eigen3::Eigen
)

add_executable(map_saver_node
  src/map_saver_node.cpp
)

ament_target_dependencies(map_saver_node
  rclcpp
  nav_msgs
  std_srvs
)


install(TARGETS
  slam_node
  map_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
