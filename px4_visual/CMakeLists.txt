cmake_minimum_required(VERSION 3.8)
project(px4_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # Add Eigen3
find_package(Curses REQUIRED)


# PX4_PLAYER
add_executable(px4_player src/px4_player.cpp)
ament_target_dependencies(px4_player
  rclcpp
  std_msgs
  geometry_msgs
)

# PX4_RVIZ
add_executable(px4_rviz src/px4_rviz.cpp)
ament_target_dependencies(px4_rviz
rclcpp
std_msgs
sensor_msgs
geometry_msgs
tf2
tf2_ros
visualization_msgs
)

install(TARGETS
  px4_player
  px4_rviz
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(px4_player PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

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

ament_package()
