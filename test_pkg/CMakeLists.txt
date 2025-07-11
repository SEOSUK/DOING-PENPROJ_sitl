cmake_minimum_required(VERSION 3.8)
project(test_pkg)

set(CMAKE_CXX_STANDARD 20)

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
find_package(crazyflie_interfaces REQUIRED)  # 추가

find_package(rosbag2_cpp REQUIRED)
find_package(turtlesim REQUIRED)



# Gazebo transport
find_package(gz-msgs11 REQUIRED)
find_package(gz-transport12 REQUIRED)



# WRENCH_BRIDGE
add_executable(wrench_bridge src/wrench_bridge.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(wrench_bridge
  rclcpp
  geometry_msgs
  crazyflie_interfaces
  tf2
  tf2_ros
)

install(TARGETS
  wrench_bridge
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(wrench_bridge PUBLIC
  ${gz-msgs11_INCLUDE_DIRS}
  ${gz-transport12_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(wrench_bridge ButterworthFilter
  ${gz-msgs11_LIBRARIES}
  ${gz-transport12_LIBRARIES}
)




#SU_RVIZ
add_executable(su_rviz src/su_rviz.cpp)
ament_target_dependencies(su_rviz
 rclcpp
 std_msgs
 sensor_msgs
 ros_gz_interfaces
 geometry_msgs
 tf2
 tf2_ros
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
  su_rviz
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(su_rviz PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
)

target_link_libraries(su_rviz)

#SU_FKIK
add_executable(su_fkik src/su_fkik.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(su_fkik
 rclcpp
 std_msgs
 sensor_msgs
 ros_gz_interfaces
 geometry_msgs
 tf2
 tf2_ros
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
  su_fkik
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(su_fkik PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
)

target_link_libraries(su_fkik ButterworthFilter)

#JTAJECTORY_GENERATOR
add_executable(trajectory_generator src/trajectory_generator.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(trajectory_generator
 rclcpp
 std_msgs
 sensor_msgs
 ros_gz_interfaces
 geometry_msgs
 tf2
 tf2_ros
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
trajectory_generator
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(trajectory_generator PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
)

target_link_libraries(trajectory_generator ButterworthFilter)


#keyboard_publisher
add_executable(keyboard_publisher src/keyboard_publisher.cpp)
ament_target_dependencies(keyboard_publisher
 rclcpp
 std_msgs
 sensor_msgs
 ros_gz_interfaces
 geometry_msgs
 tf2
 tf2_ros
 visualization_msgs
 crazyflie_interfaces)

install(TARGETS
keyboard_publisher
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(keyboard_publisher PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory
 ${CURSES_INCLUDE_DIR}
)

target_link_libraries(keyboard_publisher ${CURSES_LIBRARIES})



# DATA_DECRYPTOR

add_executable(data_decryptor src/data_decryptor.cpp)

ament_target_dependencies(data_decryptor
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
)

install(TARGETS
  data_decryptor
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(data_decryptor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



#butterworth filter
add_library(ButterworthFilter src/ButterworthFilter.cpp)
target_include_directories(ButterworthFilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)


#DATA_LOGGING
add_executable(data_logging src/data_logging.cpp)
ament_target_dependencies(data_logging
  rclcpp
  rosbag2_cpp
  geometry_msgs
  std_msgs
  turtlesim
  crazyflie_interfaces
)

install(TARGETS
  data_logging
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(data_logging PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}
)

#CF_COMMUNICATOR
add_executable(cf_communicator src/cf_communicator.cpp)
ament_target_dependencies(cf_communicator
  rclcpp
  geometry_msgs
  std_msgs
  crazyflie_interfaces
)

install(TARGETS
  cf_communicator
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(cf_communicator PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}
)

#CF_COMMUNICATOR_SIM
add_executable(cf_communicator_sim src/cf_communicator_sim.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(cf_communicator_sim
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  crazyflie_interfaces
)

install(TARGETS
  cf_communicator_sim
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(cf_communicator_sim PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(cf_communicator_sim ButterworthFilter)

# WRENCH_OBSERVER
add_executable(wrench_observer src/wrench_observer.cpp)
ament_target_dependencies(wrench_observer
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
  crazyflie_interfaces
)

install(TARGETS
  wrench_observer
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(wrench_observer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)


###


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# config 파일 설치
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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