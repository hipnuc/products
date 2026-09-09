cmake_minimum_required(VERSION 3.16)
project(hipnuc_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hipnuc_msgs REQUIRED)

# Build from a full SDK checkout placed in the workspace's src directory.
set(HIPNUC_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../c/hipnuc" CACHE PATH "HiPNUC C core directory")
set(HIPNUC_ROS_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../common" CACHE PATH "Shared ROS helpers")
add_subdirectory("${HIPNUC_CORE_DIR}" hipnuc_core EXCLUDE_FROM_ALL)
add_subdirectory("${HIPNUC_CORE_DIR}/../serial" hipnuc_serial EXCLUDE_FROM_ALL)

# Lyrical follows REP 2000 (C++20); older supported distributions use C++17.
if("$ENV{ROS_DISTRO}" STREQUAL "lyrical")
  set(HIPNUC_NODE_CXX_STANDARD 20)
else()
  set(HIPNUC_NODE_CXX_STANDARD 17)
endif()

add_executable(serial_node src/serial_node.cpp)
target_include_directories(serial_node PRIVATE ${HIPNUC_ROS_COMMON_DIR})
target_link_libraries(serial_node PRIVATE hipnuc_serial rclcpp::rclcpp
  ${rcl_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${hipnuc_msgs_TARGETS})

add_executable(can_node src/can_node.cpp)
target_include_directories(can_node PRIVATE ${HIPNUC_ROS_COMMON_DIR})
target_link_libraries(can_node PRIVATE hipnuc_j1939 rclcpp::rclcpp
  ${rcl_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${hipnuc_msgs_TARGETS})

set_target_properties(serial_node can_node PROPERTIES
  CXX_STANDARD ${HIPNUC_NODE_CXX_STANDARD} CXX_STANDARD_REQUIRED YES)
install(TARGETS serial_node can_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
