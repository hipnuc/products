cmake_minimum_required(VERSION 3.16)
project(hipnuc_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hipnuc_msgs REQUIRED)

# The C core lives in the SDK repository. Point HIPNUC_CORE_DIR elsewhere when
# this package is copied into another workspace.
set(HIPNUC_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../c/hipnuc" CACHE PATH "HiPNUC C core directory")
set(HIPNUC_ROS_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../common" CACHE PATH "Shared ROS helpers")
add_subdirectory(${HIPNUC_CORE_DIR} hipnuc_core EXCLUDE_FROM_ALL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(serial_node src/serial_node.cpp)
target_include_directories(serial_node PRIVATE ${HIPNUC_ROS_COMMON_DIR})
target_link_libraries(serial_node hipnuc_core)
ament_target_dependencies(serial_node rclcpp sensor_msgs geometry_msgs diagnostic_msgs hipnuc_msgs)

add_executable(can_node src/can_node.cpp)
target_include_directories(can_node PRIVATE ${HIPNUC_ROS_COMMON_DIR})
target_link_libraries(can_node hipnuc_j1939)
ament_target_dependencies(can_node rclcpp sensor_msgs geometry_msgs diagnostic_msgs hipnuc_msgs)

install(TARGETS serial_node can_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
