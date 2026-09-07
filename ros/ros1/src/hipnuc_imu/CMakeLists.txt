cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs diagnostic_msgs message_generation)

add_message_files(FILES HipnucImu.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs diagnostic_msgs message_runtime)

# The C core lives in the SDK repository. Point HIPNUC_CORE_DIR elsewhere when
# this package is copied into another workspace.
set(HIPNUC_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../c/hipnuc" CACHE PATH "HiPNUC C core directory")
set(HIPNUC_ROS_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../common" CACHE PATH "Shared ROS helpers")
add_subdirectory(${HIPNUC_CORE_DIR} hipnuc_core EXCLUDE_FROM_ALL)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${catkin_INCLUDE_DIRS} ${HIPNUC_ROS_COMMON_DIR})

add_executable(serial_node src/serial_node.cpp)
add_dependencies(serial_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(serial_node hipnuc_core ${catkin_LIBRARIES})

add_executable(can_node src/can_node.cpp)
add_dependencies(can_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(can_node hipnuc_j1939 ${catkin_LIBRARIES})

install(TARGETS serial_node can_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
