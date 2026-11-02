cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu C CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs diagnostic_msgs message_generation)

# The ROS 2 package holds the shared driver and the C core; msg/ is a copy of
# its message (ros/sync_copies.sh).
set(HIPNUC_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../hipnuc_imu")
if(NOT EXISTS "${HIPNUC_SHARED_DIR}/src/driver.cpp")
  message(FATAL_ERROR "Shared sources not found at ${HIPNUC_SHARED_DIR}; build from a complete SDK checkout.")
endif()

add_message_files(FILES HipnucImu.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs diagnostic_msgs message_runtime)

set(core "${HIPNUC_SHARED_DIR}/hipnuc")
add_library(hipnuc_c STATIC
  ${core}/hipnuc_dec.c ${core}/nmea_dec.c ${core}/hipnuc_sample.c
  ${core}/hipnuc_serial.c ${core}/serial_io.c)
target_include_directories(hipnuc_c PUBLIC ${core})
set_target_properties(hipnuc_c PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)

add_executable(hipnuc_imu_node src/imu_node.cpp "${HIPNUC_SHARED_DIR}/src/driver.cpp")
target_include_directories(hipnuc_imu_node PRIVATE ${catkin_INCLUDE_DIRS} "${HIPNUC_SHARED_DIR}/src")
target_link_libraries(hipnuc_imu_node hipnuc_c ${catkin_LIBRARIES})
add_dependencies(hipnuc_imu_node ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(hipnuc_imu_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

install(TARGETS hipnuc_imu_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
