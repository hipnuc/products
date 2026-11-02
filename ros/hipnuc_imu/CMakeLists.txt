cmake_minimum_required(VERSION 3.16)
project(hipnuc_imu C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} msg/HipnucImu.msg DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(hipnuc_imu_interfaces ${PROJECT_NAME} rosidl_typesupport_cpp)

# Copy of the SDK's C decoder and serial port, kept identical by ros/sync_copies.sh.
add_library(hipnuc_c STATIC
  hipnuc/hipnuc_dec.c hipnuc/nmea_dec.c hipnuc/hipnuc_sample.c
  hipnuc/hipnuc_serial.c hipnuc/serial_io.c)
target_include_directories(hipnuc_c PUBLIC hipnuc)
set_target_properties(hipnuc_c PROPERTIES
  C_STANDARD 99 C_STANDARD_REQUIRED YES POSITION_INDEPENDENT_CODE ON)

add_library(hipnuc_imu_component SHARED src/driver.cpp src/imu_node.cpp)
target_compile_features(hipnuc_imu_component PRIVATE cxx_std_17)
target_link_libraries(hipnuc_imu_component PRIVATE hipnuc_c "${hipnuc_imu_interfaces}"
  rclcpp::rclcpp rclcpp_components::component
  ${rcl_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS})
rclcpp_components_register_node(hipnuc_imu_component
  PLUGIN "hipnuc_imu::ImuNode" EXECUTABLE hipnuc_imu_node)

install(TARGETS hipnuc_imu_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
