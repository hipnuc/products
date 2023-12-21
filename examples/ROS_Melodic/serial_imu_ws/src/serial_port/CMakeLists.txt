cmake_minimum_required(VERSION 3.0.2)
project(serial_port)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(FILES Imu_0x91_msg.msg Imu_data_package.msg)

generate_messages(DEPENDENCIES std_msgs)


################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_port
  CATKIN_DEPENDS roscpp serial rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)

add_executable(serial_imu_0x91 src/serial_imu_0x91.cpp src/ch_serial.c)
target_link_libraries(serial_imu_0x91 ${catkin_LIBRARIES})
add_dependencies(serial_imu_0x91 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serial_imu_0x91 ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sub_0x91 src/sub_0x91.cpp)
target_link_libraries(sub_0x91 ${catkin_LIBRARIES})
add_dependencies(sub_0x91 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sub_0x91 ${PROJECT_NAME}_generate_messages_cpp)

add_executable(serial_imu src/serial_imu.cpp src/ch_serial.c)
target_link_libraries(serial_imu ${catkin_LIBRARIES}) 

add_executable(sub_spec src/sub_spec.cpp)
add_dependencies(sub_spec ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                          ${catkin_EXPORTED_TARGETS} 
                          ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sub_spec ${catkin_LIBRARIES})


#############
## Install ##
#############





#############
## Testing ##
#############

