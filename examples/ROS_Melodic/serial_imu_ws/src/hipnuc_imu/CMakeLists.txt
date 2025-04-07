cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


#add_message_files(FILES Ins_0x81_package.msg Imu_0x92_package.msg  Imu_0x91_package.msg hipnuc_imu_msg.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


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
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)

 add_executable(sub_spec src/sub_spec.cpp)
 add_dependencies(sub_spec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_exported_targets})
 add_dependencies(sub_spec ${PROJECT_NAME}_generate_messages_cpp)
 target_link_libraries(sub_spec ${catkin_LIBRARIES})

 add_executable(hipnuc_imu_msg src/imu_node.cpp src/serial_port.cpp src/imu_pub.cpp)
 target_link_libraries(hipnuc_imu_msg ${catkin_LIBRARIES}  hipnuc_lib)
 add_dependencies(hipnuc_imu_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_exported_targets})
 add_dependencies(hipnuc_imu_msg ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############





#############
## Testing ##
#############

