cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(FILES Ins_0x81_package.msg Imu_0x92_package.msg  Imu_0x91_package.msg hipnuc_imu_msg.msg)

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
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)

 add_executable(serial_hipnuc_msg src/serial_hipnuc_msg.cpp src/hipnuc_dec.c)
 target_link_libraries(serial_hipnuc_msg ${catkin_LIBRARIES})
 add_dependencies(serial_hipnuc_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(serial_hipnuc_msg ${PROJECT_NAME}_generate_messages_cpp)

 add_executable(sub_hipnuc_imu_msg src/sub_hipnuc_imu_msg.cpp)
 target_link_libraries(sub_hipnuc_imu_msg ${catkin_LIBRARIES})
 add_dependencies(sub_hipnuc_imu_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(sub_hipnuc_imu_msg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(serial_imu src/serial_imu.cpp src/hipnuc_dec.c)
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

