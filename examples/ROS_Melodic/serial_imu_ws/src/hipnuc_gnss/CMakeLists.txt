cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_gnss)


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


add_executable(serial_gnss src/serial_gnss.cpp src/hipnuc_dec.c src/nmea_decode.c)
target_link_libraries(serial_gnss ${catkin_LIBRARIES}) 

#add_executable(pub_path src/pub_path.cpp)
#target_link_libraries(pub_path ${catkin_LIBRARIES})

 add_executable(sub_gnss src/sub_gnss.cpp)
 add_dependencies(sub_gnss ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                           ${catkin_EXPORTED_TARGETS} 
                           ${PROJECT_NAME}_generate_messages_cpp)
 target_link_libraries(sub_gnss ${catkin_LIBRARIES})

 add_executable(sub_imu src/sub_imu.cpp)
  add_dependencies(sub_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                           ${catkin_EXPORTED_TARGETS} 
                           ${PROJECT_NAME}_generate_messages_cpp)
 target_link_libraries(sub_imu ${catkin_LIBRARIES})


#############
## Install ##
#############





#############
## Testing ##
#############

