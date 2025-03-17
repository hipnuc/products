cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_lib_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hipnuc_lib
  CATKIN_DEPENDS roscpp std_msgs

)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# 在 hipnuc_lib_package 的 CMakeLists.txt 中
add_library(hipnuc_lib src/hipnuc_dec.c src/nmea_decode.c)

# 设置库的属性
set_target_properties(hipnuc_lib PROPERTIES
  PUBLIC_HEADER "include/${PROJECT_NAME}/hipnuc_dec.h;include/${PROJECT_NAME}/nmea_decode.h"

)

target_include_directories(hipnuc_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接依赖库
target_link_libraries(hipnuc_lib
  ${catkin_LIBRARIES}
)




#############
## Install ##
#############
# 安装规则
install(TARGETS hipnuc_lib
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############


