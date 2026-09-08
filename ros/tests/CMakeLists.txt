cmake_minimum_required(VERSION 3.16)
project(hipnuc_ros_tests CXX)
enable_testing()
add_executable(hipnuc_ros_conversion_test test_conversion.cpp)
target_include_directories(hipnuc_ros_conversion_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../common"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../c/hipnuc")
set_target_properties(hipnuc_ros_conversion_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
if(MSVC)
  target_compile_options(hipnuc_ros_conversion_test PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(hipnuc_ros_conversion_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME hipnuc_ros_conversion COMMAND hipnuc_ros_conversion_test)
