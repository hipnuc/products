cmake_minimum_required(VERSION 3.16)
project(hipnuc_tools C)

if(CANHOST_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(../hipnuc hipnuc)
add_subdirectory(../serial serial)
add_subdirectory(common)
add_subdirectory(serial_update)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_subdirectory(canhost)
endif()
