cmake_minimum_required(VERSION 3.16)
project(hipnuc_sdk C)

set(hipnuc_standalone OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(hipnuc_standalone ON)
endif()
set(hipnuc_desktop OFF)
if(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(hipnuc_desktop ${hipnuc_standalone})
endif()

option(HIPNUC_BUILD_SERIAL "Build the Windows/Linux serial interface" ${hipnuc_desktop})
option(HIPNUC_BUILD_EXAMPLES "Build the desktop integration examples" ${hipnuc_desktop})
option(HIPNUC_BUILD_TESTS "Build host regression tests (requires C++)" OFF)

# A parent project links only the targets it needs. Optional modules do not
# become part of its default build merely because they are available.
add_subdirectory(hipnuc EXCLUDE_FROM_ALL)
if(HIPNUC_BUILD_SERIAL OR HIPNUC_BUILD_EXAMPLES)
    add_subdirectory(serial)
endif()
if(HIPNUC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
if(HIPNUC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
