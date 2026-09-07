# Linux command line tools for HiPNUC devices.
#
#   cmake -S c/tools -B build/tools && cmake --build build/tools
#   build/tools/hihost/hihost --help
#   build/tools/canhost/canhost --help
#
# Each tool directory can also be configured on its own, e.g.
# cmake -S c/tools/hihost -B build/hihost.
cmake_minimum_required(VERSION 3.10)
project(hipnuc_tools C)

add_subdirectory(../hipnuc hipnuc)
add_subdirectory(common)
add_subdirectory(hihost)
add_subdirectory(canhost)
