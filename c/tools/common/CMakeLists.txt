# Helpers shared by hihost and canhost: rxi logger, Intel HEX / binary
# firmware loader, INI reader.
cmake_minimum_required(VERSION 3.10)
project(hipnuc_tool_common C)

add_library(hipnuc_tool_common STATIC log.c hexfile.c ini.c)
target_include_directories(hipnuc_tool_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(hipnuc_tool_common PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
target_compile_options(hipnuc_tool_common PRIVATE -Wall -Wextra)
