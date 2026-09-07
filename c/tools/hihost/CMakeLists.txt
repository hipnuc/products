# hihost: serial command line tool (Linux, termios).
cmake_minimum_required(VERSION 3.10)
project(hihost C)

if(NOT TARGET hipnuc_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../hipnuc hipnuc)
endif()
if(NOT TARGET hipnuc_tool_common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common)
endif()

add_executable(hihost
    main.c
    commands.c
    serial_port.c
    hihost_config.c
    commands/cmd_list.c
    commands/cmd_probe.c
    commands/cmd_read.c
    commands/cmd_write.c
    commands/cmd_update.c
)
target_include_directories(hihost PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(hihost PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
target_compile_options(hihost PRIVATE -Wall -Wextra)
target_link_libraries(hihost PRIVATE hipnuc_json hipnuc_update hipnuc_tool_common m)
