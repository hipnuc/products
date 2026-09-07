# canhost: SocketCAN command line tool (Linux).
cmake_minimum_required(VERSION 3.10)
project(canhost C)

if(NOT TARGET hipnuc_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../hipnuc hipnuc)
endif()
if(NOT TARGET hipnuc_tool_common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common)
endif()

add_executable(canhost
    main.c
    commands.c
    can_interface.c
    config.c
    help.c
    utils.c
    reg_seq.c
    j1939_reg_api.c
    commands/cmd_list.c
    commands/cmd_probe.c
    commands/cmd_read.c
    commands/cmd_record.c
    commands/cmd_sync.c
    commands/cmd_reg.c
    commands/cmd_action.c
    commands/cmd_update.c
)
target_include_directories(canhost PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(canhost PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
target_compile_options(canhost PRIVATE -Wall -Wextra)
target_link_libraries(canhost PRIVATE hipnuc_json hipnuc_j1939 hipnuc_update hipnuc_tool_common m)
