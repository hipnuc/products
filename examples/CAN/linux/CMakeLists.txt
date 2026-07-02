cmake_minimum_required(VERSION 3.10)

project(canhost C)

set(CMAKE_C_STANDARD 99)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers)

set(SOURCES
    main.c
    can_interface.c
    commands.c
    log.c
    utils.c
    config.c
    commands/cmd_list.c
    commands/cmd_probe.c
    commands/cmd_read.c
    commands/cmd_record.c
    commands/cmd_sync.c
    commands/cmd_reg.c
    commands/cmd_action.c
    commands/cmd_update.c
    reg_seq.c
    j1939_reg_api.c
    help.c
    ../../../drivers/hipnuc_can_common.c
    ../../../drivers/hipnuc_j1939_parser.c
    ../../../drivers/canopen_parser.c
    ../../../drivers/hipnuc_can_update.c
)

add_executable(canhost ${SOURCES})

target_link_libraries(canhost m pthread)

target_compile_options(canhost PRIVATE -Wall -Wextra)

target_compile_definitions(canhost PRIVATE CANHOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE Debug)
