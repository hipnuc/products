cmake_minimum_required(VERSION 3.10)
project(hipnuc_can_parser)

set(CMAKE_C_STANDARD 99)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../drivers)

add_executable(hipnuc_can_parser 
    main.c 
    ../../drivers/hipnuc_can_parser.c
)

target_link_libraries(hipnuc_can_parser m)
