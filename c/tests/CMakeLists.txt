# Host tests for the HiPNUC C core.
#
#   cmake -S c/tests -B build/c-tests            (add -G "MinGW Makefiles" on Windows/MinGW)
#   cmake --build build/c-tests
#   ctest --test-dir build/c-tests --output-on-failure
#
# Besides the protocol tests this directory checks the three ways customers
# consume the core: linking the CMake targets from a parent project (this
# file), compiling a project that only copied the listed files, and including
# the headers from C++.

cmake_minimum_required(VERSION 3.16)
project(hipnuc_c_tests C CXX)

enable_testing()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(HIPNUC_CORE_DIR "${REPO_ROOT}/c/hipnuc")

# Parent-project consumption: the core is pulled in with add_subdirectory.
add_subdirectory(${HIPNUC_CORE_DIR} hipnuc)
add_subdirectory(${REPO_ROOT}/c/serial serial)

function(hipnuc_test name)
    add_executable(${name} ${ARGN})
    set_target_properties(${name} PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${name} PRIVATE -Wall -Wextra -Werror -pedantic)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

hipnuc_test(test_decoder test_decoder.c)
target_link_libraries(test_decoder PRIVATE hipnuc_json)

hipnuc_test(test_j1939 test_j1939.c)
target_link_libraries(test_j1939 PRIVATE hipnuc_j1939)

# Windows treats executables named *update* as installers; keep the name plain.
hipnuc_test(test_fwload test_update.c)
target_link_libraries(test_fwload PRIVATE hipnuc_update)

# Copy-file consumption: only the files from the README copy list, compiled as
# a standalone program with no CMake target and no other include paths.
set(COPY_DIR "${CMAKE_CURRENT_BINARY_DIR}/consume_copy")
file(MAKE_DIRECTORY ${COPY_DIR})
foreach(f hipnuc_dec.c hipnuc_dec.h nmea_dec.c nmea_dec.h hipnuc_sample.c hipnuc_sample.h)
    configure_file(${HIPNUC_CORE_DIR}/${f} ${COPY_DIR}/${f} COPYONLY)
endforeach()
add_executable(consume_copy_c consume_copy.c
    ${COPY_DIR}/hipnuc_dec.c ${COPY_DIR}/nmea_dec.c ${COPY_DIR}/hipnuc_sample.c)
target_include_directories(consume_copy_c PRIVATE ${COPY_DIR})
set_target_properties(consume_copy_c PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(consume_copy_c PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
add_test(NAME consume_copy_c COMMAND consume_copy_c)

# C++ consumption: every public header included from a C++ translation unit.
add_executable(consume_cpp consume_cpp.cpp)
target_link_libraries(consume_cpp PRIVATE hipnuc_json hipnuc_j1939 hipnuc_update)
set_target_properties(consume_cpp PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(consume_cpp PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME consume_cpp COMMAND consume_cpp)

function(hipnuc_copy_test name)
    set(directory "${CMAKE_CURRENT_BINARY_DIR}/copy_${name}")
    file(MAKE_DIRECTORY "${directory}")
    set(sources)
    foreach(file ${ARGN})
        configure_file(${HIPNUC_CORE_DIR}/${file} ${directory}/${file} COPYONLY)
        if(file MATCHES "\\.c$")
            list(APPEND sources "${directory}/${file}")
        endif()
    endforeach()
    hipnuc_test(consume_${name} consume_${name}.c ${sources})
    target_include_directories(consume_${name} PRIVATE "${directory}")
endfunction()
hipnuc_copy_test(sample hipnuc_sample.c hipnuc_sample.h)
hipnuc_copy_test(binary hipnuc_sample.c hipnuc_sample.h hipnuc_dec.c hipnuc_dec.h)
hipnuc_copy_test(nmea hipnuc_sample.c hipnuc_sample.h nmea_dec.c nmea_dec.h)
hipnuc_copy_test(can hipnuc_sample.c hipnuc_sample.h hipnuc_j1939.c hipnuc_j1939.h hipnuc_can_frame.h)

# Stream behavior uses a deterministic byte transport; port tests use Linux
# pseudoterminals and validate failure/cleanup on Windows without a device.
hipnuc_test(test_serial_stream test_serial_stream.c ../serial/hipnuc_serial.c)
target_include_directories(test_serial_stream PRIVATE ../serial)
target_link_libraries(test_serial_stream PRIVATE hipnuc_core)
hipnuc_test(test_serial_port test_serial_port.c)
target_link_libraries(test_serial_port PRIVATE hipnuc_serial)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(test_serial_port PRIVATE util)
endif()

# Exercise the actual updater entry point with deterministic I/O failures.
hipnuc_test(test_serial_cli test_serial_update_cli.c)
target_include_directories(test_serial_cli PRIVATE ../hipnuc ../serial ../tools/common)
