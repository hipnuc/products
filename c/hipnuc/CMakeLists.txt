# HiPNUC C core. Consume with add_subdirectory(<path>/c/hipnuc hipnuc) and
# link the target you need; or simply copy the files listed in ../README.md.
# Options are set per target, never globally, so a parent project's flags are
# left alone. No install() rules: this is a source library.

cmake_minimum_required(VERSION 3.10)
project(hipnuc_core C)

function(hipnuc_configure target)
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(${target} PUBLIC c_std_99)
    set_target_properties(${target} PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES C_EXTENSIONS OFF)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra)
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    endif()
endfunction()

# Common SI sample helpers. CAN and JSON need no serial decoder.
add_library(hipnuc_sample STATIC hipnuc_sample.c)
hipnuc_configure(hipnuc_sample)

# Serial decoding: binary frames and NMEA. No stdio, no libm.
add_library(hipnuc_core STATIC hipnuc_dec.c nmea_dec.c)
hipnuc_configure(hipnuc_core)
target_link_libraries(hipnuc_core PUBLIC hipnuc_sample)

# JSON formatting of samples (stdio).
add_library(hipnuc_json STATIC hipnuc_json.c)
hipnuc_configure(hipnuc_json)
target_link_libraries(hipnuc_json PUBLIC hipnuc_sample)

# CAN: J1939 measurement PGNs and CANFD83.
add_library(hipnuc_j1939 STATIC hipnuc_j1939.c)
hipnuc_configure(hipnuc_j1939)
target_link_libraries(hipnuc_j1939 PUBLIC hipnuc_sample)
