# HiPNUC C core. Consume with add_subdirectory(<path>/c/hipnuc hipnuc) and
# link the target hipnuc, or copy the files listed in ../README.md. It is a
# static library, so a program links only the parts it calls. Options are set
# on the target, never globally. No install() rules: this is a source library.

cmake_minimum_required(VERSION 3.10)
project(hipnuc_core C)

add_library(hipnuc STATIC
    hipnuc_data.c
    hipnuc_decoder.c
    hipnuc_nmea.c
    hipnuc_can.c
    hipnuc_i2c.c)
target_include_directories(hipnuc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(hipnuc PUBLIC c_std_99)
set_target_properties(hipnuc PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES C_EXTENSIONS OFF)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(hipnuc PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(hipnuc PRIVATE /W4)
endif()
