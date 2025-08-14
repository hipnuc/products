# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Set the project name and language
project(hihost C)

# Set C standard to C99
set(CMAKE_C_STANDARD 99)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/fw_downloader)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../drivers)

# Define source files
set(SOURCES
    main.c
    serial_port.c
    commands.c
    log.c
    fw_downloader/hex2bin.c
    fw_downloader/kboot.c
    ../drivers/example_data.c
    ../drivers/hipnuc_dec.c
    ../drivers/nmea_dec.c
)

# Create executable
add_executable(hihost ${SOURCES})

# Link required libraries (if any)
target_link_libraries(hihost m)  # Link math library
