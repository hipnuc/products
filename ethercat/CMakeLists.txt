cmake_minimum_required(VERSION 3.16)
project(hi15_ethercat C)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "This HI15 example requires Linux and IgH EtherCAT Master 1.6.")
endif()
find_package(EtherCAT CONFIG QUIET)
if(NOT TARGET EtherLab::ethercat)
    message(FATAL_ERROR
        "Install the IgH EtherCAT Master 1.6 development library. "
        "For a custom installation, set -DCMAKE_PREFIX_PATH=<IgH prefix>.")
endif()

# IgH supplies a config target but no CMake package version file.
include(CheckCSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES EtherLab::ethercat)
check_c_source_compiles("#include <ecrt.h>
#if ECRT_VERSION_MAGIC < ECRT_VERSION(1, 6)
#error IgH 1.6 required
#endif
int main(void) { return 0; }" HI15_HAVE_IGH_16)
unset(CMAKE_REQUIRED_LIBRARIES)
if(NOT HI15_HAVE_IGH_16)
    message(FATAL_ERROR "The IgH headers and library must be version 1.6 or newer and usable by this compiler.")
endif()

add_executable(userexample main.c hi15.c)
target_compile_features(userexample PRIVATE c_std_99)
target_compile_definitions(userexample PRIVATE _POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
target_compile_options(userexample PRIVATE -Wall -Wextra)
target_link_libraries(userexample PRIVATE EtherLab::ethercat)
