project(EtherCatUserExample C)
cmake_minimum_required(VERSION 3.2)

find_package(EtherCAT REQUIRED)

add_executable(userexample
    main.c
    hi15.c
)

target_link_libraries(userexample PUBLIC EtherLab::ethercat)
