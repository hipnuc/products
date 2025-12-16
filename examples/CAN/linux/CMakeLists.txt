# 指定CMake最低版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称和语言
project(canhost C)

# 设置C标准为C99
set(CMAKE_C_STANDARD 99)

# 添加包含目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers)

# 定义源文件
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
    j1939_reg_api.c
    help.c
    ../../../drivers/hipnuc_can_common.c
    ../../../drivers/hipnuc_j1939_parser.c
    ../../../drivers/canopen_parser.c
)

# 创建可执行文件
add_executable(canhost ${SOURCES})

# 链接数学库
target_link_libraries(canhost m pthread)

# 设置编译选项
target_compile_options(canhost PRIVATE -Wall -Wextra)

# Inject source dir macro for config search
target_compile_definitions(canhost PRIVATE CANHOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 设置调试信息
set(CMAKE_BUILD_TYPE Debug)
