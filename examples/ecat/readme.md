# IGH例程

## 概览

本工程在 Linux 上提供读取HI15系列产品的功能与示例。`userexample` 用于读取 HI15系列产品的PDO数据。

### 支持的硬件

- 超核电子 HI15系列 产品

### 测试环境

- 操作系统：Ubuntu 20.04 LTS
- 处理器架构：x86_64
- 实时内核：PREEMPT-RT     内核版本：5.15.177-rt83（经过 PREEMPT-RT 补丁编译的实时内核）
- IGH主站
- 必须安装以下依赖：
  - `build-essential`（用于编译工具）
  - `libncurses5-dev`、`bison`、`flex`（内核配置依赖）
  - `libssl-dev`、`libelf-dev`（内核编译依赖）
  - `rt-tests`（实时性能测试工具）
  - `zstd`（压缩工具，生成 initrd 镜像时需要）
  - `cmake`（用于构建工程）

注意事项：

1. PREEMPT-RT内核安装的具体流程见此目录下的”PREEMPT-RT内核和补丁.doc”文件
2. IGH搭建流程见此目录下的”IGH搭建流程.doc”文件

## 文件说明

| 文件/目录        | 说明                                                      |
| ---------------- | --------------------------------------------------------- |
| `main.c`         | 主程序入口，配置主站的DC模式和实时显示从站PDO数据         |
| `hi15.{h,c}`     | 从站相关代码：对象字典映射，从站初始化，读取PDO数据等操作 |
| `CMakeLists.txt` | CMake 构建配置文件                                        |



## 构建说明

```
mkdir -p build
cd build
cmake ..
make
```

生成可执行文件`userexample`。  