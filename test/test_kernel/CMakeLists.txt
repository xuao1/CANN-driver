cmake_minimum_required(VERSION 3.16.0)
project(Ascend_C)

# 1. 核心运行环境配置
set(SOC_VERSION "Ascend910B4" CACHE STRING "system on chip type")
set(ASCEND_CANN_PACKAGE_PATH "/home/HwHiAiUser/Ascend/ascend-toolkit/latest" CACHE PATH "ASCEND CANN package path")
set(RUN_MODE "npu" CACHE STRING "run mode: npu")

# 2. 补回 Build Type，防止底层 bash 脚本缺失参数报错！
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type Release/Debug (default Debug)" FORCE)

# 3. 引入官方编译脚本
include(${ASCEND_CANN_PACKAGE_PATH}/compiler/tikcpp/ascendc_kernel_cmake/ascendc.cmake)

# 4. 编译 Kernel
ascendc_library(kernels STATIC hello_world.cpp)

# 5. 编译 Host 主程序并链接
add_executable(main main.cpp)
target_link_libraries(main PRIVATE kernels)