cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(example_EMM_V5 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(LS2K0300_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(LS2K0300_LIB_INC_DIR "${LS2K0300_ROOT_DIR}/library/inc")
set(LS2K0300_LIB_SRC_DIR "${LS2K0300_ROOT_DIR}/library/src")
set(LS2K0300_DRV_INC_DIR "${LS2K0300_ROOT_DIR}/Driver/inc")
set(LS2K0300_DRV_SRC_DIR "${LS2K0300_ROOT_DIR}/Driver/src")

add_executable(example_EMM_V5
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${LS2K0300_DRV_SRC_DIR}/EMM_V5.c"
    "${LS2K0300_LIB_SRC_DIR}/LS2K0300_UART.c"
)

target_include_directories(example_EMM_V5 PRIVATE
    "${LS2K0300_LIB_INC_DIR}"
    "${LS2K0300_DRV_INC_DIR}"
    "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include"
)

target_link_libraries(example_EMM_V5 PRIVATE Threads::Threads)
