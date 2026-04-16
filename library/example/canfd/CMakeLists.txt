cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(example_canfd C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(LS2K0300_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(LS2K0300_INC_DIR "${LS2K0300_ROOT_DIR}/inc")
set(LS2K0300_SRC_DIR "${LS2K0300_ROOT_DIR}/src")

add_executable(example_canfd
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${LS2K0300_SRC_DIR}/LS2K0300_CANFD.c"
)

target_include_directories(example_canfd PRIVATE
    "${LS2K0300_INC_DIR}"
    "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include"
)
target_link_libraries(example_canfd PRIVATE Threads::Threads)
