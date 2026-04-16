cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(example_uart C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(LS2K0300_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(LS2K0300_INC_DIR "${LS2K0300_ROOT_DIR}/inc")
set(LS2K0300_SRC_DIR "${LS2K0300_ROOT_DIR}/src")
set(UART_SHIM_DIR "${CMAKE_CURRENT_BINARY_DIR}/uart_shim")

file(MAKE_DIRECTORY "${UART_SHIM_DIR}/bits")
file(MAKE_DIRECTORY "${UART_SHIM_DIR}/asm-generic")
file(WRITE "${UART_SHIM_DIR}/bits/termios-baud.h" "/* local shim */\n")
file(WRITE "${UART_SHIM_DIR}/asm-generic/termbits.h" "/* local shim */\n")

add_executable(example_uart
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${LS2K0300_SRC_DIR}/LS2K0300_UART.c"
)

target_include_directories(example_uart PRIVATE
    "${UART_SHIM_DIR}"
    "${LS2K0300_INC_DIR}"
    "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include"
)
target_compile_definitions(example_uart PRIVATE CRTSCTS=0)
target_link_libraries(example_uart PRIVATE Threads::Threads)
