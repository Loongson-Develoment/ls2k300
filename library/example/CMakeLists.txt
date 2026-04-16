cmake_minimum_required(VERSION 3.10)

project(ls2k0300_examples C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(LS2K0300_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(LS2K0300_INC_DIR "${LS2K0300_ROOT_DIR}/inc")
set(LS2K0300_SRC_DIR "${LS2K0300_ROOT_DIR}/src")

file(GLOB LS2K0300_DRIVER_SOURCES
    "${LS2K0300_SRC_DIR}/*.c"
)

add_executable(ls2k0300_examples
    "${CMAKE_CURRENT_LIST_DIR}/ls2k0300_library_examples.c"
    ${LS2K0300_DRIVER_SOURCES}
)

target_compile_definitions(ls2k0300_examples PRIVATE LS2K0300_EXAMPLE_RUN_ALL)
target_include_directories(ls2k0300_examples PRIVATE "${LS2K0300_INC_DIR}")
target_link_libraries(ls2k0300_examples PRIVATE Threads::Threads)
