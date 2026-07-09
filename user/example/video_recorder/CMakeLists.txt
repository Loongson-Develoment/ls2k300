cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(example_video_recorder C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
set(OpenCV_DIR "/home/ch2/opencv-4.10.0/install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

option(RECORD_WITH_TARGET "Record clips that contain the target object" ON)

set(LS2K0300_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(LS2K0300_LIB_INC_DIR "${LS2K0300_ROOT_DIR}/library/inc")
set(LS2K0300_LIB_SRC_DIR "${LS2K0300_ROOT_DIR}/library/src")
set(LS2K0300_DRV_INC_DIR "${LS2K0300_ROOT_DIR}/Driver/inc")
set(LS2K0300_DRV_SRC_DIR "${LS2K0300_ROOT_DIR}/Driver/src")

add_executable(example_video_recorder
    "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
    "${LS2K0300_DRV_SRC_DIR}/ICST7735.c"
    "${LS2K0300_LIB_SRC_DIR}/LS2K0300_MAP.c"
    "${LS2K0300_LIB_SRC_DIR}/LS2K0300_GPIO.c"
    "${LS2K0300_LIB_SRC_DIR}/LS2K0300_SPI.c"
)

if(RECORD_WITH_TARGET)
    target_compile_definitions(example_video_recorder PRIVATE RECORD_WITH_TARGET=1)
    set_target_properties(example_video_recorder PROPERTIES OUTPUT_NAME "example_video_recorder_with_target")
else()
    target_compile_definitions(example_video_recorder PRIVATE RECORD_WITH_TARGET=0)
    set_target_properties(example_video_recorder PROPERTIES OUTPUT_NAME "example_video_recorder_without_target")
endif()

target_include_directories(example_video_recorder PRIVATE
    "${LS2K0300_LIB_INC_DIR}"
    "${LS2K0300_DRV_INC_DIR}"
    "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(example_video_recorder PRIVATE Threads::Threads ${OpenCV_LIBS})
