cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(ls2k0300)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
set(OpenCV_DIR "/home/ch2/opencv-4.10.0/install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
set(ncnn_DIR "/home/ch2/ncnn/build-loongarch64/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/user/inc)
include_directories(${PROJECT_SOURCE_DIR}/library/inc)
include_directories(${PROJECT_SOURCE_DIR}/Driver/inc)
include_directories(/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB COMMON_SRC
    "${PROJECT_SOURCE_DIR}/user/src/*.c"
    "${PROJECT_SOURCE_DIR}/user/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/Driver/src/*.c"
    "${PROJECT_SOURCE_DIR}/Driver/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/library/src/*.c"
    "${PROJECT_SOURCE_DIR}/library/src/*.cpp")

add_executable(main1 ${COMMON_SRC} "${PROJECT_SOURCE_DIR}/user/main1.cpp")
target_compile_options(main1 PRIVATE -O3)
target_link_libraries(main1 PUBLIC Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(main2 ${COMMON_SRC} "${PROJECT_SOURCE_DIR}/user/main2.cpp")
target_compile_options(main2 PRIVATE -O3)
target_link_libraries(main2 PUBLIC Threads::Threads ${OpenCV_LIBS} ncnn)
