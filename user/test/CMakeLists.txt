cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++")

project(ls2k0300)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
set(OpenCV_DIR "/home/ch2/opencv-4.10.0/install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
set(ncnn_DIR "/home/ch2/ncnn/build-loongarch64/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)

include_directories(${REPO_ROOT}/user/inc)
include_directories(${REPO_ROOT}/library/inc)
include_directories(${REPO_ROOT}/Driver/inc)
include_directories(/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/include)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SRC
    "${REPO_ROOT}/user/src/*.c"
    "${REPO_ROOT}/user/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${REPO_ROOT}/Driver/src/*.c"
    "${REPO_ROOT}/Driver/src/*.cpp"
    "${REPO_ROOT}/library/src/*.c"
    "${REPO_ROOT}/library/src/*.cpp")

set(INJECTION_CURRENT_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/injection_current_test.c")
set(ROTATION_CONTROL_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/rotation_control_test.c")
set(ARM_X_SPEED_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/arm_x_speed_test.cpp")
set(ARM_Z_SPEED_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/arm_z_speed_test.cpp")
set(ARM_MOTOR23_MANUAL_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/arm_motor23_manual_test.cpp")
set(ARM_SINE_VIDEO_TEST_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/arm_sine_video_test.cpp")
list(REMOVE_ITEM SRC
    ${INJECTION_CURRENT_TEST_SOURCE}
    ${ROTATION_CONTROL_TEST_SOURCE}
    ${ARM_X_SPEED_TEST_SOURCE}
    ${ARM_Z_SPEED_TEST_SOURCE}
    ${ARM_MOTOR23_MANUAL_TEST_SOURCE}
    ${ARM_SINE_VIDEO_TEST_SOURCE})
set_source_files_properties(
    ${INJECTION_CURRENT_TEST_SOURCE}
    ${ROTATION_CONTROL_TEST_SOURCE}
    PROPERTIES LANGUAGE CXX)

file(GLOB INJECTION_SUPPORT_SRC
    "${REPO_ROOT}/user/src/*.c"
    "${REPO_ROOT}/user/src/*.cpp"
    "${REPO_ROOT}/Driver/src/*.c"
    "${REPO_ROOT}/Driver/src/*.cpp"
    "${REPO_ROOT}/library/src/*.c"
    "${REPO_ROOT}/library/src/*.cpp")

add_executable(test_motor ${SRC})
target_compile_options(test_motor PRIVATE -O3)
target_link_libraries(test_motor PUBLIC Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(injection_current_test
    ${INJECTION_CURRENT_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(injection_current_test PRIVATE -O3)
target_link_libraries(injection_current_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(rotation_control_test
    ${ROTATION_CONTROL_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(rotation_control_test PRIVATE -O3)
target_link_libraries(rotation_control_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(arm_x_speed_test
    ${ARM_X_SPEED_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(arm_x_speed_test PRIVATE -O3)
target_link_libraries(arm_x_speed_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(arm_z_speed_test
    ${ARM_Z_SPEED_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(arm_z_speed_test PRIVATE -O3)
target_link_libraries(arm_z_speed_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(arm_motor23_manual_test
    ${ARM_MOTOR23_MANUAL_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(arm_motor23_manual_test PRIVATE -O3)
target_link_libraries(arm_motor23_manual_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)

add_executable(arm_sine_video_test
    ${ARM_SINE_VIDEO_TEST_SOURCE}
    ${INJECTION_SUPPORT_SRC})
target_compile_options(arm_sine_video_test PRIVATE -O3)
target_link_libraries(arm_sine_video_test PRIVATE Threads::Threads ${OpenCV_LIBS} ncnn)
