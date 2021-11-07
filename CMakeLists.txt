set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/TeensyToolchain.cmake")

project(freedv-dongle)
cmake_minimum_required(VERSION 3.21)

set(TEENSY_CORES_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cores)
set(TEENSY_USB_MODE SERIAL)

add_subdirectory(teensy-lib)
add_subdirectory(src)