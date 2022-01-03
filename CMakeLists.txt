set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/TeensyToolchain.cmake")

project(freedv-dongle)
cmake_minimum_required(VERSION 3.21)

add_compile_options(-fasynchronous-unwind-tables)
add_subdirectory(teensy-lib)
add_subdirectory(freertos)
add_subdirectory(src)