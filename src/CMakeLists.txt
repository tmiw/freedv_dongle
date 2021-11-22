include(ExternalProject)
include(FetchContent)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${TEENSY_LDFLAGS}")

# Compiles and links CMSIS into the application. This is to bring in
# ARM-optimized math functions for Codec2.
# ==================================================================
message("Setting up CMSIS...")
FetchContent_Declare(CMSIS
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_SHALLOW ON
    GIT_PROGRESS ON
    GIT_TAG 5.8.0
)
FetchContent_GetProperties(cmsis)
if(NOT ${cmsis_POPULATED})
  FetchContent_Populate(CMSIS)
endif()

set(ROOT ${cmsis_SOURCE_DIR})
set(DSP ${cmsis_SOURCE_DIR}/CMSIS/DSP)
set(ARM_CPU "cortex-m7")
include(${cmsis_SOURCE_DIR}/CMSIS/DSP/Toolchain/GCC.cmake)
list(APPEND CMAKE_MODULE_PATH ${cmsis_SOURCE_DIR}/CMSIS/DSP ${cmsis_SOURCE_DIR}/CMSIS/DSP/Source)

include_directories(AFTER ${cmsis_SOURCE_DIR}/CMSIS/Core/Include)
include_directories(AFTER ${cmsis_SOURCE_DIR}/CMSIS/DSP/Include)
include_directories(AFTER ${cmsis_SOURCE_DIR}/CMSIS/DSP/ComputeLibrary/Include)
add_subdirectory(${cmsis_SOURCE_DIR}/CMSIS/DSP/Source ${cmsis_BINARY_DIR} EXCLUDE_FROM_ALL)

# Compiles and links Codec2 into the application.
# ==================================================================
message("Setting up Codec2...")

# This is needed because Codec2's C compiler checks otherwise fail on account of _exit not existing
# in the Pi Pico's standard library.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(MICROCONTROLLER_BUILD 1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mlittle-endian -ffunction-sections -fdata-sections -g -O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -ffunction-sections -fdata-sections")
#set(CMAKE_C_FLAGS_DEBUG "")

add_definitions(-DCORTEX_M7 -D__EMBEDDED__ -D__REAL__ -DFDV_ARM_MATH)
add_definitions(-DFREEDV_MODE_EN_DEFAULT=0 -DFREEDV_MODE_1600_EN=1 -DFREEDV_MODE_700D_EN=1 -DFREEDV_MODE_700E_EN=1 -DCODEC2_MODE_EN_DEFAULT=0 -DCODEC2_MODE_1300_EN=1 -DCODEC2_MODE_700C_EN=1)
                    
FetchContent_Declare(codec2
    GIT_REPOSITORY https://github.com/drowe67/codec2.git
    GIT_TAG origin/master
    GIT_SHALLOW ON
    GIT_PROGRESS ON
)
FetchContent_GetProperties(codec2)
if(NOT ${codec2_POPULATED})
    FetchContent_Populate(codec2)
endif()
set(CMAKE_REQUIRED_FLAGS "")

set(LPCNET OFF CACHE BOOL "")
add_subdirectory(${codec2_SOURCE_DIR} ${codec2_BINARY_DIR} EXCLUDE_FROM_ALL)

add_executable(freedv-dongle
    main.cpp
    ringbuf.c
    dongle_protocol.c
)

add_custom_command(TARGET freedv-dongle
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ARGS freedv-dongle
    COMMAND ${CMAKE_OBJCOPY} ARGS -O ihex freedv-dongle freedv-dongle.hex
)

target_link_libraries(freedv-dongle teensy4 codec2 CMSISDSP)