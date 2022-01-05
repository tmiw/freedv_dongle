set(TEENSY_CORE_DIR "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/cores/teensy4")
set(MCU IMXRT1062)
set(MCU_LD "${TEENSY_CORE_DIR}/imxrt1062_t41.ld")
set(MCU_DEF ARDUINO_TEENSY41)
set(TEENSY_OPTIONS "-DF_CPU=600000000 -DUSB_DUAL_SERIAL -DLAYOUT_US_ENGLISH -D__${MCU}__ -DARDUINO_TEENSY41 -DARDUINO=10813 -DTEENSYDUINO=154 -D${MCU_DEF}")
set(TEENSY_CPUOPTIONS "-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb")
set(TEENSY_LDFLAGS "-Os -Wl,--gc-sections,--relax ${TEENSY_CPUOPTIONS} -T${MCU_LD} -lm -lstdc++")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
set(CMAKE_C_COMPILER ${ARM_GCC_BIN}arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_GCC_BIN}arm-none-eabi-c++)
set(CMAKE_LINKER ${ARM_GCC_BIN}arm-none-eabi-ld)
set(CMAKE_ASM ${ARM_GCC_BIN}arm-none-eabi-as)
set(CMAKE_SIZE ${ARM_GCC_BIN}arm-none-eabi-size)
set(CMAKE_OBJCOPY ${ARM_GCC_BIN}arm-none-eabi-objcopy)
            
set(CMAKE_C_FLAGS_INIT "-Wall -g -O2 ${TEENSY_CPUOPTIONS} -MMD ${TEENSY_OPTIONS} -I${TEENSY_CORE_DIR} -ffunction-sections -fdata-sections -fsingle-precision-constant" CACHE STRING "Required compiler init flags")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -fno-exceptions -fno-non-call-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables -felide-constructors -fno-rtti -std=gnu++17 -Wno-error=narrowing -fpermissive" CACHE STRING "Required compiler init flags")
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
## https://stackoverflow.com/questions/10599038/can-i-skip-cmake-compiler-tests-or-avoid-error-unrecognized-option-rdynamic
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")