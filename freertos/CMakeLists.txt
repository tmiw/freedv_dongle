set(FREERTOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/freertos")

set(FREERTOS_C_SOURCES
    ${FREERTOS_DIR}/src/portable/port.c
    ${FREERTOS_DIR}/src/croutine.c
    ${FREERTOS_DIR}/src/event_groups.c
    ${FREERTOS_DIR}/src/list.c
    ${FREERTOS_DIR}/src/queue.c
    ${FREERTOS_DIR}/src/stream_buffer.c
    ${FREERTOS_DIR}/src/tasks.c
    ${FREERTOS_DIR}/src/timers.c
)

set(FREERTOS_CXX_SOURCES
    ${FREERTOS_DIR}/lib/cpp/src/condition_variable.cpp
    ${FREERTOS_DIR}/lib/cpp/src/freertos_time.cpp
    ${FREERTOS_DIR}/lib/cpp/src/gthr_key.cpp
    ${FREERTOS_DIR}/lib/cpp/src/thread.cpp
    ${FREERTOS_DIR}/lib/gcc/condition_variable.cc
    ${FREERTOS_DIR}/lib/gcc/future.cc
    ${FREERTOS_DIR}/lib/gcc/mutex.cc
    ${FREERTOS_DIR}/src/portable/event_responder_support.cpp
    ${FREERTOS_DIR}/src/portable/teensy_4.cpp
    ${FREERTOS_DIR}/src/portable/teensy_common.cpp
)

add_library(teensy4_freertos STATIC ${FREERTOS_CXX_SOURCES} ${FREERTOS_C_SOURCES})

target_include_directories(teensy4_freertos PUBLIC 
    ${FREERTOS_DIR}/lib/cpp/src/bits
    ${FREERTOS_DIR}/lib/cpp/src
    ${FREERTOS_DIR}/src
)