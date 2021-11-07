set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${TEENSY_LDFLAGS}")

add_executable(freedv-dongle
    main.cpp
)

add_custom_command(TARGET freedv-dongle
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ARGS freedv-dongle
    COMMAND ${CMAKE_OBJCOPY} ARGS -O ihex freedv-dongle freedv-dongle.hex
)

target_link_libraries(freedv-dongle teensy4)