cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rodret_unlock C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(rodret_unlock
	rodret.c
)

target_link_libraries(rodret_unlock pico_stdlib)

pico_enable_stdio_usb(rodret_unlock 1)
pico_enable_stdio_uart(rodret_unlock 1)

pico_add_extra_outputs(rodret_unlock)

