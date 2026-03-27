cmake_minimum_required(VERSION 3.13)
project(output_encoder_bridge C CXX ASM)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Set PICO_SDK_PATH before configuring this project")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

pico_sdk_init()

add_executable(output_encoder_bridge
    src/main.c
    src/bridge_protocol.c
    src/mt6835.c
)

target_include_directories(output_encoder_bridge PRIVATE src)

target_link_libraries(output_encoder_bridge
    pico_stdlib
    hardware_spi
)

pico_enable_stdio_usb(output_encoder_bridge 1)
pico_enable_stdio_uart(output_encoder_bridge 0)

pico_add_extra_outputs(output_encoder_bridge)
