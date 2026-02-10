cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(robocore_axon C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ──────────────────────────────────────────────
#  External Libraries
# ──────────────────────────────────────────────
include(FetchContent)

# BNO055 IMU driver
FetchContent_Declare(
    pico_bno055
    GIT_REPOSITORY https://github.com/alpertng02/pico-bno055.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(pico_bno055)

# Pico-ROS
FetchContent_Declare(picoros
    GIT_REPOSITORY https://github.com/Pico-ROS/Pico-ROS-software.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(picoros)

# ──────────────────────────────────────────────
#  RoboCore Axon Library (lib/)
# ──────────────────────────────────────────────
add_library(axon_lib STATIC
    lib/board.c
    lib/usb_descriptors.c
    lib/lidar_bridge.c
    # Transport
    lib/transport/uart_hw.c
    lib/transport/uart_pio.c
    lib/transport/rs485.c
    # Motor protocols
    lib/protocol/feetech.c
    lib/protocol/ddsm210.c
)

target_include_directories(axon_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(axon_lib PUBLIC
    pico_stdlib
    pico_multicore
    hardware_uart
    hardware_i2c
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_adc
    tinyusb_device
    tinyusb_board
)

# PIO programs
pico_generate_pio_header(axon_lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/pio/uart_tx.pio)
pico_generate_pio_header(axon_lib ${CMAKE_CURRENT_SOURCE_DIR}/lib/pio/uart_rx.pio)
target_include_directories(axon_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ──────────────────────────────────────────────
#  User Sketch (sketch/)
# ──────────────────────────────────────────────
add_executable(firmware
    sketch/main.c
    sketch/motors.c
    sketch/ros.c
)

target_include_directories(firmware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sketch
    ${picoros_SOURCE_DIR}/examples  # For example_types.h
)

target_link_libraries(firmware
    axon_lib
    pico_bno055
    picoros
)

pico_enable_stdio_usb(firmware 0)
pico_enable_stdio_uart(firmware 0)
pico_add_extra_outputs(firmware)

target_compile_definitions(firmware PRIVATE
    PICO_RP2350=1
    CFG_TUSB_MCU=OPT_MCU_RP2040
)

# ──────────────────────────────────────────────
#  Build info
# ──────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Target: RP2350 (Pico 2)")
