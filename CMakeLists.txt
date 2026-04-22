# Axon Multiprotocol USB Bridge Firmware
# Target: RoboCore Axon RP2350 Board

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform type")

include(pico_sdk_import.cmake)

project(axon_firmware C CXX ASM)

pico_sdk_init()

# Source files
add_executable(axon_firmware
    main.c
    usb_descriptors.c
    src/led_dual_core.c
    src/neopixel.c
    src/usb_handlers.c
    src/bus/rs485.c
    src/bus/feetech.c
    src/bus/dynamixel.c
    src/bus/can.c
    src/bus/uart_bridge.c
)

pico_set_program_name(axon_firmware "Axon Multiprotocol Bridge")
pico_set_program_version(axon_firmware "2.0")

# Optional features - uncomment to enable
target_compile_definitions(axon_firmware PRIVATE ENABLE_UART2)

# UART for debug output (USB is the composite device)
pico_enable_stdio_uart(axon_firmware 1)
pico_enable_stdio_usb(axon_firmware 0)

# Link libraries
target_link_libraries(axon_firmware
    pico_stdlib
    pico_unique_id
    pico_multicore
    tinyusb_device
    tinyusb_board
    hardware_i2c
    hardware_spi
    hardware_uart
    hardware_pio
    hardware_clocks
    hardware_pwm
)

# Include paths
target_include_directories(axon_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/bus
    ${CMAKE_CURRENT_LIST_DIR}/src/protocol
)

pico_add_extra_outputs(axon_firmware)

# Generate PIO headers
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_tx.pio)
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_rx.pio)
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)
