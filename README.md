# RoboCore Axon Multiprotocol Bridge Firmware

Production firmware for the RoboCore Axon board providing multiple communication interfaces over USB.

## Overview

The Axon board is a versatile USB-to-multiprotocol bridge built around the RP2350 microcontroller with dedicated hardware for:
- **CAN Bus** (MCP2518FD + SN65HVD230 transceiver)
- **RS485** (isolated transceiver ADM2587E)
- **Multiple Serial Buses** (Unified motor bus for Feetech SCS/STS & Dynamixel)
- **UART Bridges** (2-3 general purpose UARTs)
- **I2C** (Qwiic/STEMMA QT compatible)
- **NeoPixel LEDs** (WS2812B activity indicators)

## Hardware Features

- **RP2350** microcontroller (dual Cortex-M33, 520KB RAM, 4MB Flash)
- **MCP2518FD** CAN controller with SPI interface
- **SN65HVD230** CAN transceiver
- **ADM2587E** isolated RS485 transceiver
- **USB-C** connector for host communication
- **NeoPixel Activity LEDs** (6 LEDs for visual status feedback)
- **40MHz** crystal oscillator for CAN timing accuracy

## Architecture Overview

### Dual-Core Design
- **Core 0**: Main USB communication, protocol handling, and serial bridges
- **Core 1**: Dedicated LED controller with real-time NeoPixel updates

### PIO State Machine Allocation

The RP2350 provides 3 PIO blocks × 4 state machines = 12 total PIO resources:

#### PIO0 Block (Serial Communication)
- **SM0**: Motor Bus TX (GPIO7) - Unified Feetech/Dynamixel transmission
- **SM1**: Motor Bus RX (GPIO8) - Unified Feetech/Dynamixel reception  
- **SM2**: *Available for expansion*
- **SM3**: *Available for expansion*

#### PIO1 Block (UART Bridges)
- **SM0**: UART0 TX (GPIO3) - General purpose UART transmission
- **SM1**: UART0 RX (GPIO4) - General purpose UART reception
- **SM2**: UART1 TX (GPIO5) - General purpose UART transmission
- **SM3**: UART1 RX (GPIO6) - General purpose UART reception

#### PIO2 Block (Core 1 - Visual & Optional)
- **SM0**: NeoPixel WS2812 (GPIO18) - LED strip control
- **SM1**: *UART2 TX (GPIO19) - if ENABLE_UART2*
- **SM2**: *UART2 RX (GPIO20) - if ENABLE_UART2*
- **SM3**: *Available for expansion*

**Resource Utilization**: 7/12 state machines used (9/12 if UART2 enabled)

### USB Descriptor Layout & Endpoints

```
Configuration 1
├── Interface 0: CAN Bus (gs_usb) - Vendor Class
│   ├── EP1 IN  (0x81): CAN RX frames to host
│   └── EP2 OUT (0x02): CAN TX frames from host
├── Interface 1: RS485 Control (CDC ACM)
│   └── EP3 IN  (0x83): CDC Control (notifications)
├── Interface 2: RS485 Data (CDC ACM)  
│   ├── EP4 IN  (0x84): RS485 RX data to host
│   └── EP4 OUT (0x04): RS485 TX data from host
├── Interface 3: Motor Control (CDC ACM) [Feetech/Dynamixel]
│   └── EP5 IN  (0x85): CDC Control (notifications)
├── Interface 4: Motor Data (CDC ACM) [Feetech/Dynamixel]
│   ├── EP6 IN  (0x86): Motor RX data to host  
│   └── EP6 OUT (0x06): Motor TX data from host
├── Interface 5: UART0 Control (CDC ACM)
│   └── EP7 IN  (0x87): CDC Control (notifications)
├── Interface 6: UART0 Data (CDC ACM)
│   ├── EP8 IN  (0x88): UART0 RX data to host
│   └── EP8 OUT (0x08): UART0 TX data from host  
├── Interface 7: UART1 Control (CDC ACM)
│   └── EP9 IN  (0x89): CDC Control (notifications)
├── Interface 8: UART1 Data (CDC ACM)
│   ├── EP10 IN (0x8A): UART1 RX data to host
│   └── EP10 OUT (0x0A): UART1 TX data from host
├── [Optional] Interface 9: UART2 Control (CDC ACM)
│   └── EP11 IN (0x8B): CDC Control (notifications)  
├── [Optional] Interface 10: UART2 Data (CDC ACM)
│   ├── EP12 IN (0x8C): UART2 RX data to host
│   └── EP12 OUT (0x0C): UART2 TX data from host
└── Interface 11: I2C (i2c-tiny-usb) - Vendor Class
    └── Control transfers only (EP0)
```

## Supported Protocols

### CAN Bus (gs_usb compatible)
- **Interface**: USB Vendor Class (Interface 0, VID:0x1209 PID:0xAC01)
- **Protocol**: gs_usb (Linux SocketCAN compatible)
- **Controller**: MCP2518FD over SPI1 @ 10MHz
- **Transceiver**: SN65HVD230 with GPIO control
- **Bitrates**: 125k, 250k, 500k, 1000k bps
- **Features**: Listen-only, loopback modes, error frame detection
- **Linux Device**: `/dev/can1` (via gs_usb kernel module)
- **Sample Point**: 87.5% for optimal bus timing

### Serial Interfaces (CDC ACM)

#### RS485 Bus
- **Interface**: CDC ACM (Interface 1-2)
- **Hardware**: ADM2587E isolated transceiver
- **Direction Control**: GPIO2 (DE pin) with automatic timing
- **Linux Device**: `/dev/ttyACM0` → `/dev/axon-BUS-PORT/rs485`

#### Unified Motor Bus (Feetech SCS/STS & Dynamixel)
- **Interface**: CDC ACM (Interface 3-4)
- **Hardware**: Half-duplex UART with TX enable control (GPIO16)
- **Protocols**: Both Feetech and Dynamixel protocols on shared bus
- **Direction Control**: Automatic TX/RX switching with precise timing
- **Linux Device**: `/dev/ttyACM1` → `/dev/axon-BUS-PORT/bus1`

#### UART Bridges
- **UART0**: Interface 5-6 (`/dev/ttyACM2` → `/dev/axon-BUS-PORT/uart0`)
- **UART1**: Interface 7-8 (`/dev/ttyACM3` → `/dev/axon-BUS-PORT/uart1`)
- **UART2**: Interface 9-10 (`/dev/ttyACM4` → `/dev/axon-BUS-PORT/uart2`) - *Optional*

**Multi-device Support**: When multiple Axon devices are connected, each gets unique symlinks based on USB bus/port: `/dev/axon-1-1/rs485`, `/dev/axon-1-2/rs485`, etc.

### I2C Interface (i2c-tiny-usb compatible)
- **Interface**: USB Vendor Class (Interface 11)
- **Protocol**: i2c-tiny-usb (Linux kernel driver)
- **Hardware**: RP2350 I2C1 peripheral (GPIO14/15)
- **Clock Speed**: 100kHz (standard mode), configurable to 400kHz
- **Linux Device**: `/dev/i2c-10` (via i2c-tiny-usb kernel module)
- **Connector**: Qwiic/STEMMA QT compatible (3.3V, SDA, SCL, GND)

### Activity LEDs (NeoPixel)
- **Hardware**: 6× WS2812B LEDs on single data line (GPIO18)
- **Controller**: Dedicated PIO state machine on Core 1
- **Indicators**:
  - LED 0: CAN Bus activity
  - LED 1: RS485 activity  
  - LED 2: Motor Bus activity
  - LED 3: I2C activity
  - LED 4: UART0/1 activity
  - LED 5: System status/heartbeat
- **Colors**: Customizable RGB with fade-out effects

## Pin Assignments

```
┌─────────────────────────────────────────────────────┐
│  GPIO Pin Assignments (RP2350)                     │
├─────────────────────────────────────────────────────┤
│  RS485 Bus (Hardware UART0 + Direction Control)    │
│    GPIO0:  RS485_TX                                 │
│    GPIO1:  RS485_RX                                 │ 
│    GPIO2:  RS485_DE (Direction Enable)              │
├─────────────────────────────────────────────────────┤
│  UART Bridges (PIO-based)                          │
│    GPIO3:  UART0_TX                                 │
│    GPIO4:  UART0_RX                                 │
│    GPIO5:  UART1_TX                                 │
│    GPIO6:  UART1_RX                                 │
│    GPIO19: UART2_TX (if ENABLE_UART2)              │
│    GPIO20: UART2_RX (if ENABLE_UART2)              │
├─────────────────────────────────────────────────────┤
│  Unified Motor Bus (PIO-based, Half-duplex)        │
│    GPIO7:  MOTOR_TX  (Feetech/Dynamixel)          │
│    GPIO8:  MOTOR_RX  (Feetech/Dynamixel)          │
│    GPIO16: MOTOR_TXEN (TX Enable/Direction)        │
├─────────────────────────────────────────────────────┤
│  CAN Bus (SPI + Interrupt)                         │
│    GPIO9:  CAN_INT  (MCP2518FD interrupt)          │
│    GPIO10: SPI_SCK  (SPI1 Clock)                   │
│    GPIO11: SPI_MOSI (SPI1 Master Out)              │
│    GPIO12: SPI_MISO (SPI1 Master In)               │
│    GPIO13: SPI_CS   (MCP2518FD Chip Select)        │
├─────────────────────────────────────────────────────┤
│  I2C Bus (Hardware I2C1)                           │
│    GPIO14: I2C_SDA  (Data)                         │
│    GPIO15: I2C_SCL  (Clock)                        │
├─────────────────────────────────────────────────────┤
│  Visual & Control                                   │
│    GPIO17: SNIFF_ENABLE (Bus monitor control)      │
│    GPIO18: NEOPIXELS (WS2812B data line)           │
│    GPIO21: GP21 (General purpose)                  │
│    GPIO22: GP22 (General purpose)                  │
└─────────────────────────────────────────────────────┘
```

## Quick Start

### Prerequisites
- Linux host with SocketCAN support
- can-utils package: `sudo apt install can-utils`
- DKMS modules installed (see Installation section)

### CAN Bus Setup
```bash
# Set bitrate and bring up interface
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up

# Monitor CAN traffic
candump can1

# Send test frames
cangen can1

# Advanced: Set listen-only mode
sudo ip link set can1 type can bitrate 500000 listen-only on
```

### Serial Interface Setup
```bash
# RS485 communication
# Single device
echo "test message" > /dev/axon-*/rs485
cat /dev/axon-*/rs485

# Multiple devices - specify exact device by USB bus/port
echo "test message" > /dev/axon-1-1/rs485

# Motor bus communication (Feetech SCS protocol example)  
echo -ne "\\xFF\\xFF\\x01\\x04\\x03\\x01\\x01\\xF1" > /dev/axon-*/bus1

# Dynamixel protocol example (ping servo ID 1)
echo -ne "\\xFF\\xFF\\x01\\x02\\x01\\xFB" > /dev/axon-*/bus1

# UART bridge passthrough
echo "sensor_command" > /dev/axon-*/uart0
cat /dev/axon-*/uart0
```

### I2C Interface Setup
```bash
# Scan I2C bus for connected devices
i2cdetect -y 10

# Read from I2C device (e.g., temperature sensor at 0x48)
i2cget -y 10 0x48 0x00

# Write to I2C device
i2cset -y 10 0x48 0x01 0x60

# Use i2c-tools for complex transactions
i2ctransfer -y 10 w2@0x48 0x01 0x60 r2
```

## Installation

### 1. Build Firmware
```bash
cd /home/cdr/axon/firmware
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. Flash Firmware
```bash
# Enter bootloader mode (hold BOOT button, press RESET)
# Flash firmware
cp axon_firmware.uf2 /media/RPI-RP2/
```

### 3. Install Kernel Modules
```bash
# Install gs_usb DKMS module
cd dkms/gs_usb-dkms
sudo ./install.sh

# Install i2c-robocore DKMS module  
cd ../robocore-i2c-dkms
sudo ./install.sh

# Install udev rules
sudo cp 99-axon-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

## Technical Details

### PIO Programs

#### UART TX Program (`uart_tx.pio`)
```
Purpose: High-performance serial transmission
Features: 8N1 format, LSB first, autopull with side-set
Timing:   8 cycles per bit (configurable baud rate)
Usage:    All UART and motor bus TX operations
```

#### UART RX Program (`uart_rx.pio`)
```
Purpose: Serial reception with error detection  
Features: Start bit detection, 8 data bits, stop bit validation
Timing:   Mid-bit sampling for noise immunity
Errors:   IRQ 4 signals framing errors
Usage:    All UART and motor bus RX operations
```

#### WS2812 Program (`ws2812.pio`)
```
Purpose: NeoPixel LED control
Features: 24-bit RGB color data, precise timing
Timing:   T0H=0.3µs, T0L=0.9µs, T1H=0.9µs, T1L=0.3µs
Clock:    ~13.6MHz for optimal WS2812B timing
Usage:    Activity LEDs and status indication
```

### CAN Controller Configuration
- **Controller**: MCP2518FD over SPI1 @ 10MHz
- **Crystal**: 40MHz for precise timing
- **Transceiver**: SN65HVD230 with GPIO standby control
- **FIFO**: 32 TX + 64 RX message buffers
- **Interrupts**: RX, TX, error, and bus state changes
- **Bit Timing**: Configurable TQ, Phase Seg1/2, SJW
- **Default Bitrate**: 500kbps with 87.5% sample point

### USB Implementation Details

#### Device Descriptor
- **Vendor ID**: 0x1209 (pid.codes)
- **Product ID**: 0xAC01 (assigned for Axon)
- **Device Class**: MISC (0xEF) for composite device
- **bcdDevice**: 0x0200 (version 2.0)

#### CDC ACM Configuration
- **Baud Rates**: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
- **Data Bits**: 7, 8 bits
- **Parity**: None, Odd, Even  
- **Stop Bits**: 1, 2 bits
- **Flow Control**: None, RTS/CTS (hardware flow control)
- **Buffer Size**: 64-byte USB packets

#### Vendor Class Endpoints
- **CAN (gs_usb)**: EP1 IN (RX), EP2 OUT (TX) - 64 byte packets
- **I2C (i2c-tiny-usb)**: Control transfers only (EP0)

## Development & Extension

### Adding New Protocols

#### 1. USB Interface Addition
```c
// In usb_descriptors.c
#define NEW_INTERFACE_NUM  12  // Next available interface

// Add interface descriptor
TUD_VENDOR_DESCRIPTOR(NEW_INTERFACE_NUM, 4, EPNUM_NEW_OUT, EPNUM_NEW_IN, 64),

// In usb_handlers.c
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, 
                                tusb_control_request_t const * request)
{
    if (request->wIndex == NEW_INTERFACE_NUM) {
        return handle_new_protocol(rhport, stage, request);
    }
    // ... existing handlers
}
```

#### 2. PIO Program Integration
```c
// Create new .pio file in src/pio/
// Add to CMakeLists.txt:
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/new_protocol.pio)

// In main.c initialization:
uint offset = pio_add_program(pio0, &new_protocol_program);
new_protocol_program_init(pio0, 2, offset, PIN_NEW_TX, baud_rate);
```

#### 3. GPIO Pin Assignment
```c
// In pins.h
#define PIN_NEW_PROTOCOL_TX    21  // Use available GPIO
#define PIN_NEW_PROTOCOL_RX    22
#define PIN_NEW_PROTOCOL_EN    23  // If direction control needed
```

### Testing Framework Integration

The separate testing framework at `../testing_fw/` can be used for:
- Hardware validation of new pin assignments
- Protocol timing verification with logic analyzer
- Stress testing of PIO state machine allocation
- USB descriptor enumeration testing

### Performance Optimization

#### PIO Resource Management
```c
// Check PIO availability before allocation
void allocate_pio_resources() {
    // PIO0: Motor protocols (half-duplex, needs precise timing)
    // PIO1: UART bridges (full-duplex, moderate timing requirements)  
    // PIO2: LEDs and optional protocols (Core 1, real-time)
    
    if (pio_can_add_program(target_pio, &program)) {
        uint offset = pio_add_program(target_pio, &program);
        // Initialize state machine
    }
}
```

#### Memory Management
- **DMA**: Use for high-speed protocols (>1Mbps)
- **Double Buffering**: For continuous data streams
- **IRQ Priorities**: CAN (highest), LEDs (lowest)

#### Core Utilization
- **Core 0**: USB, CAN, serial protocols, I2C
- **Core 1**: LED controller, optional real-time protocols
- **IPC**: FIFO queue for LED commands from Core 0

### Debugging & Diagnostics

#### LED Status Codes
- **Solid Green**: Normal operation
- **Flashing Red**: USB enumeration failure
- **Blue**: CAN bus active
- **Yellow**: I2C transaction
- **Purple**: Bootloader mode

#### Debug Output
```c
// Enable debug output via UART or USB CDC
#define DEBUG_UART   0  // Use UART0 for debug
#define DEBUG_USB    1  // Use first available CDC interface

printf("CAN: Frame received ID=0x%03X\n", frame.id);
```

#### Logic Analyzer Integration
- Export PIO symbol files for timing analysis
- GPIO test points on development boards
- Protocol decode plugins for popular analyzers

## Troubleshooting

### CAN Interface Issues
```bash
# Check if gs_usb module is loaded
lsmod | grep gs_usb

# Verify USB device detection  
lsusb | grep 1209:ac01

# Check interface registration
ip link show can1

# Debug mode with verbose output
candump -x can1

# Check interrupt line (GPIO9)
sudo cat /sys/kernel/debug/gpio
```

### Serial Interface Issues
```bash
# Check CDC enumeration
ls -la /dev/ttyACM*

# Verify udev rules
udevadm test /devices/.../ttyACM0

# Monitor udev events
sudo udevadm monitor

# Check symlinks
ls -la /dev/axon-*/

# Test with different baud rates
stty -F /dev/axon-1-1/rs485 115200
```

### PIO Resource Conflicts
```bash
# Check PIO utilization in firmware logs
dmesg | grep -i pio

# Verify state machine allocation
# (Add debug prints in firmware)

# Test individual protocols
echo "test" > /dev/axon-*/uart0  # Should work
echo "test" > /dev/axon-*/bus1   # Should work simultaneously
```

### Permission Issues
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Add user to input group for USB access
sudo usermod -a -G input $USER

# Logout and login again, or:
newgrp dialout
```

### Module Build Issues
```bash
# Install kernel headers
sudo apt install linux-headers-$(uname -r)

# Clean and rebuild DKMS modules
cd dkms/gs_usb-dkms
sudo dkms remove gs_usb/1.0 --all
sudo dkms add .
sudo dkms build gs_usb/1.0
sudo dkms install gs_usb/1.0

# Check build logs
sudo dkms status
```

## Building from Source

### Development Environment Setup
```bash
# Install dependencies
sudo apt install cmake gcc-arm-none-eabi build-essential git

# Clone Pico SDK (if not installed)
git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
export PICO_SDK_PATH=$HOME/pico-sdk
echo 'export PICO_SDK_PATH=$HOME/pico-sdk' >> ~/.bashrc

# Install additional tools for development
sudo apt install minicom picocom screen  # Serial terminal tools
sudo apt install can-utils               # CAN utilities
sudo apt install i2c-tools              # I2C utilities
```

### Build Configuration Options
```bash
# Standard build
cmake ..

# Enable UART2 (uses GPIO19/20)
cmake -DENABLE_UART2=ON ..

# Debug build with verbose output
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build (optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### Cross-Platform Development
```bash
# Windows (using WSL2)
sudo apt install gcc-arm-none-eabi cmake build-essential

# macOS
brew install cmake gcc-arm-embedded

# Docker container
docker run --rm -v $PWD:/workspace -w /workspace arm64v8/ubuntu:22.04 \
  bash -c "apt update && apt install -y cmake gcc-arm-none-eabi && cmake .. && make"
```

## Protocol Specifications

### gs_usb Protocol Implementation

#### Device Configuration
```c
struct gs_device_config {
    uint32_t reserved1;
    uint32_t reserved2; 
    uint32_t reserved3;
    uint32_t icount;     // Interface count (0 = 1 interface)
    uint32_t sw_version; // Software version
    uint32_t hw_version; // Hardware version
};
```

#### Bit Timing Constants
```c  
struct gs_device_bt_const {
    uint32_t feature;     // Supported features
    uint32_t fclk_can;    // CAN clock frequency (40MHz)
    uint32_t tseg1_min;   // Time segment 1 minimum
    uint32_t tseg1_max;   // Time segment 1 maximum  
    uint32_t tseg2_min;   // Time segment 2 minimum
    uint32_t tseg2_max;   // Time segment 2 maximum
    uint32_t sjw_max;     // Sync jump width maximum
    uint32_t brp_min;     // Bit rate prescaler minimum
    uint32_t brp_max;     // Bit rate prescaler maximum
    uint32_t brp_inc;     // Bit rate prescaler increment
};
```

#### Supported Features
- `GS_CAN_FEATURE_LISTEN_ONLY`: Monitor mode
- `GS_CAN_FEATURE_LOOP_BACK`: Loopback testing
- `GS_CAN_FEATURE_TRIPLE_SAMPLE`: Enhanced noise immunity
- `GS_CAN_FEATURE_ONE_SHOT`: Single-shot transmission
- `GS_CAN_FEATURE_HW_TIMESTAMP`: Hardware timestamps (if available)

### i2c-tiny-usb Protocol Implementation

#### Function Codes
```c
#define I2C_FUNC_I2C           0x00000001  // Basic I2C support
#define I2C_FUNC_SMBUS_EMUL    0x00000008  // SMBus emulation
```

#### Command Set
- `I2C_TINY_CMD_GET_FUNC`: Get supported functions
- `I2C_TINY_CMD_SET_DELAY`: Set bus timing delays
- `I2C_TINY_CMD_GET_STATUS`: Get bus status
- `I2C_TINY_CMD_I2C_IO`: Perform I2C transaction
- `I2C_TINY_CMD_I2C_IO_BEGIN`: Start transaction
- `I2C_TINY_CMD_I2C_IO_END`: End transaction

## Changelog

### v2.1.0 (Current Development)
- 🚧 Enhanced documentation with complete architecture details
- 🚧 Improved PIO resource management and allocation
- 🚧 Added comprehensive debugging and diagnostics
- 🚧 Extended development and extension guidelines

### v2.0.0 (Production)
- ✅ Full gs_usb CAN support with MCP2518FD transceiver control
- ✅ 5x CDC ACM serial interfaces with udev rules
- ✅ i2c-tiny-usb I2C interface with hardware I2C1
- ✅ NeoPixel activity LED indicators with dual-core control
- ✅ DKMS kernel module support for plug-and-play operation
- ✅ Unified motor bus supporting both Feetech and Dynamixel protocols
- ✅ Interface 0 placement for CAN compatibility with existing tools
- ✅ Comprehensive pin assignment documentation

### v1.0.0 (Testing)
- Basic hardware testing framework
- Individual peripheral validation
- Board bring-up utilities
- PIO program development and testing

## License

Copyright (C) 2026 RoboCore. All rights reserved.

## Support

For technical support and documentation:
- **Hardware**: [RoboCore Axon Documentation](https://www.robocore.net/)
- **Firmware Issues**: Create issue in this repository  
- **Linux Driver Issues**: Check kernel compatibility and DKMS installation
- **Development Questions**: See Development & Extension section above

## Contributing

1. **Fork** the repository
2. **Create** feature branch (`git checkout -b feature/new-protocol`)
3. **Test** thoroughly with hardware and testing framework
4. **Document** changes in README and code comments
5. **Submit** pull request with detailed description

### Code Style Guidelines
- Follow existing naming conventions (`snake_case` for functions/variables)
- Document all PIO programs with timing diagrams
- Add LED status indicators for new protocols
- Ensure backward compatibility with existing interfaces
- Test multi-device scenarios