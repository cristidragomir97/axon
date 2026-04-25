# Axon Multiprotocol Bridge Firmware

Production firmware for the RoboCore Axon board providing multiple communication interfaces over USB.

## Overview

The Axon board is a versatile USB-to-multiprotocol bridge built around the RP2350 microcontroller with dedicated hardware for:
- **CAN Bus** (MCP2518FD + SN65HVD230 transceiver)
- **RS485** (isolated transceiver ADM2587E)
- **UART Bridges** (2-3 general purpose UARTs)
- **I2C** (Qwiic/STEMMA QT compatible)

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
