# RoboCore Link101 gs_usb DKMS Module

This DKMS package provides gs_usb kernel driver support for RoboCore Link101 CAN interface on NVIDIA Tegra systems (and other systems where gs_usb is not included).

## What it does

- Adds RoboCore Link101 (VID:1209 PID:AC01) to gs_usb driver device table
- Creates SocketCAN interface when Link101 device is connected
- Works with standard Linux CAN tools (candump, cansend, etc.)

## Installation

```bash
cd /path/to/link101/firmware/dkms/gs_usb-dkms
sudo ./install.sh
```

## Usage

1. Connect RoboCore Link101 device
2. Check if CAN interface appeared:
   ```bash
   ip link show | grep can
   dmesg | tail
   ```

3. Configure and bring up CAN interface:
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set can0 up
   ```

4. Test with CAN tools:
   ```bash
   # Send test frame
   cansend can0 123#DEADBEEF
   
   # Monitor CAN traffic
   candump can0
   ```

## Requirements

- DKMS installed
- Kernel headers for running kernel
- can-utils package

## Firmware Requirements

The RoboCore Link101 firmware must:
- Use VID:1209 PID:AC01 
- Implement gs_usb protocol on vendor interface
- Have proper USB descriptors for gs_usb compatibility

## Troubleshooting

- Check `dmesg` after connecting device
- Verify `lsusb` shows device with correct VID/PID
- Ensure CAN bitrate is configured before bringing interface up
- Check `dkms status` to verify module is installed

## Uninstall

```bash
sudo dkms remove gs_usb/1.0 --all
```