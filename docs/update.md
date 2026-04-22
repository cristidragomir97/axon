# Axon USB Composite Test — Claude Code Brief

## What This Is

A test firmware for RP2040 (Pico) that validates three USB interface types coexisting in one composite device. This is a go/no-go test for the RoboCore Axon product architecture.

There is also a companion Linux kernel module (`robocore-i2c-dkms/`) that must be installed on the test machine.

## Current State

We have working firmware and a working kernel module. The descriptor layout, interface filtering, and DMA-safe buffers are all validated. The remaining step is aligning the firmware's I2C protocol handler with the real i2c-tiny-usb wire protocol so the kernel driver completes its probe.

### What Works (confirmed on hardware)
- USB composite device enumerates with 4 interfaces
- CDC-ACM echo works
- CAN (gs_usb) EP0 control requests route correctly
- I2C EP0 vendor requests route correctly (device-level → I2C handler, interface-level → CAN handler)
- Kernel module `i2c-robocore` matches ONLY interface 3 (subclass=0x01, protocol=0x01), skips interface 2 (CAN)
- DMA-safe buffer in kernel module — no more "transfer buffer is on stack" warnings
- pyusb test confirms both device-level and interface-level vendor requests coexist on EP0

### What Doesn't Work Yet
- Kernel module probe fails with error -5 (EIO) because firmware uses wrong i2c-tiny-usb command numbers
- The firmware was initially written with CMD_READ=0x01, CMD_WRITE=0x02 — the REAL protocol uses CMD_GET_FUNC=0x01, CMD_I2C_IO=0x04+flags
- The updated main.c with correct protocol is written (see below) but has not been flashed and tested yet

## Architecture

```
USB Composite Device (VID:1209 PID:AC01)
├── IF0+1: CDC-ACM          (TinyUSB CDC class, 3 endpoints)
├── IF2:   CAN/gs_usb       (TinyUSB vendor class, 2 bulk endpoints)
└── IF3:   I2C/i2c-tiny-usb (TinyUSB vendor class, 2 bulk endpoints*)
                              * endpoints required by TinyUSB but never used
                              * all I2C data goes through EP0 control transfers
```

### EP0 Routing

```
tud_vendor_control_xfer_cb() receives ALL vendor requests
  ├─ recipient == TUSB_REQ_RCPT_DEVICE     → i2c-tiny-usb handler
  └─ recipient == TUSB_REQ_RCPT_INTERFACE
       └─ wIndex == ITF_NUM_CAN (2)        → gs_usb handler
```

This routing is CONFIRMED WORKING via pyusb tests.

## Firmware Files

Project uses Pico SDK 2.2.0 with TinyUSB.

### tusb_config.h
- `CFG_TUD_VENDOR=2` — MUST be 2, not 1. TinyUSB walks all interfaces during set_configuration and STALLs if it can't find a class driver for an interface. With VENDOR=1, it claims IF2 (CAN) but rejects IF3 (I2C), causing `error -32 (EPIPE)` on the host.
- Do NOT define `CFG_TUSB_MCU` or `CFG_TUSB_OS` — the Pico SDK sets these via CMake. Redefining them causes warnings.
- Must define `CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE` — required by SDK 2.x TinyUSB.

### usb_descriptors.h
- `ITF_NUM_CAN=2`, `ITF_NUM_I2C=3` — CAN must come before I2C in descriptor order so TinyUSB's first vendor instance claims CAN's bulk endpoints.
- I2C has dummy bulk endpoints (EP_I2C_OUT=0x04, EP_I2C_IN=0x84) — allocated to satisfy TinyUSB but never used for data.

### usb_descriptors.c
- CAN uses standard `TUD_VENDOR_DESCRIPTOR` (subclass=0, protocol=0)
- I2C uses custom `TUD_VENDOR_DESCRIPTOR_EX` macro that sets subclass=0x01, protocol=0x01
- These subclass/protocol values are how the kernel module distinguishes I2C from CAN
- The custom macro is identical to TUD_VENDOR_DESCRIPTOR except for the two class bytes:

```c
#define TUD_VENDOR_DESCRIPTOR_EX(_itfnum, _stridx, _epout, _epin, _epsize, _subclass, _protocol) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, _subclass, _protocol, _stridx, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0
```

### main.c — I2C Protocol (CRITICAL — this is what needs to work)

The i2c-tiny-usb protocol as implemented by the Linux kernel driver (`drivers/i2c/busses/i2c-tiny-usb.c`):

| bRequest | Direction | bmRequestType recipient | Purpose |
|----------|-----------|------------------------|---------|
| 0x00 | varies | DEVICE | CMD_ECHO |
| 0x01 | IN | DEVICE | CMD_GET_FUNC — return 4-byte LE functionality bitmask |
| 0x02 | OUT | DEVICE | CMD_SET_DELAY — wValue = delay in us, no data phase |
| 0x03 | IN | DEVICE | CMD_GET_STATUS — return 1-byte status |
| 0x04-0x07 | IN or OUT | DEVICE | CMD_I2C_IO with flags |

CMD_I2C_IO detail:
- `bRequest = 0x04 | flags` where flags = BEGIN (bit 0) | END (bit 1)
- So bRequest can be 0x04, 0x05, 0x06, or 0x07
- `wValue` = I2C message flags (bit 0 = I2C_M_RD)
- `wIndex` = I2C slave address (7-bit)
- `wLength` = data length
- Direction determined by bmRequestType: IN = read from slave, OUT = write to slave
- `nostop` = !(flags & CMD_I2C_IO_END) — if END not set, hold bus for repeated start

Typical kernel driver sequence for i2cdetect scan of address 0x28:
1. CMD_I2C_IO | BEGIN | END, wValue=0 (write), wIndex=0x28, wLength=0 → firmware does zero-length write, sets status ACK or NAK
2. CMD_GET_STATUS → firmware returns 1 byte: 0x01 (ACK, device present) or 0x02 (NAK, no device)

Status codes: IDLE=0x00, ACK=0x01, NAK=0x02

Functionality bitmask: `I2C_FUNC_I2C (0x01) | I2C_FUNC_SMBUS_EMUL (0xEFF9)` = 0x0000EFFA as little-endian 4 bytes.

### main.c — CAN Protocol

gs_usb EP0 control requests use recipient=INTERFACE with wIndex=ITF_NUM_CAN(2):
- BREQ 0: HOST_FORMAT (accept, ignore)
- BREQ 1: BITTIMING (accept, ignore)
- BREQ 2: MODE (accept, ignore)
- BREQ 4: BT_CONST (return static struct)
- BREQ 5: DEVICE_CONFIG (return static struct)

Bulk loopback: any gs_host_frame (20 bytes) received on vendor instance 0 OUT is echoed back on IN.

With CFG_TUD_VENDOR=2, use `tud_vendor_n_read(0, ...)` and `tud_vendor_n_write(0, ...)` for CAN (vendor instance 0 = IF2). Vendor instance 1 (IF3/I2C) has bulk endpoints but they're never touched.

### main.c — CDC

Simple echo. `tud_cdc_read()` → `tud_cdc_write()`. Nothing special.

## Kernel Module: robocore-i2c-dkms/

### i2c-robocore.c

Fork of i2c-tiny-usb with three fixes:
1. **Interface filtering**: `probe()` checks `bInterfaceSubClass==0x01 && bInterfaceProtocol==0x01`, returns -ENODEV otherwise
2. **USB_DEVICE_AND_INTERFACE_INFO matching**: ID table matches VID+PID+class+subclass+protocol for auto-bind
3. **DMA-safe buffer**: All USB control transfers use a `kmalloc`'d 64-byte buffer, protected by mutex. Stack buffers cause "transfer buffer is on stack" kernel warnings and transfer failures on modern kernels (since ~4.9).

### Installation

```bash
sudo apt install dkms linux-headers-$(uname -r)
sudo bash install.sh install
# This:
# - Copies source to /usr/src/i2c-robocore-1.0.0/
# - Builds via DKMS (auto-rebuilds on kernel updates)
# - Installs udev rules for /dev/robocore/i2c symlink
# - Blacklists upstream i2c-tiny-usb to prevent conflicts
```

IMPORTANT: i2c-tiny-usb MUST be blacklisted. It matches on bInterfaceClass=0xFF with no subclass filtering, so it grabs both vendor interfaces and fails on both. The install script handles this.

## Pin Map

| Pin | Function |
|-----|----------|
| GP0 | Debug UART TX (stdio, 115200) |
| GP1 | Debug UART RX |
| GP4 | I2C0 SDA |
| GP5 | I2C0 SCL |

## Build & Flash

```bash
cd rp_test  # or wherever the firmware project is
mkdir -p build && cd build
rm -rf *
cmake .. -DPICO_BOARD=pico
make -j$(nproc)
# Hold BOOTSEL, plug USB, release:
cp rp_test.uf2 /media/$USER/RPI-RP2/
```

IMPORTANT: Always do a clean build (`rm -rf *` in build dir) when changing descriptors or tusb_config.h. CMake caches can cause stale binaries.

## Testing

After flashing firmware and installing kernel module:

```bash
# Check enumeration
lsusb -d 1209:ac01 -v 2>/dev/null | grep -B1 -A8 "bInterfaceNumber"
# IF3 must show: bInterfaceSubClass=1, bInterfaceProtocol=1

# Check kernel module binding
dmesg | tail -20 | grep -i robocore
# Should show: "RoboCore I2C adapter registered as i2c-N"

# CDC echo test
echo "hello" > /dev/ttyACM0

# I2C scan (connect a sensor to GP4/GP5)
i2cdetect -l | grep -i robocore
sudo i2cdetect -y <bus_number>

# pyusb smoke test (works on macOS too)
python3 -c "
import usb.core
dev = usb.core.find(idVendor=0x1209, idProduct=0xAC01)
print('i2c func:', bytes(dev.ctrl_transfer(0xC0, 1, 0, 0, 4, 1000)).hex())
print('gs_usb cfg:', bytes(dev.ctrl_transfer(0xC1, 5, 0, 2, 12, 1000)).hex())
"
```

## Bugs We Already Hit and Fixed (DO NOT REINTRODUCE)

1. **Missing `#` on header guard** — `ifndef` instead of `#ifndef` in usb_descriptors.h
2. **Redefining CFG_TUSB_MCU / CFG_TUSB_OS** — Pico SDK sets these, redefining causes warnings
3. **Missing CFG_TUSB_RHPORT0_MODE** — SDK 2.x requires this, without it you get static_assert failure
4. **CFG_TUD_VENDOR=1 with two vendor interfaces** — TinyUSB STALLs set_configuration because it can't find a class driver for the second vendor interface. Must be CFG_TUD_VENDOR=2.
5. **I2C interface with 0 endpoints (raw descriptor)** — TinyUSB must "open" every interface during set_configuration. An interface it doesn't manage causes STALL. Give I2C dummy bulk endpoints.
6. **subclass=0 protocol=0 on I2C interface** — TUD_VENDOR_DESCRIPTOR hardcodes these to 0. Must use custom TUD_VENDOR_DESCRIPTOR_EX macro to set subclass=0x01 protocol=0x01.
7. **i2c-tiny-usb kernel driver grabbing both vendor interfaces** — Upstream driver has no subclass filtering. Blacklist it, use i2c-robocore instead.
8. **Stack buffer for USB control transfers in kernel module** — Modern kernels require DMA-safe (kmalloc'd) buffers. Stack buffers cause kernel WARNING and -EAGAIN.
9. **Wrong i2c-tiny-usb command numbers in firmware** — Our test used CMD_READ=0x01, CMD_WRITE=0x02. Real protocol: CMD_GET_FUNC=0x01, CMD_I2C_IO=0x04+flags.

## Success Criteria

1. Device enumerates with 4 interfaces, no errors in dmesg
2. IF3 shows bInterfaceSubClass=1, bInterfaceProtocol=1 in lsusb
3. i2c-robocore binds to IF3 only, not IF2
4. `i2cdetect` completes a scan (shows devices on the bus if any are connected)
5. CDC echo works
6. CAN bulk loopback works
7. All three work simultaneously

## What NOT to Do

- Do not use CFG_TUD_VENDOR=1 (must be 2)
- Do not use TUD_VENDOR_DESCRIPTOR for I2C (must use EX variant with subclass/protocol)
- Do not put USB transfer buffers on the stack in the kernel module
- Do not use the upstream i2c-tiny-usb module (must blacklist it)
- Do not define CFG_TUSB_MCU or CFG_TUSB_OS in tusb_config.h
- Do not use raw 0-endpoint interface descriptors (TinyUSB rejects them)

## Immediate Next Steps

1. Flash the updated main.c (with real i2c-tiny-usb protocol) and verify kernel module probe succeeds
2. Run `i2cdetect` to confirm the bus works end-to-end
3. Connect a real I2C device (BNO055, BME280, etc.) and read data through the adapter
4. Verify CAN bulk loopback still works alongside I2C
5. Verify CDC echo still works alongside both