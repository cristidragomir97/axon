# Link101 USB Composite Test — Claude Code Brief

## Goal

Build and flash a test firmware for RP2040 (Pico) that validates whether three USB interface types can coexist in a single composite device. This is a go/no-go test for the Link101 product architecture.

## The Question We're Answering

Can `i2c-tiny-usb` (device-level vendor requests on EP0) coexist with `gs_usb` (interface-level vendor requests on EP0) and CDC-ACM, all in one USB composite device on RP2040/RP2350?

## Project Location

The project lives at the path the user provides. It uses the Pico SDK (version 2.2.0, already installed).

## Architecture

The firmware presents a USB composite device with 4 interfaces:

| Interface | Type | Endpoints | Purpose |
|-----------|------|-----------|---------|
| IF0 (CDC-Comm) + IF1 (CDC-Data) | CDC-ACM | EP 0x81 notif, EP 0x02 OUT, EP 0x82 IN | UART echo test |
| IF2 | Vendor, 0 endpoints | Uses EP0 only | i2c-tiny-usb bridge |
| IF3 | Vendor, 2 endpoints | EP 0x03 OUT, EP 0x83 IN | gs_usb CAN loopback |

### EP0 Routing Strategy (the critical part)

All three interface types share EP0 for control transfers. The firmware routes them by inspecting the `bmRequestType` recipient field:

```
EP0 vendor request arrives in tud_vendor_control_xfer_cb()
  ├─ recipient == TUSB_REQ_RCPT_DEVICE     → i2c-tiny-usb handler
  └─ recipient == TUSB_REQ_RCPT_INTERFACE
       └─ wIndex == ITF_NUM_CAN (3)        → gs_usb handler
```

CDC class requests are handled by TinyUSB internally and never reach the vendor callback.

## Files

The project has these files:

- **CMakeLists.txt** — Build config. Links tinyusb_device, hardware_i2c, hardware_uart. Debug printf goes to UART0 (not USB, since USB is the device under test).
- **tusb_config.h** — TinyUSB configuration. 1 CDC + 1 VENDOR class. IMPORTANT: do NOT define CFG_TUSB_MCU or CFG_TUSB_OS (the SDK sets these via CMake). Must define CFG_TUSB_RHPORT0_MODE as OPT_MODE_DEVICE.
- **usb_descriptors.h** — Interface number defines shared between descriptors and main.
- **usb_descriptors.c** — USB device, configuration, and string descriptors. The configuration contains: 1 CDC-ACM (IAD + comm + data), 1 vendor interface with 0 endpoints (I2C), 1 vendor interface with 2 bulk endpoints (CAN).
- **main.c** — Entry point with three handlers: CDC echo, i2c-tiny-usb EP0 vendor handler, gs_usb EP0 vendor handler + bulk loopback.

## i2c-tiny-usb Protocol

The Linux kernel driver (`drivers/i2c/busses/i2c-tiny-usb.c`) sends these EP0 vendor requests with recipient = DEVICE:

| bRequest | Direction | Description | wValue | wLength |
|----------|-----------|-------------|--------|---------|
| 0x01 | IN | Read from I2C | address (7-bit) | bytes to read |
| 0x02 | OUT | Write to I2C | address (7-bit) | bytes to write |
| 0x03 | IN | Get status | 0 | 1 |
| 0x04 | IN | Get functionality | 0 | 4 |

Status responses: 0x00 = idle, 0x01 = ACK, 0x02 = NAK.
Functionality response: 32-bit bitmask, we report I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL.

Hardware: I2C0 on GP4 (SDA) / GP5 (SCL), 400kHz, with internal pull-ups enabled.

## gs_usb Protocol (subset)

The Linux kernel driver (`drivers/net/can/usb/gs_usb.c`) sends these EP0 vendor requests with recipient = INTERFACE, wIndex = interface number:

| bRequest | Direction | Description |
|----------|-----------|-------------|
| 0 | OUT | HOST_FORMAT |
| 1 | OUT | BITTIMING |
| 2 | OUT | MODE |
| 4 | IN | BT_CONST (bit timing constants) |
| 5 | IN | DEVICE_CONFIG |

For this test, HOST_FORMAT/BITTIMING/MODE are accepted and ignored. BT_CONST and DEVICE_CONFIG return plausible static structs.

Bulk data uses `gs_host_frame` (20 bytes): echo_id(4) + can_id(4) + dlc(1) + channel(1) + flags(1) + reserved(1) + data(8). The test firmware does loopback — any frame received on bulk OUT is echoed back on bulk IN.

## Pin Map

| Pin | Function |
|-----|----------|
| GP0 | Debug UART TX (stdio, 115200 baud) |
| GP1 | Debug UART RX |
| GP4 | I2C0 SDA |
| GP5 | I2C0 SCL |

## Build & Flash

```bash
mkdir build && cd build
cmake .. -DPICO_BOARD=pico  # RP2040
make -j$(nproc)
# Hold BOOTSEL, plug USB, release, then:
cp link101_usb_test.uf2 /Volumes/RPI-RP2/  # macOS
```

## Testing

### Phase 1: Does it enumerate? (macOS or Linux)

```bash
# Should show device with VID:PID 1209:AC01
# Should show 4 interfaces
lsusb -d 1209:ac01 -v  # Linux
# or on macOS: System Information → USB
```

### Phase 2: CDC echo (macOS or Linux)

```bash
# Find the CDC port
ls /dev/tty.usbmodem*   # macOS
ls /dev/ttyACM*          # Linux

# Send and receive
echo "hello" > /dev/ttyACM0
cat /dev/ttyACM0
```

### Phase 3: I2C (Linux only)

```bash
# Force-bind our VID:PID to the i2c-tiny-usb driver
echo "1209 AC01" | sudo tee /sys/bus/usb/drivers/i2c-tiny-usb/new_id

# Find the new I2C bus
i2cdetect -l | grep tiny

# Scan — should show devices if connected to GP4/GP5
sudo i2cdetect -y <bus_number>
```

### Phase 4: CAN bulk loopback (macOS or Linux with pyusb)

```python
import usb.core, usb.util, struct

dev = usb.core.find(idVendor=0x1209, idProduct=0xAC01)
dev.detach_kernel_driver(3)  # May fail on macOS, that's OK
usb.util.claim_interface(dev, 3)

# Test EP0 control: get device config
data = dev.ctrl_transfer(0xC1, 5, 0, 3, 12, 1000)
print(f"gs_usb config: {bytes(data).hex()}")

# Test EP0 control: i2c-tiny-usb func query (device-level)
data = dev.ctrl_transfer(0xC0, 4, 0, 0, 4, 1000)
print(f"i2c func: {bytes(data).hex()}")

# Test bulk loopback
frame = struct.pack("<IIBBBx8s", 42, 0x123, 8, 0, 0,
                    bytes([0xDE,0xAD,0xBE,0xEF,0xCA,0xFE,0xBA,0xBE]))
cfg = dev.get_active_configuration()
intf = cfg[(3,0)]
ep_out = [e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT][0]
ep_in = [e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN][0]
ep_out.write(frame)
resp = bytes(ep_in.read(20, timeout=1000))
print(f"Match: {resp == frame}")
```

### Phase 5: Simultaneous (Linux)

Run CDC echo, i2cdetect, and CAN bulk loopback at the same time. Any STALL, timeout, or wrong response = fail.

## Debug

All EP0 vendor requests are logged to UART0 (GP0) at 115200. Connect a USB-UART adapter and watch with:

```bash
minicom -D /dev/ttyUSB0 -b 115200  # Linux
screen /dev/tty.usbserial-XXX 115200  # macOS
```

Every request shows: stage, bRequest, wValue, wIndex, wLength, and recipient. This is how you see whether i2c-tiny-usb sends DEVICE or INTERFACE level requests.

## Known Risks

1. **TinyUSB may not route device-level vendor requests to `tud_vendor_control_xfer_cb`.** If the callback never fires for i2c-tiny-usb requests (visible in UART debug log — no [I2C] lines when i2cdetect runs), we need to use a lower-level hook. Check if TinyUSB has `tud_vendor_control_request_cb` or if we need to patch `usbd_control_xfer_cb`.

2. **The Linux i2c-tiny-usb driver may send INTERFACE-level requests, not DEVICE-level.** If debug log shows recipient=INTERFACE for I2C requests, the routing strategy breaks. Fix: route by bRequest range instead, or add wIndex check for ITF_NUM_I2C.

3. **VID:PID binding.** The i2c-tiny-usb kernel driver won't auto-bind to 1209:AC01. Must use `new_id` sysfs to force-bind. For production, either upstream a patch or ship a DKMS module.

4. **The I2C vendor interface descriptor (0 endpoints) might confuse some hosts.** A vendor interface with bNumEndpoints=0 is valid per spec but unusual. If enumeration fails, try adding a dummy interrupt endpoint.

## Success Criteria

1. Device enumerates with all 4 interfaces — no errors in dmesg
2. CDC echo works perfectly
3. i2cdetect completes a scan (finds devices if connected, shows empty bus if not)
4. CAN bulk loopback returns matching frames
5. CAN EP0 control requests return valid responses
6. All three work simultaneously without timeouts or STALLs
7. UART debug log shows clean routing: [I2C] lines for device-level, [CAN] lines for interface-level, never mixed

## If I2C Fails

The fallback is giving I2C its own CDC-ACM interface with a custom binary protocol. This costs 2 more USB endpoints (we have room — 13 of 16 used currently) but breaks compatibility with standard Linux I2C tools. The preferred recovery path is shipping a custom kernel module (`i2c-robocore`) that sends interface-targeted requests instead of device-targeted.

## What NOT to Do

- Do not add RTOS or async frameworks — bare metal polling loop is fine for this test
- Do not implement real CAN hardware — the loopback proves the USB layer works
- Do not implement real UART forwarding — CDC echo is sufficient
- Do not change VID:PID to match i2c-tiny-usb's expected IDs — we want to test with our own IDs
