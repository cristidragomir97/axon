# RoboCore Axon

ROS 2 motor controller firmware for the RoboCore Axon carrier board.

**Documentation:** [robocore.dev](https://robocore.dev)

## Board Overview

Axon is a carrier board that pairs a RP2350 with six motor protocols, I2C sensor ports, and a mikroBUS socket giving you access to over 2000 plug in boards over a single USB-C cable, publishing everything to ROS 2 via zenoh-pico as standard topics and services.

The firmware runs a 100Hz control loop on Core 0 while Core 1 handles USB. Motor commands arrive as `Float64MultiArray` messages on configurable topics (`base_cmd`, `arm_cmd`, etc.), and feedback goes out as standard `JointState` and `Imu` messages. The zenoh-pico transport means your ROS 2 nodes see the Axon as a native participant—no serial protocol parsing, no micro-ROS agent, just plug in and `ros2 topic list`.

A second USB CDC interface provides transparent passthrough for RPLIDAR C1 or similar sensors, so you get lidar + motor control + IMU over one cable.

### Hardware

| Feature | Specification |
|---------|---------------|
| MCU | RP2350 (dual Cortex-M33 @ 150MHz, 520KB SRAM) |
| Power | 12-24V screw terminal or USB-C PD (HUSB238) |
| USB | Composite CDC (zenoh transport + lidar passthrough) |
| Motor buses | 4 independent buses (see below) |
| Sensors | I2C (Qwiic), SPI (mikroBUS), 3x ADC |
| Expansion | mikroBUS socket, spare GPIO |

### Motor Bus Connectors

| Bus | Protocol | Baud | Use Case |
|-----|---------|------|----------|
| Feetech | Half-duplex UART | 1Mbps | SCS/STS servos |
| Dynamixel | | Half-duplex UART | 57600-4M | AX/MX/XL servos |
| DDSM | TTL UART | 115200 | DDSM210 wheels |
| RS-485 | Differential | 115200 | Industrial motors |

### Other Connectors

- **J1/J2** — Qwiic I2C (JST-SH 4-pin) for BNO055, etc.
- **U16** — RPLIDAR (JST-XH 5-pin) passthrough to USB CDC1
- **MB1** — mikroBUS socket for Click expansion boards

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  SBC (Raspberry Pi, Jetson, etc.)                               │
│  ┌──────────────┐  ┌──────────────┐                             │
│  │ ROS 2 Nodes  │  │ rplidar_ros2 │                             │
│  └──────┬───────┘  └──────┬───────┘                             │
│         │ zenoh           │ serial                              │
│         ▼                 ▼                                     │
│  /dev/robocore_zenoh    /dev/robocore_lidar                     │
└─────────────────────────────────────────────────────────────────┘
          │ USB CDC0              │ USB CDC1
          ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│  RoboCore Axon (RP2350)                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   picoros   │  │ lidar_bridge│  │   motors    │              │
│  │  (zenoh)    │  │ (passthru)  │  │  (control)  │              │
│  └──────┬──────┘  └─────────────┘  └──────┬──────┘              │
│         │                                 │                     │
│         ▼                                 ▼                     │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                      Transport Layer                       │ │
│  │  bus_feetech   bus_dynamixel   bus_ddsm   bus_rs485        │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
          │               │              │            │
          ▼               ▼              ▼            ▼
      ┌───────┐      ┌────────┐     ┌────────┐   ┌────────┐
      │Feetech│      │Dynamixel│    │ DDSM210│   │ RS-485 │
      │Servos │      │ Servos │     │ Wheels │   │ Motors │
      └───────┘      └────────┘     └────────┘   └────────┘
```

## Firmware Philosophy

This firmware follows an **Arduino-like "sketch" pattern**:

- **`lib/`** — The SDK. Hardware drivers, transport abstractions, motor protocols. Treat as read-only, unless you want to extend it, in that case we are waiting for your pull request. 
- **`sketch/`** — Your code. Define your robot, wire up motors, customize ROS topics.

The SDK provides building blocks; you assemble them for your specific robot. 

## Project Structure

```
robocore-axon/
├── lib/                        # SDK
│   ├── board.c/h               # Hardware init, bus instances
│   ├── motor.h                 # Motor interface (vtable pattern)
│   ├── transport/              # UART, PIO UART, RS-485
│   │   ├── transport.h         # Abstract transport interface
│   │   ├── uart_hw.c           # Hardware UART (half-duplex)
│   │   ├── uart_pio.c          # PIO UART (full-duplex)
│   │   └── rs485.c             # RS-485 with DE control
│   ├── protocol/               # Motor drivers
│   │   ├── feetech.c/h         # Feetech SCS/STS servos
│   │   └── ddsm210.c/h         # Waveshare DDSM210 wheels
│   ├── pio/                    # PIO programs
│   └── lidar_bridge.c/h        # RPLIDAR passthrough
│
├── sketch/                     # YOUR CODE
│   ├── main.c                  # Entry point, ROS integration
│   ├── motors.c/h              # Motor instances and control
│   ├── ros.c/h                 # picoros publishers/subscribers
│   └── robot_config.h          # IDs, counts, timing
│
├── pinout.h                    # Board pin definitions
├── config.h                    # Feature flags
└── CMakeLists.txt
```

## Transport Layer

Transports abstract the physical communication layer. Each bus is a `transport_t` with a common interface:

```c
// transport.h
typedef struct {
    bool (*write)(transport_t *t, const uint8_t *data, size_t len);
    size_t (*read)(transport_t *t, uint8_t *buf, size_t max_len);
    bool (*readable)(transport_t *t);
    void (*flush)(transport_t *t);
    void (*set_tx_mode)(transport_t *t, bool tx);  // For half-duplex
} transport_ops_t;
```

### Available Transports

| Transport | Implementation | Notes |
|-----------|----------------|-------|
| `uart_hw` | Hardware UART | Half-duplex via 2N3906 + 74HC126 circuit |
| `uart_pio` | PIO state machine | Full-duplex, any GPIO pair |
| `rs485` | PIO + DE pin | MAX3485 transceiver with direction control |

### Pre-configured Buses

These are initialized by `board_init()` and ready to use:

```c
extern transport_t bus_feetech;    // UART1, 1Mbps, half-duplex
extern transport_t bus_dynamixel;  // UART0, 57600, half-duplex
extern transport_t bus_ddsm;       // PIO, 115200, full-duplex
extern transport_t bus_rs485;      // PIO + DE, 115200
```

## Motor Protocols

Motor drivers implement the `motor_ops_t` interface:

```c
// motor.h
typedef struct {
    bool (*init)(motor_t *motor);
    bool (*set_enabled)(motor_t *motor, bool enabled);
    bool (*set_command)(motor_t *motor, const motor_cmd_t *cmd);
    bool (*get_state)(motor_t *motor, motor_state_t *state);
    void (*poll)(motor_t *motor);
} motor_ops_t;

typedef struct {
    float position;     // radians
    float velocity;     // rad/s
} motor_cmd_t;
```

### Feetech SCS/STS

Position or velocity controlled servos (SCS0009, STS3215, etc.) at 1Mbps half-duplex.

```c
#include "protocol/feetech.h"

motor_t arm_servo;
feetech_motor_init(&arm_servo, &bus_feetech, 1, "shoulder");
arm_servo.ops->init(&arm_servo);

// Command position
motor_cmd_t cmd = {.mode = MOTOR_MODE_POSITION, .position = 1.57};
arm_servo.ops->set_command(&arm_servo, &cmd);
```

### DDSM210
Waveshare brushless wheel motors with integrated driver. Velocity or position mode.

```c
#include "protocol/ddsm210.h"

motor_t wheel;
ddsm210_motor_init(&wheel, &bus_ddsm, 1, "wheel_left");
wheel.ops->init(&wheel);

// Command velocity (rad/s)
motor_cmd_t cmd = {.mode = MOTOR_MODE_VELOCITY, .velocity = 3.14};
wheel.ops->set_command(&wheel, &cmd);
```

**Protocol details:**
- 10-byte packets with CRC-8/MAXIM checksum
- Velocity: ±210 RPM (0.1 RPM resolution)
- Position: 0-360° (0.01° resolution)
- Feedback: velocity, position, temperature, error code

## Example Sketch

The included sketch implements a diff-drive robot with a 6-DOF arm:

### robot_config.h

```c
// Motor IDs (set via motor configuration tools)
#define WHEEL_LEFT_ID   1
#define WHEEL_RIGHT_ID  2
#define NUM_WHEELS      2

#define ARM_J1_ID  1    // shoulder_pan
#define ARM_J2_ID  2    // shoulder_lift
// ...
#define NUM_ARM    6

#define CAM_PAN_ID   7
#define CAM_TILT_ID  8
#define NUM_CAMERA   2

// Timing
#define CONTROL_RATE_HZ   100  // Motor commands
#define PUBLISH_RATE_HZ   50   // ROS feedback
```

### motors.c

```c
void motors_init(void) {
    // Wheels - DDSM210 velocity control
    ddsm210_motor_init(&wheels[0], &bus_ddsm, WHEEL_LEFT_ID, "wheel_left");
    ddsm210_motor_init(&wheels[1], &bus_ddsm, WHEEL_RIGHT_ID, "wheel_right");

    // Arm - Feetech position control
    feetech_motor_init(&arm[0], &bus_feetech, ARM_J1_ID, "shoulder_pan");
    feetech_motor_init(&arm[1], &bus_feetech, ARM_J2_ID, "shoulder_lift");
    // ...

    // Initialize all
    for (int i = 0; i < NUM_WHEELS; i++)
        wheels[i].ops->init(&wheels[i]);
    // ...
}
```

### ROS Topics

**Subscribes:**
| Topic | Type | Description |
|-------|------|-------------|
| `base_cmd` | Float64MultiArray | Wheel velocities [left, right] rad/s |
| `arm_cmd` | Float64MultiArray | Arm positions [j1..j6] rad |
| `camera_cmd` | Float64MultiArray | Camera angles [pan, tilt] rad |

**Publishes:**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `joint_states` | JointState | 50Hz | All motor positions/velocities |
| `imu/data` | Imu | 50Hz | BNO055 orientation, gyro, accel |

## Building

```bash
# Clone with submodules
git clone --recursive https://github.com/robocore/robocore-axon

# Build
mkdir build && cd build
cmake ..
make -j4

# Flash
# Hold BOOTSEL, plug USB, release
cp firmware.uf2 /Volumes/RP2350/
```

### Dependencies (fetched automatically)

- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software) — zenoh-pico ROS 2 bridge
- [pico-bno055](https://github.com/alpertng02/pico-bno055) — IMU driver

## Adding a Motor Driver

1. Create `lib/protocol/mymotor.h`:
```c
#include "motor.h"
#include "transport/transport.h"

void mymotor_init(motor_t *motor, transport_t *transport,
                  uint8_t id, const char *name);
```

2. Implement `lib/protocol/mymotor.c`:
```c
static bool mymotor_set_command(motor_t *motor, const motor_cmd_t *cmd) {
    // Build and send packet via motor->transport
    // Parse response, update motor->state
}

static const motor_ops_t mymotor_ops = {
    .init = mymotor_init_hw,
    .set_enabled = mymotor_set_enabled,
    .set_command = mymotor_set_command,
    .get_state = mymotor_get_state,
    .poll = mymotor_poll,
};
```

3. Add to `CMakeLists.txt`:
```cmake
add_library(axon_lib STATIC
    # ...
    lib/protocol/mymotor.c
)
```

4. Use in your sketch:
```c
#include "protocol/mymotor.h"
mymotor_init(&motor, &bus_rs485, 1, "joint1");
```

## License

MIT
