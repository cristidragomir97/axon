/**
 * @file board_pins.h
 * @brief RoboCore Axon Carrier Board — Pin Definitions & Hardware Config
 *
 * MCU:    Raspberry Pi Pico 2 W (RP2350, dual Cortex-M33 @ 150MHz, 520KB SRAM)
 * USB:    Composite CDC — CDC0 zenoh-pico, CDC1 lidar bridge
 * Board:  Rev 1.0 (2026-02-10)
 *
 * Half-duplex circuit: 2N3906 PNP + 74HC126 tri-state buffer
 *   - TX line drives transistor base → generates TXEN for buffer gate
 *   - Channels 1-2: Feetech, Channels 3-4: Dynamixel
 */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* ──────────────────────────────────────────────
 *  RS-485 Bus (PIO UART + MAX3485 transceiver)
 *  Connector: CN5 screw terminal / CN8-10 ZH1.5
 *  Pinout: 1-GND, 2-A, 3-B [, 4-MOTOR_12V]
 *  Termination: 120Ω (R11), Bias: A→3V3, B→GND
 * ────────────────────────────────────────────── */
#define PIN_RS485_TX            0   /* GP0  — PIO UART TX */
#define PIN_RS485_RX            1   /* GP1  — PIO UART RX */
#define PIN_RS485_DE            2   /* GP2  — MAX3485 DE/RE# (active high = TX) */
#define RS485_BAUD              115200
#define RS485_PIO               pio0
#define RS485_SM_TX             0
#define RS485_SM_RX             1

/* ──────────────────────────────────────────────
 *  DDSM210 Motor Bus (PIO UART, plain TTL)
 *  Connectors: CN1-CN4, ZH1.5 4-pin, bus-addressable
 *  Pinout: 1-MOTOR_12V, 2-GND, 3-DDS_TX, 4-DDS_RX
 *  Protocol: 115200 baud, IDs 1-253
 * ────────────────────────────────────────────── */
#define PIN_DDS_TX              3   /* GP3  — PIO UART TX → motor RX (pin 3) */
#define PIN_DDS_RX              4   /* GP4  — PIO UART RX ← motor TX (pin 4) */
#define DDS_BAUD                115200
#define DDS_PIO                 pio0
#define DDS_SM_TX               2
#define DDS_SM_RX               3
#define DDS_MAX_MOTORS          4
#define DDS_SPEED_UNIT_RPM_X10  1   /* cmd=100 → 10.0 RPM */

/* ──────────────────────────────────────────────
 *  Feetech SCS/STS Servo Bus (HW UART1, half-duplex)
 *  Connectors: CN6-CN7, 5264-3P
 *  Pinout: 1-GND, 2-MOTOR_12V, 3-ST_DATA
 *  Half-duplex: Q1 (2N3906) + 74HC126 ch1-2
 *  Direction auto-switched by TX line level
 * ────────────────────────────────────────────── */
#define PIN_ST_TX               5   /* GP5  — UART1 TX → half-duplex data line */
#define PIN_ST_RX               6   /* GP6  — UART1 RX ← half-duplex data line */
#define ST_UART                 uart1
#define ST_BAUD                 1000000  /* STS3215 default 1Mbps */

/* ──────────────────────────────────────────────
 *  Dynamixel TTL Servo Bus (HW UART0, half-duplex)
 *  Connectors: U5/U7 (12V), U17 (5V for XL330)
 *  Pinout: 1-GND, 2-VDD, 3-DATA
 *  Half-duplex: Q2 (2N3906) + 74HC126 ch3-4
 *  Direction auto-switched by TX line level
 * ────────────────────────────────────────────── */
#define PIN_DX_TX               7   /* GP7  — UART0 TX → half-duplex data line */
#define PIN_DX_RX               8   /* GP8  — UART0 RX ← half-duplex data line */
#define DX_UART                 uart0
#define DX_BAUD                 57600   /* Dynamixel default; XL330 supports up to 4Mbps */

/* ──────────────────────────────────────────────
 *  mikroBUS Socket (MB1)
 *  Standard 2×8 header, 2.54mm pitch
 *  Left:  AN  RST CS  SCK MISO MOSI 3V3 GND
 *  Right: PWM INT TX  RX  SCL  SDA  5V  GND
 *  Click boards: LIN, CAN, Ethernet, etc.
 * ────────────────────────────────────────────── */
#define PIN_MB_RST              9   /* GP9  — mikroBUS reset */
#define PIN_MB_SCK              10  /* GP10 — SPI1 SCK */
#define PIN_MB_MOSI             11  /* GP11 — SPI1 MOSI */
#define PIN_MB_MISO             12  /* GP12 — SPI1 MISO */
#define PIN_MB_CS               13  /* GP13 — SPI1 CS */
#define PIN_MB_INT              16  /* GP16 — interrupt input */
#define PIN_MB_TX               17  /* GP17 — PIO UART TX */
#define PIN_MB_RX               18  /* GP18 — PIO UART RX */
#define PIN_MB_PWM              19  /* GP19 — PWM output */
#define PIN_MB_AN               26  /* GP26 — ADC0 analog input */
#define MB_SPI                  spi1
#define MB_PIO                  pio1
#define MB_SM_TX                0
#define MB_SM_RX                1

/* ──────────────────────────────────────────────
 *  I2C Sensor Bus (shared: Qwiic + mikroBUS + BNO055)
 *  Connectors: J1/J2 JST-SH 4-pin (Qwiic)
 *  Pinout: 1-GND, 2-3V3, 3-SDA, 4-SCL
 *  No on-board pull-ups (modules provide their own)
 * ────────────────────────────────────────────── */
#define PIN_I2C_SDA             14  /* GP14 — I2C1 SDA */
#define PIN_I2C_SCL             15  /* GP15 — I2C1 SCL */
#define SENSOR_I2C              i2c1
#define I2C_FREQ_HZ             400000  /* 400kHz fast mode */

/* Known I2C addresses */
#define I2C_ADDR_BNO055         0x28
#define I2C_ADDR_HUSB238        0x08

/* ──────────────────────────────────────────────
 *  RPLIDAR C1 Passthrough (PIO UART → USB CDC1)
 *  Connector: U16, JST XH 5-pin
 *  Pinout: 1-5V, 2-LIDAR_TX, 3-LIDAR_RX,
 *          4-GND, 5-MOTOCTL  (verify pin 4/5!)
 *  Transparent bridge — MCU does not parse RPLIDAR protocol
 *  rplidar_ros2 on SBC points at /dev/robocore_lidar
 * ────────────────────────────────────────────── */
#define PIN_LIDAR_TX            20  /* GP20 — PIO UART TX → lidar RX (pin 3) */
#define PIN_LIDAR_RX            21  /* GP21 — PIO UART RX ← lidar TX (pin 2) */
#define LIDAR_BAUD              460800
#define LIDAR_PIO               pio1
#define LIDAR_SM_TX             2
#define LIDAR_SM_RX             3
#define LIDAR_BRIDGE_BUF_SIZE   4096  /* ring buffer bytes */

/* ──────────────────────────────────────────────
 *  Spare / Expansion
 * ────────────────────────────────────────────── */
#define PIN_SPARE_GPIO          22  /* GP22 — e-stop / MOTOCTL / general */
#define PIN_SPARE_ADC1          27  /* GP27 — ADC1 */
#define PIN_SPARE_ADC2          28  /* GP28 — ADC2 */

/* ──────────────────────────────────────────────
 *  USB Composite Device (TinyUSB)
 *  CDC0: zenoh-pico transport (/dev/robocore_zenoh)
 *  CDC1: lidar UART bridge  (/dev/robocore_lidar)
 * ────────────────────────────────────────────── */
#define USB_CDC_ZENOH           0
#define USB_CDC_LIDAR           1
#define USB_VID                 0x2E8A  /* Raspberry Pi */
#define USB_PID                 0x000A  /* Pico SDK composite */

/* ──────────────────────────────────────────────
 *  PIO State Machine Allocation
 *
 *  pio0: SM0 RS485_TX    pio1: SM0 MB_TX
 *        SM1 RS485_RX          SM1 MB_RX
 *        SM2 DDS_TX            SM2 LIDAR_TX
 *        SM3 DDS_RX            SM3 LIDAR_RX
 *
 *  6/12 SMs used. 6 free for encoders / WS2812 / 1-wire
 * ────────────────────────────────────────────── */


#endif /* BOARD_PINS_H */
