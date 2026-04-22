#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

//--------------------------------------------------------------------
// Axon USB Composite Device - Interface Layout
//--------------------------------------------------------------------
//
// CDC #0 (IF0+1):   RS485 serial bridge
// CDC #1 (IF2+3):   Motor bus (unified Feetech/Dynamixel)
// CDC #2 (IF4+5):   UART0 (DDSM)
// CDC #3 (IF6+7):   UART1 (Lidar)
#ifdef ENABLE_UART2
// CDC #4 (IF8+9):   UART2 (Additional)
#endif
// Vendor #0 (IF10): CAN (gs_usb protocol)
// Vendor #1 (IF11): I2C (i2c-tiny-usb protocol)
//
#ifdef ENABLE_UART2
// Total: 12 interfaces
#else
// Total: 10 interfaces
#endif
//--------------------------------------------------------------------

// Interface numbers - CAN moved to interface 0 for better compatibility
#define ITF_NUM_CAN        0   // gs_usb - moved to interface 0
#define ITF_NUM_CDC0_COMM  1   // RS485
#define ITF_NUM_CDC0_DATA  2
#define ITF_NUM_CDC1_COMM  3   // Motor bus (unified)
#define ITF_NUM_CDC1_DATA  4
#define ITF_NUM_CDC2_COMM  5   // UART0 (DDSM)
#define ITF_NUM_CDC2_DATA  6
#define ITF_NUM_CDC3_COMM  7   // UART1 (Lidar)
#define ITF_NUM_CDC3_DATA  8
#ifdef ENABLE_UART2
#define ITF_NUM_CDC4_COMM  9   // UART2 (Additional)
#define ITF_NUM_CDC4_DATA  10
#define ITF_NUM_I2C        11  // i2c-tiny-usb
#define ITF_NUM_TOTAL      12
#else
#define ITF_NUM_I2C        9   // i2c-tiny-usb
#define ITF_NUM_TOTAL      10
#endif

// CDC #0 endpoints (RS485) - moved to free up EP 1/2 for CAN
#define EP_CDC0_NOTIF      0x82
#define EP_CDC0_OUT        0x03
#define EP_CDC0_IN         0x83

// CDC #1 endpoints (Motor bus)
#define EP_CDC1_NOTIF      0x85
#define EP_CDC1_OUT        0x04
#define EP_CDC1_IN         0x86

// CDC #2 endpoints (UART0/DDSM)
#define EP_CDC2_NOTIF      0x87
#define EP_CDC2_OUT        0x05
#define EP_CDC2_IN         0x88

// CDC #3 endpoints (UART1/Lidar)
#define EP_CDC3_NOTIF      0x89
#define EP_CDC3_OUT        0x06
#define EP_CDC3_IN         0x8A

#ifdef ENABLE_UART2
// CDC #4 endpoints (UART2/Additional)
#define EP_CDC4_NOTIF      0x8C
#define EP_CDC4_OUT        0x07
#define EP_CDC4_IN         0x8D
#endif

// CAN bulk endpoints (gs_usb) - using required EP 1/2  
#define EP_CAN_OUT         0x02
#define EP_CAN_IN          0x81

// I2C bulk endpoints (required by TinyUSB, never actually used for data)
#ifdef ENABLE_UART2
#define EP_I2C_OUT         0x08
#define EP_I2C_IN          0x8E
#else
#define EP_I2C_OUT         0x07
#define EP_I2C_IN          0x8B
#endif

//--------------------------------------------------------------------
// CDC port mapping to bus drivers
//--------------------------------------------------------------------
#define CDC_IDX_RS485      0
#define CDC_IDX_MOTOR      1  // Unified motor bus (Feetech/Dynamixel)
#define CDC_IDX_UART0      2
#define CDC_IDX_UART1      3
#ifdef ENABLE_UART2
#define CDC_IDX_UART2      4
#endif

//--------------------------------------------------------------------
// String descriptor indices
//--------------------------------------------------------------------
#define STR_IDX_LANG       0
#define STR_IDX_MANUF      1
#define STR_IDX_PRODUCT    2
#define STR_IDX_SERIAL     3
#define STR_IDX_CDC0       4   // RS485
#define STR_IDX_CDC1       5   // Motor bus (unified)
#define STR_IDX_CDC2       6   // UART0
#define STR_IDX_CDC3       7   // UART1
#ifdef ENABLE_UART2
#define STR_IDX_CDC4       8   // UART2
#define STR_IDX_CAN        9
#define STR_IDX_I2C        10
#else
#define STR_IDX_CAN        8
#define STR_IDX_I2C        9
#endif

#endif // USB_DESCRIPTORS_H
