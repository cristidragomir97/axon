#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "pico.h"

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_DEBUG        0

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENABLED       1
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE 64

//--------------------------------------------------------------------
// CLASS CONFIGURATION
//--------------------------------------------------------------------

// 5 CDC interfaces: RS485, Feetech, Dynamixel, UART0, UART1
#define CFG_TUD_CDC           5
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

// 2 VENDOR interfaces: CAN (IF6) + I2C (IF7)
// I2C only uses EP0 for data, but TinyUSB requires each vendor
// interface to have bulk endpoints or it STALLs set_configuration.
// The I2C bulk endpoints are allocated but never used.
#define CFG_TUD_VENDOR        2
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 64

#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0
#define CFG_TUD_MSC           0

#endif
