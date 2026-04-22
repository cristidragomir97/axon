#ifndef PROTOCOL_I2C_TINY_USB_H
#define PROTOCOL_I2C_TINY_USB_H

#include <stdint.h>

// i2c-tiny-usb protocol - Linux kernel i2c-tiny-usb driver compatible
// Reference: https://github.com/harbaum/I2C-Tiny-USB

//--------------------------------------------------------------------
// USB Control Request Commands (bRequest)
//--------------------------------------------------------------------
#define I2C_TINY_CMD_ECHO           0x00  // Echo test
#define I2C_TINY_CMD_GET_FUNC       0x01  // Get I2C functionality
#define I2C_TINY_CMD_SET_DELAY      0x02  // Set I2C delay
#define I2C_TINY_CMD_GET_STATUS     0x03  // Get transaction status

// I2C_IO commands (0x04-0x07 depending on flags)
#define I2C_TINY_CMD_I2C_IO         0x04
#define I2C_TINY_CMD_I2C_IO_BEGIN   (1 << 0)  // Generate START
#define I2C_TINY_CMD_I2C_IO_END     (1 << 1)  // Generate STOP

//--------------------------------------------------------------------
// I2C Message Flags (from Linux i2c.h)
//--------------------------------------------------------------------
#define I2C_M_RD                    0x0001  // Read from slave

//--------------------------------------------------------------------
// I2C Functionality Flags (from Linux i2c.h)
//--------------------------------------------------------------------
#define I2C_FUNC_I2C                0x00000001
#define I2C_FUNC_10BIT_ADDR         0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING  0x00000004
#define I2C_FUNC_SMBUS_PEC          0x00000008
#define I2C_FUNC_NOSTART            0x00000010
#define I2C_FUNC_SLAVE              0x00000020
#define I2C_FUNC_SMBUS_QUICK        0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE    0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE   0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA   0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA   0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL    0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK   0x04000000
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000

// Common functionality combination
#define I2C_FUNC_SMBUS_EMUL         0x0EFF0008

//--------------------------------------------------------------------
// Status Codes (returned by GET_STATUS)
//--------------------------------------------------------------------
#define I2C_TINY_STATUS_IDLE        0x00  // Ready for new transaction
#define I2C_TINY_STATUS_ACK         0x01  // Last transaction ACK'd
#define I2C_TINY_STATUS_NAK         0x02  // Last transaction NAK'd

#endif // PROTOCOL_I2C_TINY_USB_H
