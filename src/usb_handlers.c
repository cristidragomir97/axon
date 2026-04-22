#include "usb_handlers.h"
#include "led_dual_core.h"
#include "pins.h"
#include "bus/can.h"
#include "protocol/gs_usb.h"
#include "protocol/i2c_tiny_usb.h"
#include "usb_descriptors.h"

#include "tusb.h"
#include "hardware/i2c.h"
#include <string.h>

//--------------------------------------------------------------------
// I2C Configuration
//--------------------------------------------------------------------
#define I2C_PORT     i2c1
#define I2C_TIMEOUT_US 50000

//--------------------------------------------------------------------
// State
//--------------------------------------------------------------------
static uint8_t i2c_status = I2C_TINY_STATUS_IDLE;
static uint8_t ctrl_buf[64];  // Buffer for control transfers

// gs_usb device info
static struct gs_device_config gs_config = {
    .reserved1  = 0,
    .reserved2  = 0, 
    .reserved3  = 0,
    .icount     = 0,  // Driver adds 1, so 0 becomes 1 interface
    .sw_version = 2,
    .hw_version = 1,
};

static struct gs_device_bt_const gs_bt_const = {
    .feature   = GS_CAN_FEATURE_LISTEN_ONLY | GS_CAN_FEATURE_LOOP_BACK,
    .fclk_can  = 40000000,
    .tseg1_min = 1,
    .tseg1_max = 256,
    .tseg2_min = 1, 
    .tseg2_max = 128,
    .sjw_max   = 128,
    .brp_min   = 1,
    .brp_max   = 512,
    .brp_inc   = 1,
};

// CAN frame buffers
static struct gs_host_frame gs_tx_frame;
static struct gs_host_frame gs_rx_frame;

//--------------------------------------------------------------------
// i2c-tiny-usb Protocol Handler
//--------------------------------------------------------------------
static bool handle_i2c_request(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    uint8_t cmd = request->bRequest;
    uint8_t direction = request->bmRequestType_bit.direction;

    // I2C I/O commands (0x04-0x07)
    bool is_i2c_io = (cmd >= I2C_TINY_CMD_I2C_IO &&
                      cmd <= (I2C_TINY_CMD_I2C_IO | I2C_TINY_CMD_I2C_IO_BEGIN | I2C_TINY_CMD_I2C_IO_END));

    if (is_i2c_io) {
        uint8_t flags = cmd & 0x03;
        uint8_t addr = request->wIndex & 0x7F;
        bool nostop = !(flags & I2C_TINY_CMD_I2C_IO_END);
        bool is_read = (direction == TUSB_DIR_IN);

        if (is_read) {
            if (stage == CONTROL_STAGE_SETUP) {
                int ret = i2c_read_timeout_us(I2C_PORT, addr, ctrl_buf,
                                              request->wLength, nostop, I2C_TIMEOUT_US);
                i2c_status = (ret >= 0) ? I2C_TINY_STATUS_ACK : I2C_TINY_STATUS_NAK;
                led_activity(LED_I2C);
                return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
            }
        } else {
            if (stage == CONTROL_STAGE_SETUP) {
                return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
            }
            if (stage == CONTROL_STAGE_DATA) {
                int ret = i2c_write_timeout_us(I2C_PORT, addr, ctrl_buf,
                                               request->wLength, nostop, I2C_TIMEOUT_US);
                i2c_status = (ret >= 0) ? I2C_TINY_STATUS_ACK : I2C_TINY_STATUS_NAK;
                led_activity(LED_I2C);
            }
        }
        return true;
    }

    // Other i2c-tiny-usb commands
    switch (cmd) {
    case I2C_TINY_CMD_GET_FUNC:
        if (stage == CONTROL_STAGE_SETUP) {
            uint32_t func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
            memcpy(ctrl_buf, &func, sizeof(func));
            return tud_control_xfer(rhport, request, ctrl_buf, sizeof(func));
        }
        return true;

    case I2C_TINY_CMD_GET_STATUS:
        if (stage == CONTROL_STAGE_SETUP) {
            ctrl_buf[0] = i2c_status;
            return tud_control_xfer(rhport, request, ctrl_buf, 1);
        }
        return true;

    case I2C_TINY_CMD_SET_DELAY:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_status(rhport, request);
        }
        return true;

    case I2C_TINY_CMD_ECHO:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
        }
        return true;
    }

    return false;
}

//--------------------------------------------------------------------
// gs_usb Protocol Handler (CAN)
//--------------------------------------------------------------------
static bool handle_gs_usb_request(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request)
{
    switch (request->bRequest) {
    case GS_USB_BREQ_DEVICE_CONFIG:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, &gs_config, sizeof(gs_config));
        }
        return true;

    case GS_USB_BREQ_BT_CONST:
        if (stage == CONTROL_STAGE_SETUP) {
            static struct gs_device_bt_const bt_const = {
                .feature   = 0,  // No features like the working example
                .fclk_can  = 40000000,
                .tseg1_min = 1,
                .tseg1_max = 8,  // Match working example
                .tseg2_min = 1,
                .tseg2_max = 8,  // Match working example  
                .sjw_max   = 4,  // Match working example
                .brp_min   = 2,  // Match working example
                .brp_max   = 64, // Match working example
                .brp_inc   = 2,  // Match working example
            };
            return tud_control_xfer(rhport, request, &bt_const, sizeof(bt_const));
        }
        return true;

    case GS_USB_BREQ_HOST_FORMAT:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
        }
        return true;

    case GS_USB_BREQ_BITTIMING:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
        }
        // Could configure CAN timing here if needed
        return true;

    case GS_USB_BREQ_MODE:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_xfer(rhport, request, ctrl_buf, request->wLength);
        }
        if (stage == CONTROL_STAGE_DATA) {
            struct gs_device_mode *mode = (struct gs_device_mode *)ctrl_buf;
            if (mode->mode == GS_CAN_MODE_START) {
                can_start();
            } else {
                can_stop();
            }
        }
        return true;

    case GS_USB_BREQ_IDENTIFY:
        if (stage == CONTROL_STAGE_SETUP) {
            return tud_control_status(rhport, request);
        }
        return true;
    }

    return false;
}

//--------------------------------------------------------------------
// TinyUSB Vendor Control Callback
//--------------------------------------------------------------------
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request)
{
    uint8_t recipient = request->bmRequestType_bit.recipient;

    // DEVICE-level requests -> i2c-tiny-usb
    if (recipient == TUSB_REQ_RCPT_DEVICE) {
        return handle_i2c_request(rhport, stage, request);
    }

    // INTERFACE-level requests to CAN interface -> gs_usb
    if (recipient == TUSB_REQ_RCPT_INTERFACE && request->wIndex == ITF_NUM_CAN) {
        return handle_gs_usb_request(rhport, stage, request);
    }

    return false;
}

//--------------------------------------------------------------------
// Public API
//--------------------------------------------------------------------

void usb_handlers_init(void) {
    i2c_status = I2C_TINY_STATUS_IDLE;
}

void can_usb_task(void) {
    // USB -> CAN
    while (tud_vendor_n_available(0) >= sizeof(gs_tx_frame)) {
        uint32_t count = tud_vendor_n_read(0, &gs_tx_frame, sizeof(gs_tx_frame));
        if (count == sizeof(gs_tx_frame)) {
            can_frame_t frame = {
                .can_id = gs_tx_frame.can_id,
                .dlc = gs_tx_frame.can_dlc,
                .flags = gs_tx_frame.flags,
            };
            memcpy(frame.data, gs_tx_frame.data, 8);

            if (can_send(&frame)) {
                // Echo back for TX confirmation
                if (tud_vendor_n_write_available(0) >= sizeof(gs_tx_frame)) {
                    tud_vendor_n_write(0, &gs_tx_frame, sizeof(gs_tx_frame));
                    tud_vendor_n_flush(0);
                }
            }
        }
    }

    // CAN -> USB
    can_frame_t rx_frame;
    while (can_recv(&rx_frame)) {
        if (tud_vendor_n_write_available(0) >= sizeof(gs_rx_frame)) {
            memset(&gs_rx_frame, 0, sizeof(gs_rx_frame));
            gs_rx_frame.echo_id = 0xFFFFFFFF;  // RX frame marker
            gs_rx_frame.can_id = rx_frame.can_id;
            gs_rx_frame.can_dlc = rx_frame.dlc;
            gs_rx_frame.channel = 0;
            gs_rx_frame.flags = rx_frame.flags;
            memcpy(gs_rx_frame.data, rx_frame.data, 8);

            tud_vendor_n_write(0, &gs_rx_frame, sizeof(gs_rx_frame));
            tud_vendor_n_flush(0);
        } else {
            break;
        }
    }
}
