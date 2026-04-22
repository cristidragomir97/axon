#ifndef PROTOCOL_GS_USB_H
#define PROTOCOL_GS_USB_H

#include <stdint.h>

// gs_usb protocol - Linux SocketCAN compatible USB-CAN interface
// Reference: https://github.com/linux-can/can-utils

//--------------------------------------------------------------------
// Control Request Commands
//--------------------------------------------------------------------
#define GS_USB_BREQ_HOST_FORMAT     0
#define GS_USB_BREQ_BITTIMING       1
#define GS_USB_BREQ_MODE            2
#define GS_USB_BREQ_BERR            3
#define GS_USB_BREQ_BT_CONST        4
#define GS_USB_BREQ_DEVICE_CONFIG   5
#define GS_USB_BREQ_TIMESTAMP       6
#define GS_USB_BREQ_IDENTIFY        7

//--------------------------------------------------------------------
// Mode Flags
//--------------------------------------------------------------------
#define GS_CAN_MODE_RESET           0
#define GS_CAN_MODE_START           1

#define GS_CAN_MODE_NORMAL          0
#define GS_CAN_MODE_LISTEN_ONLY     (1 << 0)
#define GS_CAN_MODE_LOOP_BACK       (1 << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE   (1 << 2)
#define GS_CAN_MODE_ONE_SHOT        (1 << 3)
#define GS_CAN_MODE_HW_TIMESTAMP    (1 << 4)

//--------------------------------------------------------------------
// Feature Flags (returned in bt_const)
//--------------------------------------------------------------------
#define GS_CAN_FEATURE_LISTEN_ONLY  (1 << 0)
#define GS_CAN_FEATURE_LOOP_BACK    (1 << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE (1 << 2)
#define GS_CAN_FEATURE_ONE_SHOT     (1 << 3)
#define GS_CAN_FEATURE_HW_TIMESTAMP (1 << 4)
#define GS_CAN_FEATURE_IDENTIFY     (1 << 5)
#define GS_CAN_FEATURE_FD           (1 << 8)

//--------------------------------------------------------------------
// Frame Flags
//--------------------------------------------------------------------
#define GS_CAN_FLAG_OVERFLOW        (1 << 0)
#define GS_CAN_FLAG_FD              (1 << 1)
#define GS_CAN_FLAG_BRS             (1 << 2)
#define GS_CAN_FLAG_ESI             (1 << 3)

//--------------------------------------------------------------------
// Protocol Structures
//--------------------------------------------------------------------

// Host -> Device frame (and echo back)
struct gs_host_frame {
    uint32_t echo_id;       // Echo ID for TX confirmation
    uint32_t can_id;        // CAN ID + flags (EFF/RTR/ERR)
    uint8_t  can_dlc;       // Data length code
    uint8_t  channel;       // CAN channel (0 for single-channel)
    uint8_t  flags;         // Frame flags
    uint8_t  reserved;
    uint8_t  data[8];       // CAN data bytes
} __attribute__((packed));

// Device configuration (response to DEVICE_CONFIG)
struct gs_device_config {
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  reserved3;
    uint8_t  icount;        // Number of CAN interfaces
    uint32_t sw_version;    // Firmware version
    uint32_t hw_version;    // Hardware version
} __attribute__((packed));

// Bit timing constants (response to BT_CONST)
struct gs_device_bt_const {
    uint32_t feature;       // Feature flags
    uint32_t fclk_can;      // CAN clock frequency (Hz)
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} __attribute__((packed));

// Bit timing configuration (from host)
struct gs_device_bittiming {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} __attribute__((packed));

// Mode configuration (from host)
struct gs_device_mode {
    uint32_t mode;          // GS_CAN_MODE_*
    uint32_t flags;         // GS_CAN_MODE_* flags
} __attribute__((packed));

#endif // PROTOCOL_GS_USB_H
