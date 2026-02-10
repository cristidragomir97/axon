/**
 * @file usb_descriptors.c
 * @brief USB descriptors for dual CDC composite device
 */

#include "tusb.h"
#include "config.h"
#include "pinout.h"

/* ──────────────────────────────────────────────
 *  String descriptors
 * ────────────────────────────────────────────── */
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_ZENOH,
#ifdef ENABLE_LIDAR_BRIDGE
    STRID_CDC_LIDAR,
#endif
};

static char const *string_desc_arr[] = {
    [STRID_LANGID]       = (const char[]){0x09, 0x04},  /* English (US) */
    [STRID_MANUFACTURER] = "RoboCore",
    [STRID_PRODUCT]      = "Axon",
    [STRID_SERIAL]       = "000001",
    [STRID_CDC_ZENOH]    = "Zenoh Transport",
#ifdef ENABLE_LIDAR_BRIDGE
    [STRID_CDC_LIDAR]    = "Lidar Bridge",
#endif
};

/* ──────────────────────────────────────────────
 *  Device descriptor
 * ────────────────────────────────────────────── */
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* ──────────────────────────────────────────────
 *  Configuration descriptor
 * ────────────────────────────────────────────── */
#ifdef ENABLE_LIDAR_BRIDGE
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)
#else
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#endif

/* Endpoint numbers */
#define EPNUM_CDC0_NOTIF    0x81
#define EPNUM_CDC0_OUT      0x02
#define EPNUM_CDC0_IN       0x82

#ifdef ENABLE_LIDAR_BRIDGE
#define EPNUM_CDC1_NOTIF    0x83
#define EPNUM_CDC1_OUT      0x04
#define EPNUM_CDC1_IN       0x84
#endif

static uint8_t const desc_configuration[] = {
    /* Configuration descriptor */
    TUD_CONFIG_DESCRIPTOR(1, CFG_TUD_CDC * 2, 0, CONFIG_TOTAL_LEN,
                          0x00, 100),

    /* CDC0: Zenoh transport */
    TUD_CDC_DESCRIPTOR(0, STRID_CDC_ZENOH, EPNUM_CDC0_NOTIF,
                       8, EPNUM_CDC0_OUT, EPNUM_CDC0_IN, 64),

#ifdef ENABLE_LIDAR_BRIDGE
    /* CDC1: Lidar bridge */
    TUD_CDC_DESCRIPTOR(2, STRID_CDC_LIDAR, EPNUM_CDC1_NOTIF,
                       8, EPNUM_CDC1_OUT, EPNUM_CDC1_IN, 64),
#endif
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* ──────────────────────────────────────────────
 *  String descriptor callback
 * ────────────────────────────────────────────── */
static uint16_t desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = str[i];
        }
    }

    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return desc_str;
}
