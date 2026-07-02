#include "tusb.h"
#include "pico/unique_id.h"
#include "usb_descriptors.h"

//--------------------------------------------------------------------
// Device Descriptor
//--------------------------------------------------------------------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x1209,
    .idProduct          = 0xAC01,
    .bcdDevice          = 0x0200,

    .iManufacturer      = STR_IDX_MANUF,
    .iProduct           = STR_IDX_PRODUCT,
    .iSerialNumber      = STR_IDX_SERIAL,

    .bNumConfigurations = 1
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------
// Configuration Descriptor
//--------------------------------------------------------------------

#define TUD_VENDOR_DESCRIPTOR_EX(_itfnum, _stridx, _epout, _epin, _epsize, _subclass, _protocol) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, _subclass, _protocol, _stridx, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TUD_VENDOR_DESC_EX_LEN TUD_VENDOR_DESC_LEN

#ifdef ENABLE_UART2
#define CONFIG_TOTAL_LEN ( \
    TUD_CONFIG_DESC_LEN + \
    TUD_CDC_DESC_LEN * 5 + \
    TUD_VENDOR_DESC_LEN + \
    TUD_VENDOR_DESC_EX_LEN \
)
#else
#define CONFIG_TOTAL_LEN ( \
    TUD_CONFIG_DESC_LEN + \
    TUD_CDC_DESC_LEN * 4 + \
    TUD_VENDOR_DESC_LEN + \
    TUD_VENDOR_DESC_EX_LEN \
)
#endif

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500),

    // CDC #0: RS485
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0_COMM, STR_IDX_CDC0,
                       EP_CDC0_NOTIF, 8, EP_CDC0_OUT, EP_CDC0_IN, 64),

    // CDC #1: Motor Bus (unified Feetech/Dynamixel)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1_COMM, STR_IDX_CDC1,
                       EP_CDC1_NOTIF, 8, EP_CDC1_OUT, EP_CDC1_IN, 64),

    // CDC #2: UART0 (DDSM)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC2_COMM, STR_IDX_CDC2,
                       EP_CDC2_NOTIF, 8, EP_CDC2_OUT, EP_CDC2_IN, 64),

    // CDC #3: UART1 (Lidar)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC3_COMM, STR_IDX_CDC3,
                       EP_CDC3_NOTIF, 8, EP_CDC3_OUT, EP_CDC3_IN, 64),

#ifdef ENABLE_UART2
    // CDC #4: UART2 (Additional)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC4_COMM, STR_IDX_CDC4,
                       EP_CDC4_NOTIF, 8, EP_CDC4_OUT, EP_CDC4_IN, 64),
#endif

    // Vendor: CAN (gs_usb)
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_CAN, STR_IDX_CAN,
                          EP_CAN_OUT, EP_CAN_IN, 64),

    // Vendor: I2C (i2c-tiny-usb, subclass=1 proto=1)
    TUD_VENDOR_DESCRIPTOR_EX(ITF_NUM_I2C, STR_IDX_I2C,
                             EP_I2C_OUT, EP_I2C_IN, 64, 0x01, 0x01),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

//--------------------------------------------------------------------
// String Descriptors
//--------------------------------------------------------------------
static char const *string_desc_arr[] = {
    [STR_IDX_LANG]    = (const char[]){0x09, 0x04},
    [STR_IDX_MANUF]   = "RoboCore",
    [STR_IDX_PRODUCT] = "Link101 Multiprotocol Bridge",
    [STR_IDX_SERIAL]  = NULL,
    [STR_IDX_CDC0]    = "RoboCore Link101 RS485",
    [STR_IDX_CDC1]    = "RoboCore Link101 Motor",
    [STR_IDX_CDC2]    = "RoboCore Link101 UART0",
    [STR_IDX_CDC3]    = "RoboCore Link101 UART1",
#ifdef ENABLE_UART2
    [STR_IDX_CDC4]    = "RoboCore Link101 UART2",
#endif
    [STR_IDX_CAN]     = "RoboCore Link101 CAN",
    [STR_IDX_I2C]     = "RoboCore Link101 I2C",
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == STR_IDX_LANG) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == STR_IDX_SERIAL) {
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);
        chr_count = 0;
        for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES && chr_count < 16; i++) {
            const char hex[] = "0123456789ABCDEF";
            _desc_str[1 + chr_count++] = hex[(id.id[i] >> 4) & 0xF];
            _desc_str[1 + chr_count++] = hex[id.id[i] & 0xF];
        }
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        if (!str) return NULL;
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
