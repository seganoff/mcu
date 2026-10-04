#include "tusb.h"

/*------------------------------------------------------------------*/
/* Device descriptor                                                */
/*------------------------------------------------------------------*/

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = 0x4001,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

/*------------------------------------------------------------------*/
/* Configuration descriptor                                         */
/*------------------------------------------------------------------*/

#define CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

#define EPNUM_CDC_NOTIF  0x81
#define EPNUM_CDC_OUT    0x02
#define EPNUM_CDC_IN     0x82

#define CDC_NOTIF_SIZE   8
#define CDC_EP_SIZE      64

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(
        1,                  // configuration number
        2,                  // interface count
        0,                  // string index
        CONFIG_TOTAL_LEN,   // total length
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
        100                 // 100 mA
    ),

    TUD_CDC_DESCRIPTOR(
        0,                  // interface number
        4,                  // string index
        EPNUM_CDC_NOTIF,
        CDC_NOTIF_SIZE,
        EPNUM_CDC_OUT,
        EPNUM_CDC_IN,
        CDC_EP_SIZE
    )
};

/*------------------------------------------------------------------*/
/* String descriptors                                               */
/*------------------------------------------------------------------*/

char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: English (0x0409)
    "TinyUSB",                       // 1: Manufacturer
    "STM32F405 CDC",                 // 2: Product
    "00000001",                      // 3: Serial
    "CDC",                           // 4: CDC interface
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = 0x0409;
        _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
        return _desc_str;
    }

    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        return NULL;
    }

    const char *str = string_desc_arr[index];

    chr_count = 0;

    while (str[chr_count] != '\0' &&
           chr_count < (sizeof(_desc_str) / sizeof(_desc_str[0])) - 1) {
        _desc_str[1 + chr_count] = str[chr_count];
        chr_count++;
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}

/*------------------------------------------------------------------*/
/* TinyUSB descriptor callbacks                                     */
/*------------------------------------------------------------------*/

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;

    return desc_configuration;
}

