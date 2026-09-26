#include "tusb.h"

#define USB_VID   0xCafe
#define USB_PID   0x4001

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

#if 0
static const uint8_t desc_device[] =
{
    TUD_DEVICE_DESCRIPTOR(
        0x0200,
        TUSB_CLASS_MISC,
        MISC_SUBCLASS_COMMON,
        MISC_PROTOCOL_IAD,
        USB_VID,
        USB_PID,
        0x0100,
        0x01,
        0x02,
        0x03,
        0x01
    )
};
#endif
#if 1
static const uint8_t desc_device[] =
{
    18,                     // bLength
    TUSB_DESC_DEVICE,       // bDescriptorType
    0x00, 0x02,             // bcdUSB = 2.00

    TUSB_CLASS_MISC,        // bDeviceClass
    MISC_SUBCLASS_COMMON,   // bDeviceSubClass
    MISC_PROTOCOL_IAD,      // bDeviceProtocol

    64,                     // bMaxPacketSize0

    (uint8_t)(USB_VID & 0xff),
    (uint8_t)(USB_VID >> 8),

    (uint8_t)(USB_PID & 0xff),
    (uint8_t)(USB_PID >> 8),

    0x00, 0x01,             // bcdDevice = 1.00

    0x01,                   // iManufacturer
    0x02,                   // iProduct
    0x03,                   // iSerialNumber

    0x01                    // bNumConfigurations
};
#endif


static const uint8_t desc_configuration[] =
{
    TUD_CONFIG_DESCRIPTOR(
        1,
        ITF_NUM_TOTAL,
        0,
        CONFIG_TOTAL_LEN,
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
        100
    ),

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC,
        4,
        EPNUM_CDC_NOTIF,
        8,
        EPNUM_CDC_OUT,
        EPNUM_CDC_IN,
        64
    )
};

static const char *string_desc[] =
{
    (const char[]) { 0x09, 0x04 },
    "Blue Pill",
    "STM32F103 USB CDC",
    "0001",
    "CDC",
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    static uint16_t desc_str[32];
    (void) langid;

    uint8_t chr_count;

    if (index == 0)
    {
        desc_str[1] = 0x0409;
        chr_count = 1;
    }
    else
    {
        if (index >= sizeof(string_desc) / sizeof(string_desc[0]))
            return NULL;

        const char *str = string_desc[index];

        chr_count = 0;
        while (str[chr_count] && chr_count < 31)
            desc_str[1 + chr_count] = str[chr_count], chr_count++;
    }

    desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return desc_str;
}

