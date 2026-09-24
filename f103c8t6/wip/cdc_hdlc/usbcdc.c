// PA11 = USB_DM
// PA12 = USB_DP

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "usbcdc.h"


/*********************************************************************
 * Configuration
 *********************************************************************/

#define USB_TX_QUEUE_SIZE    128
#define USB_RX_QUEUE_SIZE    128

#define USB_EP_RX            0x01
#define USB_EP_TX            0x82
#define USB_EP_NOTIFY        0x83

#define USB_PACKET_SIZE      64


/*********************************************************************
 * State
 *********************************************************************/

static volatile bool initialized = false;

static QueueHandle_t usb_txq = NULL;
static QueueHandle_t usb_rxq = NULL;


/*********************************************************************
 * USB device descriptor
 *********************************************************************/

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,

    .bcdUSB = 0x0200,

    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,

    .bMaxPacketSize0 = 64,

    .idVendor = 0x0483,
    .idProduct = 0x5740,

    .bcdDevice = 0x0200,

    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,

    .bNumConfigurations = 1,
};


/*********************************************************************
 * CDC notification endpoint
 *********************************************************************/

static const struct usb_endpoint_descriptor comm_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,

        .bEndpointAddress = USB_EP_NOTIFY,

        .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,

        .wMaxPacketSize = 16,
        .bInterval = 255,
    }
};


/*********************************************************************
 * CDC data endpoints
 *********************************************************************/

static const struct usb_endpoint_descriptor data_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,

        .bEndpointAddress = USB_EP_RX,

        .bmAttributes = USB_ENDPOINT_ATTR_BULK,

        .wMaxPacketSize = USB_PACKET_SIZE,
        .bInterval = 1,
    },

    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,

        .bEndpointAddress = USB_EP_TX,

        .bmAttributes = USB_ENDPOINT_ATTR_BULK,

        .wMaxPacketSize = USB_PACKET_SIZE,
        .bInterval = 1,
    }
};


/*********************************************************************
 * CDC functional descriptors
 *********************************************************************/

static const struct {
    struct usb_cdc_header_descriptor header;

    struct usb_cdc_call_management_descriptor call_mgmt;

    struct usb_cdc_acm_descriptor acm;

    struct usb_cdc_union_descriptor cdc_union;

} __attribute__((packed)) cdcacm_functional_descriptors = {

    .header = {
        .bFunctionLength =
            sizeof(struct usb_cdc_header_descriptor),

        .bDescriptorType = CS_INTERFACE,

        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,

        .bcdCDC = 0x0110,
    },

    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),

        .bDescriptorType = CS_INTERFACE,

        .bDescriptorSubtype =
            USB_CDC_TYPE_CALL_MANAGEMENT,

        .bmCapabilities = 0,

        .bDataInterface = 1,
    },

    .acm = {
        .bFunctionLength =
            sizeof(struct usb_cdc_acm_descriptor),

        .bDescriptorType = CS_INTERFACE,

        .bDescriptorSubtype = USB_CDC_TYPE_ACM,

        .bmCapabilities = 0,
    },

    .cdc_union = {
        .bFunctionLength =
            sizeof(struct usb_cdc_union_descriptor),

        .bDescriptorType = CS_INTERFACE,

        .bDescriptorSubtype = USB_CDC_TYPE_UNION,

        .bControlInterface = 0,

        .bSubordinateInterface0 = 1,
    }
};


/*********************************************************************
 * Interfaces
 *********************************************************************/

static const struct usb_interface_descriptor comm_iface[] = {
    {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,

        .bInterfaceNumber = 0,

        .bAlternateSetting = 0,

        .bNumEndpoints = 1,

        .bInterfaceClass = USB_CLASS_CDC,

        .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,

        .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,

        .iInterface = 0,

        .endpoint = comm_endp,

        .extra = &cdcacm_functional_descriptors,

        .extralen =
            sizeof(cdcacm_functional_descriptors),
    }
};


static const struct usb_interface_descriptor data_iface[] = {
    {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,

        .bInterfaceNumber = 1,

        .bAlternateSetting = 0,

        .bNumEndpoints = 2,

        .bInterfaceClass = USB_CLASS_DATA,

        .bInterfaceSubClass = 0,

        .bInterfaceProtocol = 0,

        .iInterface = 0,

        .endpoint = data_endp,
    }
};


static const struct usb_interface ifaces[] = {
    {
        .num_altsetting = 1,
        .altsetting = comm_iface,
    },

    {
        .num_altsetting = 1,
        .altsetting = data_iface,
    }
};


/*********************************************************************
 * USB configuration
 *********************************************************************/

static const struct usb_config_descriptor config = {

    .bLength = USB_DT_CONFIGURATION_SIZE,

    .bDescriptorType = USB_DT_CONFIGURATION,

    .wTotalLength = 0,

    .bNumInterfaces = 2,

    .bConfigurationValue = 1,

    .iConfiguration = 0,

    .bmAttributes = 0x80,

    .bMaxPower = 0x32,

    .interface = ifaces,
};


/*********************************************************************
 * USB strings
 *********************************************************************/

static const char *usb_strings[] = {
    "usbcdc.c driver",
    "usbcdc module",
    "usbcdcdemo",
};


/*********************************************************************
 * Control buffer
 *********************************************************************/

static uint8_t usbd_control_buffer[128];


/*********************************************************************
 * CDC control request
 *********************************************************************/

static enum usbd_request_return_codes
cdcacm_control_request(
    usbd_device *usbd_dev __attribute__((unused)),

    struct usb_setup_data *req,

    uint8_t **buf __attribute__((unused)),

    uint16_t *len,

    void (**complete)(
        usbd_device *usbd_dev,
        struct usb_setup_data *req
    ) __attribute__((unused))
)
{
    switch (req->bRequest) {

    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:

        /*
         * Linux cdc_acm expects this request to work.
         */
        return USBD_REQ_HANDLED;


    case USB_CDC_REQ_SET_LINE_CODING:

        if (*len <
            sizeof(struct usb_cdc_line_coding)) {

            return USBD_REQ_NOTSUPP;
        }

        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}


/*********************************************************************
 * USB RX callback
 *********************************************************************/

static void cdcacm_data_rx_cb(
    usbd_device *usbd_dev,

    uint8_t ep __attribute__((unused))
)
{
    uint8_t buf[USB_PACKET_SIZE];

    int len;

    /*
     * Read the USB packet.
     */
    len = usbd_ep_read_packet(
        usbd_dev,
        USB_EP_RX,
        buf,
        sizeof(buf)
    );

    if (len <= 0)
        return;


    /*
     * Put every received byte into the FreeRTOS RX queue.
     *
     * timeout = 0 because this is called from the USB
     * polling/driver context and must never block.
     */
    for (int i = 0; i < len; ++i) {

        if (xQueueSend(
                usb_rxq,
                &buf[i],
                0
            ) != pdPASS) {

            /*
             * RX queue full.
             *
             * Drop remaining bytes.
             */
            break;
        }
    }
}


/*********************************************************************
 * USB configuration callback
 *********************************************************************/

static void cdcacm_set_config(
    usbd_device *usbd_dev,

    uint16_t wValue __attribute__((unused))
)
{
    /*
     * RX endpoint.
     */
    usbd_ep_setup(
        usbd_dev,
        USB_EP_RX,
        USB_ENDPOINT_ATTR_BULK,
        USB_PACKET_SIZE,
        cdcacm_data_rx_cb
    );


    /*
     * TX endpoint.
     */
    usbd_ep_setup(
        usbd_dev,
        USB_EP_TX,
        USB_ENDPOINT_ATTR_BULK,
        USB_PACKET_SIZE,
        NULL
    );


    /*
     * Notification endpoint.
     */
    usbd_ep_setup(
        usbd_dev,
        USB_EP_NOTIFY,
        USB_ENDPOINT_ATTR_INTERRUPT,
        16,
        NULL
    );


    /*
     * CDC class requests.
     */
    usbd_register_control_callback(
        usbd_dev,

        USB_REQ_TYPE_CLASS |
        USB_REQ_TYPE_INTERFACE,

        USB_REQ_TYPE_TYPE |
        USB_REQ_TYPE_RECIPIENT,

        cdcacm_control_request
    );


    /*
     * Host has configured the CDC device.
     */
    initialized = true;
}


/*********************************************************************
 * USB driver task
 *********************************************************************/

static void usb_task(void *arg)
{
    usbd_device *udev = (usbd_device *)arg;

    uint8_t txbuf[USB_PACKET_SIZE];

    unsigned txlen = 0;


    for (;;) {

        /*
         * This is what services USB RX/TX.
         */
        usbd_poll(udev);


        if (!initialized) {

            vTaskDelay(pdMS_TO_TICKS(1));

            continue;
        }


        /*
         * Fill a USB packet from the TX queue.
         */
        while (txlen < sizeof(txbuf)) {

            if (xQueueReceive(
                    usb_txq,
                    &txbuf[txlen],
                    0
                ) != pdPASS) {

                break;
            }

            ++txlen;
        }


        /*
         * Send packet if we have anything.
         */
        if (txlen > 0) {

            int sent = usbd_ep_write_packet(
                udev,
                USB_EP_TX,
                txbuf,
                txlen
            );

            if (sent > 0) {

                txlen = 0;
            }
        }


        /*
         * Do not let this task consume 100% CPU.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/*********************************************************************
 * USB test task
 *
 * This is deliberately included here so you can test USB CDC
 * independently of the bridge code.
 *********************************************************************/

static void usb_test_task(void *arg)
{
    (void)arg;


    /*
     * Wait until Linux has configured the CDC device.
     */
    while (!usb_ready()) {

        vTaskDelay(pdMS_TO_TICKS(10));
    }


    usb_puts(
        "\r\n"
        "USB CDC TEST TASK STARTED\r\n"
        "Type something and it will be echoed.\r\n"
    );


    for (;;) {

        /*
         * BLOCK here until a character arrives.
         *
         * This task consumes essentially zero CPU while idle.
         */
        int c = usb_getc();

        if (c >= 0) {

            /*
             * Echo CR/LF nicely.
             */
            if (c == '\r') {

                usb_puts("\r\n");

            } else if (c == '\n') {

                /*
                 * picocom may already send CR/LF depending
                 * on configuration. Don't produce two lines.
                 */
                usb_puts("\r\n");

            } else {

                usb_putc((char)c);
            }
        }
    }
}


/*********************************************************************
 * Start USB
 *********************************************************************/

void usb_start(bool gpio_init, unsigned priority)
{
    usbd_device *udev;


    /*
     * Create queues BEFORE starting USB task.
     */
    usb_txq = xQueueCreate(
        USB_TX_QUEUE_SIZE,
        sizeof(uint8_t)
    );

    usb_rxq = xQueueCreate(
        USB_RX_QUEUE_SIZE,
        sizeof(uint8_t)
    );


    if (usb_txq == NULL || usb_rxq == NULL) {

        /*
         * Queue creation failed.
         *
         * Don't continue with a broken USB driver.
         */
        for (;;)
            taskYIELD();
    }


    /*
     * USB GPIO/peripheral clocks.
     */
    if (gpio_init) {

        rcc_periph_clock_enable(RCC_GPIOA);

        rcc_periph_clock_enable(RCC_USB);
    }


    /*
     * Initialize USB peripheral.
     */
    udev = usbd_init(
        &st_usbfs_v1_usb_driver,

        &dev,

        &config,

        usb_strings,

        3,

        usbd_control_buffer,

        sizeof(usbd_control_buffer)
    );


    /*
     * Register configuration callback.
     */
    usbd_register_set_config_callback(
        udev,
        cdcacm_set_config
    );


    /*
     * Start USB polling task.
     */
    if (xTaskCreate(
            usb_task,
            "USB",
            300,
            udev,
            priority,
            NULL
        ) != pdPASS) {

        for (;;)
            taskYIELD();
    }


    /*
     * TEST ONLY:
     *
     * Start a separate task which simply echoes USB input.
     *
     * Once USB is confirmed working, remove/comment this.
     */
    if (xTaskCreate(
            usb_test_task,
            "USBTEST",
            300,
            NULL,
            priority - 1,
            NULL
        ) != pdPASS) {

        for (;;)
            taskYIELD();
    }
}


/*********************************************************************
 * USB status
 *********************************************************************/

bool usb_ready(void)
{
    return initialized;
}


/*********************************************************************
 * USB RX
 *********************************************************************/

int usb_peek(void)
{
    uint8_t ch;

    if (xQueuePeek(
            usb_rxq,
            &ch,
            0
        ) == pdPASS) {

        return 1;
    }

    return 0;
}


int usb_getc(void)
{
    uint8_t ch;

    if (xQueueReceive(
            usb_rxq,
            &ch,
            portMAX_DELAY
        ) != pdPASS) {

        return -1;
    }

    return (int)ch;
}


int usb_getc_nb(void)
{
    uint8_t ch;

    if (xQueueReceive(
            usb_rxq,
            &ch,
            0
        ) != pdPASS) {

        return -1;
    }

    return (int)ch;
}


/*********************************************************************
 * USB TX
 *********************************************************************/

void usb_write(
    const char *buf,
    unsigned bytes
)
{
    while (bytes--) {

        /*
         * Blocks if TX queue is full.
         */
        xQueueSend(
            usb_txq,
            buf,
            portMAX_DELAY
        );

        ++buf;
    }
}


bool usb_write_nb(
    const char *buf,
    unsigned bytes
)
{
    while (bytes--) {

        if (xQueueSend(
                usb_txq,
                buf,
                0
            ) != pdPASS) {

            return false;
        }

        ++buf;
    }

    return true;
}


void usb_puts(const char *buf)
{
    while (*buf)
        usb_putc(*buf++);
}


void usb_putc(char ch)
{
    static const char cr = '\r';


    /*
     * Wait until Linux has configured the CDC device.
     */
    while (!usb_ready())
        taskYIELD();


    /*
     * Convert LF to CRLF.
     */
    if (ch == '\n') {

        xQueueSend(
            usb_txq,
            &cr,
            portMAX_DELAY
        );
    }


    xQueueSend(
        usb_txq,
        &ch,
        portMAX_DELAY
    );
}


bool usb_putc_nb(char ch)
{
    static const char cr = '\r';


    if (!usb_ready())
        return false;


    if (ch == '\n') {

        if (xQueueSend(
                usb_txq,
                &cr,
                0
            ) != pdPASS) {

            return false;
        }
    }


    return xQueueSend(
        usb_txq,
        &ch,
        0
    ) == pdPASS;
}


/*********************************************************************
 * Debug formatting helpers
 *********************************************************************/

void usb_puthex32(uint32_t v)
{
    static const char hex[] =
        "0123456789abcdef";


    for (int shift = 28;
         shift >= 0;
         shift -= 4) {

        usb_putc(
            hex[(v >> shift) & 0x0f]
        );
    }
}


void usb_putdec(uint32_t v)
{
    char buf[10];

    int i = 0;


    if (v == 0) {

        usb_putc('0');

        return;
    }


    while (v) {

        buf[i++] =
            '0' + (v % 10);

        v /= 10;
    }


    while (i--)
        usb_putc(buf[i]);
}

