/*
 * giant_main.c
 *
 * STM32F103C8T6
 *
 * Diagnostic stage:
 *
 *     PC -> USB CDC OUT -> RX callback -> debug USART3
 *
 * NO USB echo.
 * NO USB TX.
 * NO USART2.
 *
 * Type characters into /dev/ttyACM1.
 *
 * Expected USART3 debug output:
 *
 *     USB RX: len=3
 *     61 62 63
 *
 * if "abc" was typed.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "FreeRTOS.h"
#include "task.h"


/*********************************************************************
 * DEBUGGING
 *********************************************************************/

#define DEBUGGING 1


/*********************************************************************
 * Debug USART3
 *
 * USART3:
 *   TX = PB10
 *   RX = PB11
 *
 * 115200 8N1
 *********************************************************************/

static void debug_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART3_TX
    );

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_USART3_RX
    );

    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(
        USART3,
        USART_FLOWCONTROL_NONE
    );

    usart_enable(USART3);
}


static void debug_putc(char c)
{
    usart_send_blocking(USART3, c);
}


static void debug_puts(const char *s)
{
    while (*s)
        debug_putc(*s++);
}


static void debug_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";

    debug_putc(hex[(v >> 4) & 0x0F]);
    debug_putc(hex[v & 0x0F]);
}


static void debug_dec(unsigned int v)
{
    char buf[10];
    unsigned int i = 0;

    if (v == 0) {
        debug_putc('0');
        return;
    }

    while (v) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }

    while (i)
        debug_putc(buf[--i]);
}


/*********************************************************************
 * USB CDC state
 *********************************************************************/

static volatile bool usb_initialized = false;

static usbd_device *usb_device = NULL;

/*
 * USB OUT -> this queue.
 *
 * Endpoint 0x01 feeds this queue.
 */
static QueueHandle_t usb_rxq;


/*********************************************************************
 * USB descriptors
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

        .bEndpointAddress = 0x83,

        .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,

        .wMaxPacketSize = 16,

        .bInterval = 255,
    },
};


/*********************************************************************
 * CDC data endpoints
 *
 * OUT = 0x01
 * IN  = 0x82
 *********************************************************************/

static const struct usb_endpoint_descriptor data_endp[] = {

    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,

        .bEndpointAddress = 0x01,

        .bmAttributes = USB_ENDPOINT_ATTR_BULK,

        .wMaxPacketSize = 64,

        .bInterval = 1,
    },

    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,

        .bEndpointAddress = 0x82,

        .bmAttributes = USB_ENDPOINT_ATTR_BULK,

        .wMaxPacketSize = 64,

        .bInterval = 1,
    },
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

        .bDescriptorSubtype =
            USB_CDC_TYPE_HEADER,

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

        .bDescriptorSubtype =
            USB_CDC_TYPE_ACM,

        .bmCapabilities = 0,
    },

    .cdc_union = {

        .bFunctionLength =
            sizeof(struct usb_cdc_union_descriptor),

        .bDescriptorType = CS_INTERFACE,

        .bDescriptorSubtype =
            USB_CDC_TYPE_UNION,

        .bControlInterface = 0,

        .bSubordinateInterface0 = 1,
    },
};


/*********************************************************************
 * CDC communication interface
 *********************************************************************/

static const struct usb_interface_descriptor comm_iface[] = {

    {
        .bLength = USB_DT_INTERFACE_SIZE,

        .bDescriptorType = USB_DT_INTERFACE,

        .bInterfaceNumber = 0,

        .bAlternateSetting = 0,

        .bNumEndpoints = 1,

        .bInterfaceClass = USB_CLASS_CDC,

        .bInterfaceSubClass =
            USB_CDC_SUBCLASS_ACM,

        .bInterfaceProtocol =
            USB_CDC_PROTOCOL_AT,

        .iInterface = 0,

        .endpoint = comm_endp,

        .extra =
            &cdcacm_functional_descriptors,

        .extralen =
            sizeof(cdcacm_functional_descriptors),
    },
};


/*********************************************************************
 * CDC data interface
 *********************************************************************/

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
    },
};


/*********************************************************************
 * Interfaces
 *********************************************************************/

static const struct usb_interface ifaces[] = {

    {
        .num_altsetting = 1,
        .altsetting = comm_iface,
    },

    {
        .num_altsetting = 1,
        .altsetting = data_iface,
    },
};


/*********************************************************************
 * Configuration
 *********************************************************************/

static const struct usb_config_descriptor config = {

    .bLength = USB_DT_CONFIGURATION_SIZE,

    .bDescriptorType =
        USB_DT_CONFIGURATION,

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
 * USB control buffer
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

#ifdef DEBUGGING
        debug_puts(
            "USB: SET_CONTROL_LINE_STATE\r\n"
        );
#endif

        return USBD_REQ_HANDLED;


    case USB_CDC_REQ_SET_LINE_CODING:

#ifdef DEBUGGING
        debug_puts(
            "USB: SET_LINE_CODING\r\n"
        );
#endif

        if (*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;

        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}


/*********************************************************************
 * USB RX callback
 *
 * THIS IS THE IMPORTANT PART OF THIS TEST.
 *
 * Host sends data:
 *
 *     PC
 *      |
 *      v
 *   USB OUT
 *      |
 *      v
 * endpoint 0x01
 *      |
 *      v
 * this callback
 *      |
 *      v
 *    usb_rxq
 *
 *********************************************************************/

static void cdcacm_data_rx_cb(
    usbd_device *usbd_dev,
    uint8_t ep __attribute__((unused))
)
{
    uint8_t buf[64];

    int len;

    len = usbd_ep_read_packet(
        usbd_dev,
        0x01,
        buf,
        sizeof(buf)
    );

    if (len <= 0)
        return;


#ifdef DEBUGGING

    debug_puts("USB RX: len=");

    debug_dec((unsigned int)len);

    debug_puts("\r\n");

#endif


    for (int i = 0; i < len; i++) {

#ifdef DEBUGGING

        debug_hex8(buf[i]);
        debug_putc(' ');

#endif

        /*
         * Put byte into RX queue.
         *
         * Zero timeout because this callback must
         * not block.
         */
        xQueueSend(
            usb_rxq,
            &buf[i],
            0
        );
    }


#ifdef DEBUGGING

    debug_puts("\r\n");

#endif
}


/*********************************************************************
 * USB configuration callback
 *********************************************************************/

static void cdcacm_set_config(
    usbd_device *usbd_dev,
    uint16_t wValue __attribute__((unused))
)
{
#ifdef DEBUGGING

    debug_puts(
        "USB: SET_CONFIGURATION\r\n"
    );

#endif


    /*
     * USB CDC OUT
     */
    usbd_ep_setup(
        usbd_dev,
        0x01,
        USB_ENDPOINT_ATTR_BULK,
        64,
        cdcacm_data_rx_cb
    );


    /*
     * USB CDC IN
     *
     * Not used in this diagnostic.
     */
    usbd_ep_setup(
        usbd_dev,
        0x82,
        USB_ENDPOINT_ATTR_BULK,
        64,
        NULL
    );


    /*
     * CDC notification endpoint.
     */
    usbd_ep_setup(
        usbd_dev,
        0x83,
        USB_ENDPOINT_ATTR_INTERRUPT,
        16,
        NULL
    );


    usbd_register_control_callback(
        usbd_dev,

        USB_REQ_TYPE_CLASS |
        USB_REQ_TYPE_INTERFACE,

        USB_REQ_TYPE_TYPE |
        USB_REQ_TYPE_RECIPIENT,

        cdcacm_control_request
    );


    usb_initialized = true;


#ifdef DEBUGGING

    debug_puts(
        "USB: CONFIGURED\r\n"
    );

#endif
}


/*********************************************************************
 * USB task
 *
 * This task owns:
 *
 *     usbd_poll()
 *
 * No USB TX is done here in this diagnostic.
 *********************************************************************/
#if 0
static void usb_task(void *arg)
{
    usbd_device *udev =
        (usbd_device *)arg;


#ifdef DEBUGGING

    debug_puts(
        "USB TASK: started\r\n"
    );

#endif


    for (;;) {

        /*
         * This is what services the USB
         * peripheral and invokes callbacks.
         */
        usbd_poll(udev);

        taskYIELD();
    }
}
#endif
static void usb_task(void *arg)
{
    usbd_device *udev = (usbd_device *)arg;

    debug_puts("USB TASK: started\r\n");

    for (;;) {
        usbd_poll(udev);

        /*
         * Temporary USB RX -> USB TX test.
         *
         * If a byte is sitting in usb_rxq, send it directly
         * to the host through CDC IN endpoint 0x82.
         */
        if (usb_initialized) {
            uint8_t ch;

            if (xQueueReceive(usb_rxq, &ch, 0) == pdPASS) {

                debug_puts("USB ECHO TX: ");
                debug_hex8(ch);
                debug_puts("\r\n");

                int rc = usbd_ep_write_packet(
                    udev,
                    0x82,
                    &ch,
                    1
                );

                debug_puts("USB ECHO rc=");
                debug_hex8(rc);
                debug_puts("\r\n");
            }
        }

        taskYIELD();
    }
}



/*********************************************************************
 * USB RX diagnostic task
 *
 * For this test we DON'T echo anything.
 *
 * We merely prove that bytes reached the RX queue.
 *********************************************************************/

static void usb_rx_debug_task(
    void *arg __attribute__((unused))
)
{
    uint8_t c;


#ifdef DEBUGGING

    debug_puts(
        "USB RX DEBUG TASK: started\r\n"
    );

#endif


    for (;;) {

        if (xQueueReceive(
                usb_rxq,
                &c,
                portMAX_DELAY
            ) == pdPASS) {

#ifdef DEBUGGING

            debug_puts(
                "QUEUE RX: "
            );

            debug_hex8(c);

            debug_puts(
                "\r\n"
            );

#endif
        }
    }
}


/*********************************************************************
 * USB startup
 *********************************************************************/

static void usb_start(
    bool gpio_init,
    unsigned priority
)
{
    if (gpio_init) {

        rcc_periph_clock_enable(RCC_GPIOA);

        rcc_periph_clock_enable(RCC_USB);
    }


#ifdef DEBUGGING

    debug_puts(
        "USB: clocks enabled\r\n"
    );

#endif


    usb_rxq =
        xQueueCreate(
            256,
            sizeof(uint8_t)
        );


    if (usb_rxq == NULL) {

#ifdef DEBUGGING

        debug_puts(
            "ERROR: usb_rxq create failed\r\n"
        );

#endif

        return;
    }


    usb_device =
        usbd_init(
            &st_usbfs_v1_usb_driver,
            &dev,
            &config,
            usb_strings,
            3,
            usbd_control_buffer,
            sizeof(usbd_control_buffer)
        );


    if (usb_device == NULL) {

#ifdef DEBUGGING

        debug_puts(
            "ERROR: usbd_init failed\r\n"
        );

#endif

        return;
    }


#ifdef DEBUGGING

    debug_puts(
        "USB: usbd_init OK\r\n"
    );

#endif


    usbd_register_set_config_callback(
        usb_device,
        cdcacm_set_config
    );


#ifdef DEBUGGING

    debug_puts(
        "USB: config callback registered\r\n"
    );

#endif


    xTaskCreate(
        usb_task,
        "USB",
        300,
        usb_device,
        priority,
        NULL
    );


#ifdef DEBUGGING

    debug_puts(
        "USB: driver task created\r\n"
    );

#endif


    xTaskCreate(
        usb_rx_debug_task,
        "USB_RX",
        256,
        NULL,
        priority,
        NULL
    );


#ifdef DEBUGGING

    debug_puts(
        "USB: RX debug task created\r\n"
    );

#endif
}


/*********************************************************************
 * MAIN
 *********************************************************************/

int main(void)
{
    /*
     * Keep the clock setup exactly as the
     * known-working version.
     */
    rcc_clock_setup_pll(
        &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]
    );


    debug_init();


#ifdef DEBUGGING

    debug_puts(
        "\r\n"
        "================================\r\n"
        " STM32F103 USB CDC RX TEST\r\n"
        "================================\r\n"
    );

    debug_puts(
        "DEBUG USART3: 115200 8N1\r\n"
    );

#endif


    /*
     * Start USB.
     */
    usb_start(
        true,
        configMAX_PRIORITIES - 1
    );


#ifdef DEBUGGING

    debug_puts(
        "Starting FreeRTOS scheduler...\r\n"
    );

#endif


    vTaskStartScheduler();


    /*
     * Should never get here.
     */
    for (;;)
        ;
}
