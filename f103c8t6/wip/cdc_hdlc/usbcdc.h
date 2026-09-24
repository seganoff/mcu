/* usbd_ ep_setup _ep_write_packet _ep_read_packet
 * lib/usb/usb.c
 * return usbd_dev->driver->ep_read_packet
 * hXXps://libopencm3.org/docs/latest/stm32f4/html/group__usb__drivers__file.html
 */
#if 0
#ifndef LIBUSBCDC_H
#define LIBUSBCDC_H

#include <stdbool.h>
#include <stdint.h>
// ll /opt/gcc-arm/arm-none-eabi/include/
// find /opt/gcc-arm/arm-none-eabi/include/ -name "stdbool.h"


void usb_start(/*void*/bool gpio_init,unsigned priority);
bool usb_ready(void);//ok
int usb_peek(void);

int usb_getc(void);
void usb_write(const char *buf,unsigned bytes);
void usb_puts(const char *buf);
void usb_putc(char ch);

void usb_putdec(uint32_t v);
void usb_puthex32(uint32_t v);

#endif /* LIBUSBCDC_H */
#endif //0


#ifndef USBCDC_H
#define USBCDC_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Start USB CDC ACM.
 *
 * priority:
 *     FreeRTOS priority of the USB driver task.
 */
void usb_start(bool gpio_init, unsigned priority);

/*
 * USB connection state.
 *
 * true = host has configured the CDC device.
 */
bool usb_ready(void);

/*
 * Blocking receive.
 *
 * Returns:
 *     0..255  received byte
 *     -1      error
 */
int usb_getc(void);

/*
 * Non-blocking receive.
 *
 * Returns:
 *     0..255  received byte
 *     -1      no byte available
 */
int usb_getc_nb(void);

/*
 * Returns:
 *     1 = character available
 *     0 = nothing available
 *    -1 = error
 */
int usb_peek(void);

/*
 * Blocking transmit.
 */
void usb_putc(char ch);
void usb_puts(const char *str);
void usb_write(const char *buf, unsigned bytes);

/*
 * Non-blocking transmit.
 *
 * Returns true if everything was queued.
 * Returns false if the TX queue did not have enough space.
 */
bool usb_putc_nb(char ch);
bool usb_write_nb(const char *buf, unsigned bytes);

void usb_puthex32(uint32_t v);
void usb_putdec(uint32_t v);

#endif /* USBCDC_H */

