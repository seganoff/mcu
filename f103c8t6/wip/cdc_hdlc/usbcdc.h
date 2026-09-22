/* usbd_ ep_setup _ep_write_packet _ep_read_packet
 * lib/usb/usb.c
 * return usbd_dev->driver->ep_read_packet
 * hXXps://libopencm3.org/docs/latest/stm32f4/html/group__usb__drivers__file.html
 */
#ifndef LIBUSBCDC_H
#define LIBUSBCDC_H

//#include <stdarg.h>
//#include <stdlib.h>
//#include <string.h>
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

//int usb_getc(void);//blocking
//int usb_getline(char *buf,unsigned maxbuf);
// /\return getline(buf,bufsiz,usb_getc,usb_putc); getline.h+c
//void usb_putc(char ch);
//void usb_puts(const char *buf);//while(*buf) usb_putc(*buf++)
//void usb_write(const char *buf,unsigned bytes);

//int usb_vprintf(const char *format,va_list ap);
//int usb_printf(const char *format,...);

#endif /* LIBUSBCDC_H */
