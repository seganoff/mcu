#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "usbcdc.h"

//static void usb_test_task(void *arg)
//{
//    (void)arg;
//
//    usb_puts("\r\nUSB TEST TASK STARTED\r\n");
//
//    for (;;) {
//        int c = usb_getc();
//
//        if (c >= 0) {
//            usb_putc((char)c);
//        }
//    }
//}



/*********************************************************************
 * DEBUGGING
 *
 * 1 = verbose USART3 debug output
 * 0 = remove debug output
 *********************************************************************/

#define DEBUGGING 1


/*********************************************************************
 * Configuration
 *********************************************************************/

#define USB_LINE_MAX       256
#define RS232_BAUD         9600

/*
 * After this much silence from USART2, consider the response finished.
 */
#define RS232_REPLY_GAP_MS 30


/*********************************************************************
 * Queues
 *********************************************************************/

static QueueHandle_t txq2;
static QueueHandle_t rxq2;


/*********************************************************************
 * USART3 DEBUG
 *
 * USART3 is NEVER part of the USB <-> RS232 bridge.
 *
 * PB10 = USART3 TX
 * PB11 = USART3 RX
 *********************************************************************/

static void debug_putc(char c)
{
#if DEBUGGING
    usart_send_blocking(USART3, (uint8_t)c);
#else
    (void)c;
#endif
}


static void debug_puts(const char *s)
{
#if DEBUGGING
    while (*s)
        debug_putc(*s++);
#else
    (void)s;
#endif
}


static void debug_puthex8(uint8_t v)
{
#if DEBUGGING
    static const char hex[] = "0123456789ABCDEF";

    debug_putc(hex[(v >> 4) & 0x0f]);
    debug_putc(hex[v & 0x0f]);
#else
    (void)v;
#endif
}


static void debug_putdec(unsigned v)
{
#if DEBUGGING
    char buf[10];
    int i = 0;

    if (v == 0) {
        debug_putc('0');
        return;
    }

    while (v && i < (int)sizeof(buf)) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }

    while (i--)
        debug_putc(buf[i]);
#else
    (void)v;
#endif
}


/*
 * Print a byte in a useful form:
 *
 *   [6B 'k']
 *   [0D CR]
 *   [0A LF]
 *   [08 BS]
 */
static void debug_byte(uint8_t c)
{
#if DEBUGGING

    debug_putc('[');
    debug_puthex8(c);
    debug_puts(" ");

    if (c == '\r') {
        debug_puts("CR");
    }
    else if (c == '\n') {
        debug_puts("LF");
    }
    else if (c == '\b') {
        debug_puts("BS");
    }
    else if (c == 0x7f) {
        debug_puts("DEL");
    }
    else if (c >= 32 && c <= 126) {
        debug_putc('\'');
        debug_putc((char)c);
        debug_putc('\'');
    }
    else {
        debug_puts("CTRL");
    }

    debug_putc(']');
#else
    (void)c;
#endif
}


/*********************************************************************
 * UART initialization
 *********************************************************************/

static void uart_init(void)
{
    /***************************************************************
     * USART3 - DEBUG
     ***************************************************************/

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    /* PB10 = USART3_TX */
    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART3_TX
    );

    /* PB11 = USART3_RX */
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
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    usart_enable(USART3);


    /***************************************************************
     * USART2 - RS232 / LG MONITOR
     *
     * PA2 = TX
     * PA3 = RX
     ***************************************************************/

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART2_TX
    );

    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_USART2_RX
    );

    usart_set_baudrate(USART2, RS232_BAUD);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);


    /***************************************************************
     * Queues
     ***************************************************************/

    txq2 = xQueueCreate(256, sizeof(uint8_t));
    rxq2 = xQueueCreate(256, sizeof(uint8_t));

    configASSERT(txq2 != NULL);
    configASSERT(rxq2 != NULL);


#if DEBUGGING
    debug_puts("\r\n");
    debug_puts("========================================\r\n");
    debug_puts("UART initialization complete\r\n");
    debug_puts("USART3: DEBUG @ 115200 8N1\r\n");
    debug_puts("USART2: RS232 @ 9600 8N1\r\n");
    debug_puts("TX queue: 256 bytes\r\n");
    debug_puts("RX queue: 256 bytes\r\n");
    debug_puts("========================================\r\n");
#endif
}


/*********************************************************************
 * USART2 hardware TX
 *
 * NON-BLOCKING.
 *
 * Returns:
 *   true  = byte written to USART2
 *   false = USART2 TX register not ready
 *********************************************************************/

static bool uart2_hw_putc_nb(uint8_t c)
{
    if (!(USART_SR(USART2) & USART_SR_TXE))
        return false;

    usart_send(USART2, c);

    return true;
}


/*********************************************************************
 * USART2 hardware RX
 *
 * NON-BLOCKING.
 *
 * Returns:
 *   -1      = no character
 *   0..255  = received character
 *********************************************************************/

static int uart2_hw_getc_nb(void)
{
    if (!(USART_SR(USART2) & USART_SR_RXNE))
        return -1;

    return usart_recv(USART2);
}


/*********************************************************************
 * USB CDC -> USART2
 *
 * Reads one line from USB.
 *
 * Example:
 *
 *     kh 01 24<ENTER>
 *
 * gets converted into:
 *
 *     kh 01 24\r
 *
 * and placed into txq2.
 *********************************************************************/

static void usb_to_rs232_task(void *arg)
{
    (void)arg;

    char line[USB_LINE_MAX];
    size_t pos = 0;

#if DEBUGGING
    debug_puts("[USB->RS232] task started\r\n");
#endif

    for (;;) {

        /*
         * This blocks until USB CDC receives a character.
         */
        int c = usb_getc();

        if (c < 0) {

#if DEBUGGING
            debug_puts(
                "[USB->RS232] usb_getc() returned ERROR\r\n"
            );
#endif

            continue;
        }


#if DEBUGGING
        /*
         * VERY IMPORTANT:
         *
         * If you type "k", this should immediately appear on
         * USART3:
         *
         * [USB->RS232] RX [6B 'k']
         *
         * If you see nothing here, the problem is before this task.
         */
        debug_puts("[USB->RS232] RX ");
        debug_byte((uint8_t)c);
        debug_puts("\r\n");
#endif


        /***********************************************************
         * ENTER
         ***********************************************************/

        if (c == '\r' || c == '\n') {

#if DEBUGGING
            debug_puts("[USB->RS232] ENTER\r\n");
            debug_puts("[USB->RS232] line length = ");
            debug_putdec((unsigned)pos);
            debug_puts("\r\n");
#endif

            if (pos > 0) {

#if DEBUGGING
                debug_puts("[USB->RS232] command = \"");
                for (size_t i = 0; i < pos; i++)
                    debug_putc(line[i]);
                debug_puts("\"\r\n");

                debug_puts(
                    "[USB->RS232] sending command to USART2:\r\n"
                );
#endif

                /***************************************************
                 * Put complete command into USART2 TX queue.
                 ***************************************************/

                for (size_t i = 0; i < pos; i++) {

                    uint8_t ch = (uint8_t)line[i];

                    if (xQueueSend(
                            txq2,
                            &ch,
                            portMAX_DELAY) != pdPASS) {

#if DEBUGGING
                        debug_puts(
                            "[USB->RS232] ERROR: TX queue send failed\r\n"
                        );
#endif
                    }

#if DEBUGGING
                    debug_puts("  TXQ ");
                    debug_byte(ch);
                    debug_puts("\r\n");
#endif
                }


                /***************************************************
                 * LG protocol command terminator.
                 *
                 * Do NOT send LF here.
                 ***************************************************/

                uint8_t cr = '\r';

                xQueueSend(
                    txq2,
                    &cr,
                    portMAX_DELAY
                );

#if DEBUGGING
                debug_puts("  TXQ ");
                debug_byte(cr);
                debug_puts("  <COMMAND TERMINATOR>\r\n");
#endif

                pos = 0;

#if DEBUGGING
                debug_puts(
                    "[USB->RS232] command queued completely\r\n"
                );
#endif
            }
            else {

#if DEBUGGING
                debug_puts(
                    "[USB->RS232] empty line\r\n"
                );
#endif
            }


            /*
             * Terminal formatting.
             */
            usb_puts("\r\n");

            continue;
        }


        /***********************************************************
         * BACKSPACE
         ***********************************************************/

        if (c == '\b' || c == 0x7f) {

#if DEBUGGING
            debug_puts("[USB->RS232] BACKSPACE\r\n");
#endif

            if (pos > 0) {

                pos--;

                /*
                 * Remove character visually from terminal.
                 */
                usb_puts("\b \b");
            }

            continue;
        }


        /***********************************************************
         * NORMAL CHARACTER
         ***********************************************************/

        if (pos < sizeof(line) - 1) {

            line[pos++] = (char)c;

            /*
             * Local echo.
             */
            usb_putc((char)c);

#if DEBUGGING
            debug_puts("[USB->RS232] buffered, pos=");
            debug_putdec((unsigned)pos);
            debug_puts("\r\n");
#endif

        }
        else {

            /*
             * Buffer full.
             */
            usb_putc('\a');

#if DEBUGGING
            debug_puts(
                "[USB->RS232] ERROR: command buffer FULL\r\n"
            );
#endif
        }
    }
}


/*********************************************************************
 * USART2 -> USB
 *
 * Also drains txq2 into USART2.
 *********************************************************************/

static void rs232_task(void *arg)
{
    (void)arg;

    bool reply_active = false;
    TickType_t last_rx_time = 0;

#if DEBUGGING
    debug_puts("[RS232] task started\r\n");
#endif

    for (;;) {


        /***********************************************************
         * USART2 TX
         *
         * txq2 -> USART2 hardware
         ***********************************************************/

        uint8_t txchar;

        if (xQueuePeek(txq2, &txchar, 0) == pdPASS) {

            if (uart2_hw_putc_nb(txchar)) {

                /*
                 * Hardware accepted it.
                 * NOW remove it from queue.
                 */
                xQueueReceive(
                    txq2,
                    &txchar,
                    0
                );

#if DEBUGGING
                debug_puts("[RS232] HW TX ");
                debug_byte(txchar);
                debug_puts("\r\n");
#endif

            }
        }


        /***********************************************************
         * USART2 RX
         *
         * Hardware -> rxq2
         *
         * Drain ALL currently available bytes rather than just
         * one byte per scheduler iteration.
         ***********************************************************/

        for (;;) {

            int c = uart2_hw_getc_nb();

            if (c < 0)
                break;

            uint8_t rxchar = (uint8_t)c;

#if DEBUGGING
            debug_puts("[RS232] HW RX ");
            debug_byte(rxchar);
            debug_puts("\r\n");
#endif

            if (xQueueSend(
                    rxq2,
                    &rxchar,
                    0) != pdPASS) {

#if DEBUGGING
                debug_puts(
                    "[RS232] ERROR: RX QUEUE FULL - BYTE DROPPED\r\n"
                );
#endif
            }
        }


        /***********************************************************
         * rxq2 -> USB
         ***********************************************************/

        uint8_t usbchar;

        while (xQueueReceive(
                   rxq2,
                   &usbchar,
                   0) == pdPASS) {

#if DEBUGGING
            debug_puts("[RS232] -> USB ");
            debug_byte(usbchar);
            debug_puts("\r\n");
#endif

            usb_putc((char)usbchar);

            reply_active = true;
            last_rx_time = xTaskGetTickCount();
        }


        /***********************************************************
         * RESPONSE END DETECTION
         ***********************************************************/

        if (reply_active) {

            TickType_t now = xTaskGetTickCount();

            if ((now - last_rx_time) >=
                pdMS_TO_TICKS(RS232_REPLY_GAP_MS)) {

#if DEBUGGING
                debug_puts(
                    "[RS232] response timeout -> newline\r\n"
                );
#endif

                usb_puts("\r\n");

                reply_active = false;
            }
        }


        /*
         * Give the scheduler some room.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/*********************************************************************
 * USART3 debug task
 *********************************************************************/

static void flasher(void *arg)
{
    (void)arg;

#if DEBUGGING
    debug_puts("[DEBUG] flasher task started\r\n");
#endif

    for (;;) {

        gpio_toggle(GPIOB, GPIO2);

#if DEBUGGING
        debug_puts(
            "[DEBUG] bridge alive\r\n"
        );
#endif

        vTaskDelay(
            pdMS_TO_TICKS(600)
        );
    }
}


/*********************************************************************
 * main
 *********************************************************************/

int main(void)
{
    /***************************************************************
     * System clock
     ***************************************************************/

    rcc_clock_setup_pll(
        &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]
    );


    /***************************************************************
     * GPIO clocks
     ***************************************************************/

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);


    /***************************************************************
     * Debug LEDs
     ***************************************************************/

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO2
    );

    gpio_set_mode(
        GPIOC,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO13
    );


    /***************************************************************
     * UARTs + queues
     ***************************************************************/

    uart_init();


#if DEBUGGING

    debug_puts("\r\n");
    debug_puts("########################################\r\n");
    debug_puts("# STM32 USB -> RS232 BRIDGE            #\r\n");
    debug_puts("########################################\r\n");
    debug_puts("\r\n");

    debug_puts("DEBUGGING      = ON\r\n");
    debug_puts("USART3         = DEBUG @ 115200 8N1\r\n");
    debug_puts("USART2         = RS232 @ 9600 8N1\r\n");
    debug_puts("USB CDC        = terminal input/output\r\n");
    debug_puts("RS232 gap      = ");
    debug_putdec(RS232_REPLY_GAP_MS);
    debug_puts(" ms\r\n");
    debug_puts("\r\n");

#endif


    /***************************************************************
     * Start USB CDC
     ***************************************************************/

#if DEBUGGING
    debug_puts("[MAIN] starting USB CDC...\r\n");
#endif

    usb_start(
        true,
        configMAX_PRIORITIES - 1
    );

#if DEBUGGING
    debug_puts(
        "[MAIN] usb_start() returned\r\n"
    );
//usb_start(true, 2);
//while (!usb_ready()) {vTaskDelay(pdMS_TO_TICKS(10));}
//
//    xTaskCreate(
//        usb_test_task,
//        "usbtest",
//        256,
//        NULL,
//        2,
//        NULL
//    );

usb_puts("\r\nUSB CDC configured!\r\n");
debug_puts("\r\n[MAIN] USB CDC configured!\r\n");

#endif


    /***************************************************************
     * USB -> RS232
     ***************************************************************/

#if DEBUGGING
    debug_puts(
        "[MAIN] creating USB->RS232 task\r\n"
    );
#endif

    BaseType_t rc;

    rc = xTaskCreate(
        usb_to_rs232_task,
        "usb2rs232",
        384,
        NULL,
        2,
        NULL
    );

#if DEBUGGING
    debug_puts("[MAIN] USB->RS232 xTaskCreate = ");
    debug_putdec((unsigned)rc);
    debug_puts("\r\n");
#endif


    /***************************************************************
     * RS232 task
     ***************************************************************/

#if DEBUGGING
    debug_puts(
        "[MAIN] creating RS232 task\r\n"
    );
#endif

    rc = xTaskCreate(
        rs232_task,
        "rs232",
        384,
        NULL,
        2,
        NULL
    );

#if DEBUGGING
    debug_puts("[MAIN] RS232 xTaskCreate = ");
    debug_putdec((unsigned)rc);
    debug_puts("\r\n");
#endif


    /***************************************************************
     * USART3 debug task
     ***************************************************************/

    rc = xTaskCreate(
        flasher,
        "debug",
        256,
        NULL,
        1,
        NULL
    );

#if DEBUGGING
    debug_puts("[MAIN] DEBUG xTaskCreate = ");
    debug_putdec((unsigned)rc);
    debug_puts("\r\n");

    debug_puts(
        "\r\n[MAIN] starting FreeRTOS scheduler...\r\n"
    );
#endif


    /***************************************************************
     * Start scheduler
     ***************************************************************/

    vTaskStartScheduler();


    /*
     * Should never get here.
     */
    for (;;)
        ;
}

