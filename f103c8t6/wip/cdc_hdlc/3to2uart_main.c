#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "usbcdc.h"


/*********************************************************************
 * UART queues
 *********************************************************************/

static QueueHandle_t txq3;
static QueueHandle_t rxq3;

static QueueHandle_t txq2;
static QueueHandle_t rxq2;


/*********************************************************************
 * UART initialization
 *
 * USART3:
 *   PB10 = TX
 *   PB11 = RX
 *
 * USART2:
 *   PA2 = TX
 *   PA3 = RX
 *
 * USART3 = terminal
 * USART2 = MAX3232 / RS-232 device
 *********************************************************************/

static void uart_init(void)
{
    /***********************
     * USART3
     ***********************/

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

    txq3 = xQueueCreate(256, sizeof(uint8_t));
    rxq3 = xQueueCreate(256, sizeof(uint8_t));


    /***********************
     * USART2
     ***********************/

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    /* PA2 = USART2_TX */
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART2_TX
    );

    /* PA3 = USART2_RX */
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_USART2_RX
    );

    /*
     * LG 43UN700P:
     *
     * 9600 baud
     * 8 data bits
     * no parity
     * 1 stop bit
     */
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);

    txq2 = xQueueCreate(256, sizeof(uint8_t));
    rxq2 = xQueueCreate(256, sizeof(uint8_t));
}


/*********************************************************************
 * Return TX queue belonging to UART
 *********************************************************************/

static QueueHandle_t uart_tx_queue(uint32_t uart)
{
    if (uart == USART3)
        return txq3;

    if (uart == USART2)
        return txq2;

    return NULL;
}


/*********************************************************************
 * Return RX queue belonging to UART
 *********************************************************************/

static QueueHandle_t uart_rx_queue(uint32_t uart)
{
    if (uart == USART3)
        return rxq3;

    if (uart == USART2)
        return rxq2;

    return NULL;
}


/*********************************************************************
 * Queue one character for transmission.
 *
 * Non-blocking.
 *
 * true  = character was queued
 * false = queue full / invalid UART
 *********************************************************************/

static bool uart_putc(uint32_t uart, uint8_t c)
{
    QueueHandle_t q = uart_tx_queue(uart);

    if (q == NULL)
        return false;

    return xQueueSend(q, &c, 0) == pdPASS;
}


/*********************************************************************
 * Queue string for transmission.
 *
 * Non-blocking.
 *********************************************************************/

static bool uart_puts(uint32_t uart, const char *s)
{
    while (*s) {

        if (!uart_putc(uart, (uint8_t)*s))
            return false;

        ++s;
    }

    return true;
}


/*********************************************************************
 * Receive one character from RX queue.
 *
 * Non-blocking:
 *
 *   -1     = nothing available
 *   0..255 = received character
 *********************************************************************/

static int uart_getc_nb(uint32_t uart)
{
    QueueHandle_t q = uart_rx_queue(uart);
    uint8_t c;

    if (q == NULL)
        return -1;

    if (xQueueReceive(q, &c, 0) != pdPASS)
        return -1;

    return (int)c;
}


/*********************************************************************
 * UART hardware service task
 *
 * Moves data between:
 *
 * USART3 hardware <-> txq3/rxq3
 * USART2 hardware <-> txq2/rxq2
 *
 * The bridge/application never directly touches the USART hardware.
 *********************************************************************/

static void uart_task(void *arg)
{
    (void)arg;

    uint8_t ch;

    for (;;) {

        /***********************
         * USART3 RX
         ***********************/

        if (USART_SR(USART3) & USART_SR_RXNE) {

            ch = usart_recv(USART3);

            /*
             * Don't block here.
             *
             * If queue is full, drop the character.
             */
            xQueueSend(rxq3, &ch, 0);
        }


        /***********************
         * USART3 TX
         ***********************/

        if (USART_SR(USART3) & USART_SR_TXE) {

            if (xQueueReceive(txq3, &ch, 0) == pdPASS) {

                usart_send(USART3, ch);
            }
        }


        /***********************
         * USART2 RX
         ***********************/

        if (USART_SR(USART2) & USART_SR_RXNE) {

            ch = usart_recv(USART2);

            /*
             * Don't block here.
             */
            xQueueSend(rxq2, &ch, 0);
        }


        /***********************
         * USART2 TX
         ***********************/

        if (USART_SR(USART2) & USART_SR_TXE) {

            if (xQueueReceive(txq2, &ch, 0) == pdPASS) {

                usart_send(USART2, ch);
            }
        }


        /*
         * Polling version for now.
         *
         * Later this can be replaced with USART interrupts.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/*********************************************************************
 * USART3 <-> USART2 bridge
 *
 * USART3:
 *   terminal / picocom
 *
 * USART2:
 *   MAX3232 -> LG 43UN700P
 *
 * Operation:
 *
 *   User types:
 *
 *       kh 01 32<ENTER>
 *
 *   USART3 receives the line.
 *
 *   We send:
 *
 *       kh 01 32<CR>
 *
 *   to USART2.
 *
 *   Response from USART2 is forwarded to USART3.
 *********************************************************************/

static void bridge_task(void *arg)
{
    (void)arg;

    char line[256];
    unsigned pos = 0;

    for (;;) {

        /*********************************************************
         * USART3 -> USART2
         *********************************************************/

        int c = uart_getc_nb(USART3);

        if (c >= 0) {

            /*
             * ENTER / CR
             *
             * picocom normally gives us CR when Enter is pressed.
             */
            if (c == '\r') {

                /*
                 * Echo a clean newline to terminal.
                 */
                uart_putc(USART3, '\r');
                uart_putc(USART3, '\n');


                /*
                 * Send accumulated line to LG monitor.
                 */
                if (pos > 0) {

                    for (unsigned i = 0; i < pos; ++i) {

                        /*
                         * Wait until there is room in TX queue.
                         */
                        while (!uart_putc(
                            USART2,
                            (uint8_t)line[i]
                        )) {
                            taskYIELD();
                        }
                    }


                    /*
                     * LG command terminator:
                     *
                     * CR only.
                     */
                    while (!uart_putc(USART2, '\r'))
                        taskYIELD();
                }


                /*
                 * Prepare for next command.
                 */
                pos = 0;
            }

            /*
             * If terminal sends LF after CR,
             * ignore it.
             */
            else if (c == '\n') {

                /* Nothing */
            }

            /*
             * Normal character.
             */
            else {

                /*
                 * Echo character back to picocom.
                 */
                uart_putc(USART3, (uint8_t)c);


                /*
                 * Store in line buffer.
                 */
                if (pos < sizeof(line) - 1) {

                    line[pos++] = (char)c;
                }
                else {

                    /*
                     * Line too long.
                     *
                     * Reset buffer.
                     */
                    pos = 0;
                }
            }
        }


        /*********************************************************
         * USART2 -> USART3
         *
         * Anything returned by the LG monitor gets sent to
         * picocom.
         *********************************************************/

        c = uart_getc_nb(USART2);

        if (c >= 0) {

            /*
             * Wait if terminal TX queue is temporarily full.
             */
            while (!uart_putc(USART3, (uint8_t)c)) {

                taskYIELD();
            }
        }


        /*
         * Give other tasks CPU time.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/*********************************************************************
 * Optional flasher task
 *********************************************************************/

//static void flasher(void *arg)
//{
//    (void)arg;
//
//    uart_puts(
//        USART3,
//        "flasher started\r\n"
//    );
//
//    for (;;) {
//
//        gpio_toggle(GPIOB, GPIO2);
//
//        vTaskDelay(pdMS_TO_TICKS(600));
//    }
//}
//

/*********************************************************************
 * main
 *********************************************************************/

int main(void)
{
    /***************************************************************
     * Blue Pill:
     *
     * 8 MHz HSE -> 72 MHz
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
     * PB2 debug output
     ***************************************************************/

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO2
    );


    /***************************************************************
     * PC13 Blue Pill LED
     ***************************************************************/

    gpio_set_mode(
        GPIOC,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO13
    );


    /***************************************************************
     * Initialize USART2 + USART3 + queues.
     ***************************************************************/

    uart_init();


    /***************************************************************
     * Direct hardware startup message.
     *
     * FreeRTOS isn't running yet, so don't use uart_puts()
     * here because uart_puts() uses the TX queue.
     ***************************************************************/

    const char startup[] =
        "UART bridge ready\r\n";

    for (unsigned i = 0;
         i < sizeof(startup) - 1;
         ++i) {

        while (!(USART_SR(USART3) & USART_SR_TXE))
            ;

        usart_send(USART3, startup[i]);
    }


    /***************************************************************
     * USB CDC
     *
     * Keep USB priority below the UART bridge for now.
     ***************************************************************/

    usb_start(true, 2);


    /***************************************************************
     * UART hardware service
     ***************************************************************/

    xTaskCreate(
        uart_task,
        "uart",
        256,
        NULL,
        3,
        NULL
    );


    /***************************************************************
     * USART3 <-> USART2 bridge
     ***************************************************************/

    xTaskCreate(
        bridge_task,
        "bridge",
        512,
        NULL,
        2,
        NULL
    );


    /***************************************************************
     * Optional flasher.
     *
     * Leave disabled while debugging RS-232.
     *
     * Uncomment if wanted:
     *
     * xTaskCreate(
     *     flasher,
     *     "flash",
     *     128,
     *     NULL,
     *     1,
     *     NULL
     * );
     ***************************************************************/


    /***************************************************************
     * Start FreeRTOS
     ***************************************************************/

    vTaskStartScheduler();


    /*
     * Should never reach here.
     */
    for (;;)
        ;
}
