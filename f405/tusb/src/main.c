#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "tusb.h"

uint32_t SystemCoreClock = 168000000;


/* ========================================================================
 * Configuration
 * ======================================================================== */

#define UART_BAUDRATE       115200

#define UART_RX_BUF_SIZE    1024
#define UART_TX_BUF_SIZE    1024


/* ========================================================================
 * UART3 ring buffers
 * ======================================================================== */

static volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t uart_tx_buf[UART_TX_BUF_SIZE];

/*
 * RX:
 *
 * rx_head = position where ISR writes
 * rx_tail = position where main reads
 */
static volatile uint16_t uart_rx_head;
static volatile uint16_t uart_rx_tail;

/*
 * TX:
 *
 * tx_head = position where producer writes
 * tx_tail = position where ISR reads
 */
static volatile uint16_t uart_tx_head;
static volatile uint16_t uart_tx_tail;


/* ========================================================================
 * Ring-buffer helpers
 * ======================================================================== */

static inline uint16_t rb_next(uint16_t pos, uint16_t size)
{
    pos++;

    if (pos >= size)
        pos = 0;

    return pos;
}


/* ========================================================================
 * UART RX buffer
 * ======================================================================== */

static bool uart_rx_available(void)
{
    return uart_rx_head != uart_rx_tail;
}


static bool uart_rx_get(uint8_t *c)
{
    uint16_t tail;

    tail = uart_rx_tail;

    if (tail == uart_rx_head)
        return false;

    *c = uart_rx_buf[tail];

    uart_rx_tail = rb_next(tail, UART_RX_BUF_SIZE);

    return true;
}


/* ========================================================================
 * UART TX buffer
 * ======================================================================== */

static bool uart_tx_put(uint8_t c)
{
    uint16_t head;
    uint16_t next;

    head = uart_tx_head;
    next = rb_next(head, UART_TX_BUF_SIZE);

    /*
     * Buffer full.
     */
    if (next == uart_tx_tail)
        return false;

    uart_tx_buf[head] = c;

    /*
     * Publish byte before enabling TXE.
     */
    uart_tx_head = next;

    /*
     * Enable USART3 TX empty interrupt.
     */
    usart_enable_tx_interrupt(USART3);

    return true;
}


/* ========================================================================
 * Clock
 * ======================================================================== */

static void clock_setup(void)
{
    /*
     * Assumes 8 MHz HSE.
     *
     * Result:
     *
     * SYSCLK = 168 MHz
     * USB    = 48 MHz
     */
    //rcc_clock_setup_pll(&rcc_hse_8mhz_168mhz);
rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}


/* ========================================================================
 * USART3 setup
 * ======================================================================== */

static void usart3_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    /*
     * PB10 = USART3_TX
     * PB11 = USART3_RX
     *
     * Alternate function 7.
     */
    gpio_mode_setup(
        GPIOB,
        GPIO_MODE_AF,
        GPIO_PUPD_NONE,
        GPIO10 | GPIO11
    );

    gpio_set_af(
        GPIOB,
        GPIO_AF7,
        GPIO10 | GPIO11
    );

    /*
     * 115200 8N1
     */
    usart_set_baudrate(
        USART3,
        UART_BAUDRATE
    );

    usart_set_databits(
        USART3,
        8
    );

    usart_set_stopbits(
        USART3,
        USART_STOPBITS_1
    );

    usart_set_mode(
        USART3,
        USART_MODE_TX_RX
    );

    usart_set_parity(
        USART3,
        USART_PARITY_NONE
    );

    usart_set_flow_control(
        USART3,
        USART_FLOWCONTROL_NONE
    );

    /*
     * Clear pending status by reading SR/DR if necessary.
     */
    (void) USART_SR(USART3);
    (void) USART_DR(USART3);

    /*
     * Enable USART3 interrupt in NVIC.
     */
    nvic_enable_irq(NVIC_USART3_IRQ);

    /*
     * Enable RX-not-empty interrupt.
     *
     * TXE is enabled only when there is data to transmit.
     */
    usart_enable_rx_interrupt(USART3);

    /*
     * Finally enable USART.
     */
    usart_enable(USART3);
}


/* ========================================================================
 * USART3 interrupt handler
 * ======================================================================== */

void usart3_isr(void)
{
    uint32_t sr;

    sr = USART_SR(USART3);


    /* --------------------------------------------------------------------
     * RXNE
     * -------------------------------------------------------------------- */

    if (sr & USART_SR_RXNE)
    {
        uint8_t c;
        uint16_t head;
        uint16_t next;

        /*
         * Reading DR clears RXNE.
         */
        c = usart_recv(USART3);

        head = uart_rx_head;
        next = rb_next(head, UART_RX_BUF_SIZE);

        /*
         * If buffer is full, drop the new byte.
         *
         * This is preferable to overwriting unread data.
         */
        if (next != uart_rx_tail)
        {
            uart_rx_buf[head] = c;

            /*
             * Publish byte.
             */
            uart_rx_head = next;
        }
    }


    /* --------------------------------------------------------------------
     * TXE
     * -------------------------------------------------------------------- */

    if ((sr & USART_SR_TXE) != 0)
    {
        uint16_t tail;

        tail = uart_tx_tail;

        if (tail != uart_tx_head)
        {
            /*
             * There is another byte to transmit.
             */
            usart_send(USART3, uart_tx_buf[tail]);

            uart_tx_tail = rb_next(
                tail,
                UART_TX_BUF_SIZE
            );
        }
        else
        {
            /*
             * Nothing left to transmit.
             *
             * Disable TXE interrupt until another byte is queued.
             */
            usart_disable_tx_interrupt(USART3);
        }
    }
}


/* ========================================================================
 * USB CDC -> UART3
 *
 * TinyUSB calls this from tud_task() when USB data arrives.
 * ======================================================================== */

void tud_cdc_rx_cb(uint8_t itf)
{
    (void) itf;

    uint8_t buf[64];

    while (tud_cdc_available())
    {
        uint32_t count;

        count = tud_cdc_read(
            buf,
            sizeof(buf)
        );

        for (uint32_t i = 0; i < count; i++)
        {
            /*
             * Put byte into UART TX ring.
             *
             * The USART3 TXE ISR actually sends it.
             */
            (void) uart_tx_put(buf[i]);
        }
    }
}


/* ========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    clock_setup();

    /*
     * USART3:
     *
     * PB10 TX
     * PB11 RX
     * 115200 8N1
     */
    usart3_setup();

    /*
     * TinyUSB device stack.
     */
rcc_periph_clock_enable(RCC_OTGFS);
tusb_init();


    while (1)
    {
        /*
         * USB stack.
         *
         * This processes USB events and invokes tud_cdc_rx_cb()
         * when the host sends data.
         */
        tud_task();


        /*
         * ---------------------------------------------------------------
         * USART3 RX -> USB CDC
         * ---------------------------------------------------------------
         *
         * USART3 RX is collected asynchronously by the ISR.
         *
         * Here we move those bytes to TinyUSB.
         */

        if (tud_cdc_connected())
        {
            uint8_t c;

            while (uart_rx_available())
            {
                if (!tud_cdc_write_available())
                    break;

                if (!uart_rx_get(&c))
                    break;

                tud_cdc_write_char(c);
            }

            /*
             * Start USB transmission.
             */
            tud_cdc_write_flush();
        }
    }

    return 0;
}

