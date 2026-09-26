#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/nvic.h>

#include "tusb.h"


#define UART3_RX_BUF_SIZE 256
#define UART3_TX_BUF_SIZE 256


static volatile uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];
static volatile uint16_t uart3_rx_head;
static volatile uint16_t uart3_rx_tail;

static volatile uint8_t uart3_tx_buf[UART3_TX_BUF_SIZE];
static volatile uint16_t uart3_tx_head;
static volatile uint16_t uart3_tx_tail;


/* -------------------------------------------------------------------------- */
/* Debug output                                                               */
/* -------------------------------------------------------------------------- */

static void dbg_putc(char c)
{
    while (!usart_get_flag(USART3, USART_SR_TXE))
        ;

    usart_send(USART3, (uint8_t)c);
}


static void dbg_puts(const char *s)
{
    while (*s)
        dbg_putc(*s++);
}


static void dbg_puthex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";

    dbg_putc(hex[(v >> 4) & 0x0F]);
    dbg_putc(hex[v & 0x0F]);
}


static void dbg_puthex32(uint32_t v)
{
    dbg_puthex8((uint8_t)(v >> 24));
    dbg_puthex8((uint8_t)(v >> 16));
    dbg_puthex8((uint8_t)(v >> 8));
    dbg_puthex8((uint8_t)v);
}


/* -------------------------------------------------------------------------- */
/* UART3 RX                                                                    */
/* -------------------------------------------------------------------------- */

static bool uart3_rx_put(uint8_t c)
{
    uint16_t next =
        (uart3_rx_head + 1) % UART3_RX_BUF_SIZE;

    if (next == uart3_rx_tail)
        return false;

    uart3_rx_buf[uart3_rx_head] = c;
    uart3_rx_head = next;

    return true;
}


/* -------------------------------------------------------------------------- */
/* UART3 TX                                                                    */
/* -------------------------------------------------------------------------- */

static bool uart3_tx_put(uint8_t c)
{
    uint16_t next =
        (uart3_tx_head + 1) % UART3_TX_BUF_SIZE;

    if (next == uart3_tx_tail)
        return false;

    uart3_tx_buf[uart3_tx_head] = c;
    uart3_tx_head = next;

    /*
     * TXE is level-triggered. Enable the interrupt after putting
     * the byte in the FIFO so the ISR can start transmitting it.
     */
    usart_enable_tx_interrupt(USART3);

    return true;
}


/* -------------------------------------------------------------------------- */
/* USART3 interrupt                                                            */
/* -------------------------------------------------------------------------- */

void usart3_isr(void)
{
    /*
     * RXNE
     */
    if (usart_get_flag(USART3, USART_SR_RXNE))
    {
        uint8_t c = usart_recv(USART3);

        uart3_rx_put(c);
    }

    /*
     * TXE
     */
    if (usart_get_flag(USART3, USART_SR_TXE))
    {
        if (uart3_tx_tail != uart3_tx_head)
        {
            uint8_t c = uart3_tx_buf[uart3_tx_tail];

            uart3_tx_tail =
                (uart3_tx_tail + 1) % UART3_TX_BUF_SIZE;

            usart_send(USART3, c);
        }
        else
        {
            /*
             * Nothing left to transmit.
             * TXE interrupts must be disabled or this ISR
             * would fire continuously.
             */
            usart_disable_tx_interrupt(USART3);
        }
    }
}


/* -------------------------------------------------------------------------- */
/* TinyUSB USB interrupts                                                      */
/* -------------------------------------------------------------------------- */

volatile uint32_t usb_hp_irq_count;
volatile uint32_t usb_lp_irq_count;


void usb_hp_can_tx_isr(void)
{
    usb_hp_irq_count++;
    dcd_int_handler(0);
}


void usb_lp_can_rx0_isr(void)
{
    usb_lp_irq_count++;
    dcd_int_handler(0);
}


/* -------------------------------------------------------------------------- */
/* Clock                                                                      */
/* -------------------------------------------------------------------------- */

static void clock_setup(void)
{
    rcc_clock_setup_pll(
        &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]
    );
}


/* -------------------------------------------------------------------------- */
/* USART3                                                                     */
/* -------------------------------------------------------------------------- */

static void usart3_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART3);

    /*
     * PB10 = USART3_TX
     * PB11 = USART3_RX
     */

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO10
    );

    gpio_set_mode(
        GPIOB,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO11
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

    nvic_enable_irq(NVIC_USART3_IRQ);

    usart_enable_rx_interrupt(USART3);

    usart_enable(USART3);
}


/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    clock_setup();

    usart3_setup();

    /*
     * STM32F103 USB peripheral clock.
     */
    rcc_periph_clock_enable(RCC_USB);

    tusb_init();

    dbg_puts("\r\nSTM32F103 USB CDC\r\n");
    dbg_puts("UART3 IRQ mode\r\n");

    while (1)
    {
        tud_task();


        /*
         * ------------------------------------------------------------------
         * UART3 -> USB CDC
         * ------------------------------------------------------------------
         */

        while (uart3_rx_head != uart3_rx_tail)
        {
            uint8_t c =
                uart3_rx_buf[uart3_rx_tail];

            uart3_rx_tail =
                (uart3_rx_tail + 1) % UART3_RX_BUF_SIZE;

            if (tud_cdc_connected() &&
                tud_cdc_write_available())
            {
                tud_cdc_write_char(c);
            }
        }

        tud_cdc_write_flush();


        /*
         * ------------------------------------------------------------------
         * USB CDC -> UART3
         * ------------------------------------------------------------------
         *
         * No polling of USART3 here.
         * uart3_tx_put() queues the byte and enables TXE IRQ.
         */

        if (tud_cdc_available())
        {
            uint8_t buf[64];

            uint32_t count =
                tud_cdc_read(buf, sizeof(buf));

            for (uint32_t i = 0; i < count; i++)
            {
                uart3_tx_put(buf[i]);
            }
        }
    }
}
