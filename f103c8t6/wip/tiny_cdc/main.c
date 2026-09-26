#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/nvic.h>

#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idle_task_tcb;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

volatile uint32_t usb_irq_count;

void usb_lp_can_rx0_isr(void)
{
    usb_irq_count++;
    dcd_int_handler(0);
}

void usb_hp_can_tx_isr(void)
{
    usb_irq_count++;
    dcd_int_handler(0);
}


/* ------------------------------------------------------------------------- */
/* UART3 stream buffers                                                      */
/* ------------------------------------------------------------------------- */

#define UART3_RX_BUF_SIZE 256
#define UART3_TX_BUF_SIZE 256

static uint8_t uart3_rx_storage[UART3_RX_BUF_SIZE];
static uint8_t uart3_tx_storage[UART3_TX_BUF_SIZE];

static StaticStreamBuffer_t uart3_rx_stream_struct;
static StaticStreamBuffer_t uart3_tx_stream_struct;

static StreamBufferHandle_t uart3_rx_stream;
static StreamBufferHandle_t uart3_tx_stream;


/* ------------------------------------------------------------------------- */
/* USART3 interrupt                                                          */
/* ------------------------------------------------------------------------- */

void usart3_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    /*
     * RXNE: hardware has received a byte.
     */
    if (usart_get_flag(USART3, USART_SR_RXNE))
    {
        uint8_t c = usart_recv(USART3);

        /*
         * One writer: this ISR.
         * One reader: USB task.
         */
        xStreamBufferSendFromISR(
            uart3_rx_stream,
            &c,
            1,
            &higher_priority_task_woken
        );
    }

    /*
     * TXE: UART transmit register is ready for another byte.
     */
    if (usart_get_flag(USART3, USART_SR_TXE))
    {
        uint8_t c;

        size_t n = xStreamBufferReceiveFromISR(
            uart3_tx_stream,
            &c,
            1,
            &higher_priority_task_woken
        );

        if (n == 1)
        {
            usart_send(USART3, c);
        }
        else
        {
            /*
             * Nothing left to transmit.
             *
             * Do NOT leave TXE interrupt enabled or we would continuously
             * enter this ISR while the UART is idle.
             */
            usart_disable_tx_interrupt(USART3);
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* ------------------------------------------------------------------------- */
/* Clock                                                                     */
/* ------------------------------------------------------------------------- */

static void clock_setup(void)
{
    rcc_clock_setup_pll(
        &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]
    );
}


/* ------------------------------------------------------------------------- */
/* USART3                                                                    */
/* ------------------------------------------------------------------------- */

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
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    nvic_enable_irq(NVIC_USART3_IRQ);

    /*
     * Initially only RX interrupts are enabled.
     *
     * TXE gets enabled when something is actually placed in the TX
     * StreamBuffer.
     */
    usart_enable_rx_interrupt(USART3);

    usart_enable(USART3);
}


/* ------------------------------------------------------------------------- */
/* USB task                                                                  */
/* ------------------------------------------------------------------------- */

static void usb_task(void *arg)
{
    (void)arg;

    uint8_t buf[64];

    for (;;)
    {
        /*
         * TinyUSB device state machine.
         */
        tud_task();


        /* ------------------------------------------------------------- */
        /* UART3 -> USB CDC                                               */
        /* ------------------------------------------------------------- */

        if (tud_cdc_connected())
        {
            size_t count;

            do
            {
                count = xStreamBufferReceive(
                    uart3_rx_stream,
                    buf,
                    sizeof(buf),
                    0
                );

                if (count != 0 &&
                    tud_cdc_write_available() >= count)
                {
                    tud_cdc_write(buf, count);
                }
                else if (count != 0)
                {
                    /*
                     * CDC TX FIFO is temporarily full.
                     *
                     * For this first test, don't build another buffering
                     * layer here. Put the bytes back isn't supported by a
                     * StreamBuffer, so only receive what CDC can accept.
                     *
                     * Normally we'd size/drain the USB side differently.
                     */
                    for (size_t i = 0; i < count; i++)
                    {
                        if (tud_cdc_write_available())
                            tud_cdc_write_char(buf[i]);
                        else
                            break;
                    }
                }

            } while (count != 0);

            tud_cdc_write_flush();
        }


        /* ------------------------------------------------------------- */
        /* USB CDC -> UART3                                               */
        /* ------------------------------------------------------------- */

        while (tud_cdc_available())
        {
            size_t count = tud_cdc_read(
                buf,
                sizeof(buf)
            );

            if (count == 0)
                break;

            size_t sent = xStreamBufferSend(
                uart3_tx_stream,
                buf,
                count,
                0
            );

            /*
             * Something has entered the TX stream.
             * Kick the UART TX interrupt.
             *
             * TXE will cause usart3_isr() to pull bytes from the stream.
             */
            if (sent != 0)
            {
                taskENTER_CRITICAL();
                usart_enable_tx_interrupt(USART3);
                taskEXIT_CRITICAL();
            }

            /*
             * If the stream is full, the remaining CDC data stays in
             * TinyUSB's RX buffering and will be consumed on the next
             * iteration.
             */
            if (sent != count)
                break;
        }


        /*
         * Give other tasks/interrupt-driven work some CPU time.
         *
         * tud_task() remains non-blocking, so this is intentionally short.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/* ------------------------------------------------------------------------- */
/* main                                                                      */
/* ------------------------------------------------------------------------- */

int main(void)
{
    clock_setup();

    usart3_setup();

    /*
     * STM32F103 USB peripheral clock.
     *
     * This was the missing piece that prevented enumeration earlier.
     */
    rcc_periph_clock_enable(RCC_USB);


    /* ------------------------------------------------------------------ */
    /* FreeRTOS StreamBuffers                                             */
    /* ------------------------------------------------------------------ */

    uart3_rx_stream = xStreamBufferCreateStatic(
        UART3_RX_BUF_SIZE,
        1,
        uart3_rx_storage,
        &uart3_rx_stream_struct
    );

    uart3_tx_stream = xStreamBufferCreateStatic(
        UART3_TX_BUF_SIZE,
        1,
        uart3_tx_storage,
        &uart3_tx_stream_struct
    );

    configASSERT(uart3_rx_stream != NULL);
    configASSERT(uart3_tx_stream != NULL);


    /* ------------------------------------------------------------------ */
    /* TinyUSB                                                            */
    /* ------------------------------------------------------------------ */

    tusb_init();


    /* ------------------------------------------------------------------ */
    /* USB/serial bridge task                                             */
    /* ------------------------------------------------------------------ */

    BaseType_t result = xTaskCreate(
        usb_task,
        "usb",
        256,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );
if (result != pdPASS)
{
    /* task creation failed */
    for (;;)
        ;
}

    configASSERT(result == pdPASS);


    /* ------------------------------------------------------------------ */
    /* Go                                                                  */
    /* ------------------------------------------------------------------ */

    vTaskStartScheduler();


    /*
     * We should never get here.
     */
    for (;;)
        ;
}

