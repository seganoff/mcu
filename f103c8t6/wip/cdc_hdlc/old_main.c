#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

#include "usbcdc.h"
static QueueHandle_t txq3;
static QueueHandle_t rxq3;
static QueueHandle_t txq2;
static QueueHandle_t rxq2;
static void uart_init(void)
{
rcc_periph_clock_enable(RCC_GPIOB);
rcc_periph_clock_enable(RCC_USART3);
//* PB10 = USART3_TX
gpio_set_mode(
GPIOB,
GPIO_MODE_OUTPUT_50_MHZ,
GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
GPIO_USART3_TX //GPIO10
);
// PB11 = USART3_RX, if you need RX
gpio_set_mode(
GPIOB,
GPIO_MODE_INPUT,
GPIO_CNF_INPUT_FLOAT,
GPIO_USART3_RX //GPIO11
);
usart_set_baudrate(USART3, 115200);
usart_set_databits(USART3, 8);
usart_set_stopbits(USART3, USART_STOPBITS_1);
usart_set_mode(USART3, USART_MODE_TX_RX);
usart_set_parity(USART3, USART_PARITY_NONE);
usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
usart_enable(USART3);
txq3= xQueueCreate(256,sizeof(uint8_t));
rxq3= xQueueCreate(256,sizeof(uint8_t));
//--------------------
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
usart_set_baudrate(USART2, 115200);
usart_set_databits(USART2, 8);
usart_set_stopbits(USART2, USART_STOPBITS_1);
usart_set_mode(USART2, USART_MODE_TX_RX);
usart_set_parity(USART2, USART_PARITY_NONE);
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
usart_enable(USART2);
txq2= xQueueCreate(256,sizeof(uint8_t));
rxq2= xQueueCreate(256,sizeof(uint8_t));
}

static void uart_putc(uint32_t uart,char c)
{usart_send_blocking(uart, c);}

static void uart_puts(uint32_t uart,const char *s)
{while (*s)uart_putc(uart,*s++);}
//
static bool uart_putc_nb(uint32_t uart, char c)
{
if (!(USART_SR(uart) & USART_SR_TXE)) return false;
usart_send(uart, c);//non blocking
return true;
}
static bool uart_puts_nb(uint32_t uart, const char *s)
{
    while (*s) {
        if (!uart_putc_nb(uart, *s))
            return false;
        ++s;
    }
    return true;
}
// -1 none avil ; 0..255 chars received
static int uart_getc_nb(uint32_t uart)
{
if (!(USART_SR(uart) & USART_SR_RXNE))
return -1;
return usart_recv(uart);
}



/*********************************************************************
 * USART Task: 
// *********************************************************************/
static void uart_task(void *arg)
{
uint32_t uart = (uint32_t)arg;
for (;;) {
int c = uart_getc_nb(uart);
//rcv 1 char & send it into queue
if (c >= 0) { xQueueSend(rxq3,&c,0);
//Echo received character.
//uart_puts_nb(uart,"\r\nwent into rxq3\r\n");
//uart_putc_nb(uart, (char)c);
//uart_puts_nb(uart,"\r\n");
}
int ch;
//tx one char, taken from the txqueue
if (xQueueReceive(txq3, &ch, 0) == pdPASS) {
//usart_send(uart, ch);
uart_putc_nb(uart,ch);
}

// Don't spin at 100% CPU.
vTaskDelay(pdMS_TO_TICKS(1));
}//for
}


//
//
static void flasher(void *arg __attribute__((unused))) {
uart_puts(USART3,"tick from Flashertask (usart 3) \r\n");
while(1)//for (;;) 
{
//gpio_set(GPIOC,GPIO13);
//vTaskDelay(pdMS_TO_TICKS(2000));
gpio_toggle(GPIOB,GPIO2);
vTaskDelay(pdMS_TO_TICKS(600));
//uart3_puts("debug: toggle next\r\n");
//uart_puts(USART3,"tick from task (usart 3) \r\n");
//usb_puts("tick on usb \r\n");//blocks
}
}

//int main(void) __attribute__ ((noreturn)); > warning, implicit main return
int main(void){
//rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]); //f4

//rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"
rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
rcc_periph_clock_enable(RCC_GPIOA);
rcc_periph_clock_enable(RCC_GPIOB);
rcc_periph_clock_enable(RCC_GPIOC);
gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO2);
gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

//gpio_set(GPIOB,GPIO2);
uart_init();

uart_puts(USART3,"uart init done: \r\n");
//uart3_puts("start usb next\r\n");
//uart3_putc('\r');uart3_putc('\n');

//gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
//gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO14);
//gpio_clear(GPIOC,GPIO14);

//usbcdc.c:413 xTaskCreate(usb_task,"USB",200,udev,configMAX_PRIORITIES-1,NULL);
usb_start(true,configMAX_PRIORITIES-1);

//xTaskCreate(adventure,"game",300,NULL,configMAX_PRIORITIES-1,NULL);
xTaskCreate(flasher,"flash",  100,NULL,configMAX_PRIORITIES-1,NULL);
//xTaskCreate(counter,"counter",100,NULL,configMAX_PRIORITIES-1,NULL);
xTaskCreate(
    uart_task,
    "uart3",
    256,//?
    (void *)USART3,//arg*
configMAX_PRIORITIES-1, //2,//prio
    NULL//?
);


vTaskStartScheduler();
for (;;);
//return 0;//noreturn
}

// End main.c
