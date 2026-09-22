#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

#include "usbcdc.h"
static void uart3_init(void)
{
    //rcc_periph_clock_enable(RCC_GPIOB);
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
}

static void uart3_putc(char c)
{usart_send_blocking(USART3, c);}

static void uart3_puts(const char *s)
{while (*s)uart3_putc(*s++);}
//

//static void counter(void *arg __attribute__((unused))) {
////int userChar = usb_getc();//user connect
//uint8_t data0[] = {'a','b','c','d','e','f'};
//uint8_t out[8];
////uint32_t frame_length = encode_frame(data0,6,out);
//
//while(1)
//{
//usb_write(out,8);
////for(int i=0;i<frame_length; i++){
//  //xQueueSend(usb_txq,buf,portMAX_DELAY);
//  //usb_putc(out[i]);
//  //vTaskDelay(pdMS_TO_TICKS(400));
////}
////ch_= (ch_ + 1) % 140;
//vTaskDelay(pdMS_TO_TICKS(400));
////usb_puts("new input \n");//userChar = usb_getc();
//}//while 1 end
//}
//
static void flasher(void *arg __attribute__((unused))) {
while(1)//for (;;) 
{
//gpio_set(GPIOC,GPIO13);
//vTaskDelay(pdMS_TO_TICKS(2000));
//gpio_toggle(GPIOB,GPIO2);
vTaskDelay(pdMS_TO_TICKS(600));
//uart3_puts("debug: toggle next\r\n");
uart3_puts("tick\r\n");
usb_puts("debug: con rn \r\n");
usb_puts("debug: con n \n");
usb_puts("debug: sin rn");
}
}

//int main(void) __attribute__ ((noreturn)); > warning, implicit main return
int main(void){
//rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]); //f4

//rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"
rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
rcc_periph_clock_enable(RCC_GPIOC);
rcc_periph_clock_enable(RCC_GPIOB);
gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO2);
gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

//gpio_set(GPIOB,GPIO2);
uart3_init();

uart3_puts("init done: \n");
uart3_puts("start usb next\r\n");
uart3_putc('\r');uart3_putc('\n');

//gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
//gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO14);
//gpio_clear(GPIOC,GPIO14);

//usbcdc.c:413 xTaskCreate(usb_task,"USB",200,udev,configMAX_PRIORITIES-1,NULL);
usb_start(true,configMAX_PRIORITIES-1);

//xTaskCreate(adventure,"game",300,NULL,configMAX_PRIORITIES-1,NULL);
xTaskCreate(flasher,"flash",  100,NULL,configMAX_PRIORITIES-1,NULL);
//xTaskCreate(counter,"counter",100,NULL,configMAX_PRIORITIES-1,NULL);

vTaskStartScheduler();
for (;;);
//return 0;//noreturn
}

// End main.c
