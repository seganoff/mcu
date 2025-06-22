#include "hal.h"

uint8_t send[] = "sending\r\n";
int main(void){
clock_init_hse();
uart_init(USART3,115200);
printf("clock init done!!!\r\n");
//gpio_config();
//gpio_output(PIN('B',5));
gpio_init(PIN('B',5), GPIO_MODE_OUTPUT,GPIO_OTYPE_PUSH_PULL,GPIO_SPEED_LOW,GPIO_PULL_NONE,0);
gpio_output(PIN('B',2));
gpio_init(PIN('B',6), GPIO_MODE_INPUT,GPIO_OTYPE_PUSH_PULL,GPIO_SPEED_LOW,GPIO_PULL_UP,0);
exti_attach();
start_tim2();
while(1){
//uart_write_buf(USART3, (char*)send, (sizeof(send)/sizeof(uint8_t)));

//volatile uint32_t now=//get_millis();//s_ticks;
//if( timer_expired(&timerB2,periodB2,s_ticks) )
//{
//(*GPIOB).ODR^=GPIO_ODR_OD2;
//(*GPIOB).ODR^=GPIO_ODR_OD5;
//}
//else{}
}

return 0;}//main end
