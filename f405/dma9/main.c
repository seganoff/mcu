#include "hal.h"

//volatile uint8_t led_on/*=0*/;
//void SystemInit(void){/*(*RCC).APB2ENR|=RCC_APB2ENR_SYSCFGEN;*/led_on=0x0;}
//void _init(void) {}
//void EXTI9_5_IRQHandler(void){if((*EXTI).PR&EXTI_PR_PR6){(*EXTI).PR|=EXTI_PR_PR6;led_on=!led_on;(*GPIOB).ODR^=GPIO_ODR_OD5;}}
//void SysTick_Handler(void) {s_ticks++;}

int main(void){
clock_init_hse();
//uint32_t retVal = SysTick_Config(sys_clock_hz/1000);
//(void) retVal;
/*volatile*/// uint32_t timerB2=0,periodB2=500;//in ms
gpio_config();
exti_attach();
start_tim2();
dma_sin();
while(1){

//volatile uint32_t now=//get_millis();//s_ticks;
//if( timer_expired(&timerB2,periodB2,s_ticks) )
//{
//(*GPIOB).ODR^=GPIO_ODR_OD2;
//(*GPIOB).ODR^=GPIO_ODR_OD5;
//}
//else{}
}

return 0;}//main end
