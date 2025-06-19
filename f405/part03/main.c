#include "stm32f405xx.h"

volatile uint8_t led_on/*=0*/;
void SystemInit(void){/*(*RCC).APB2ENR|=RCC_APB2ENR_SYSCFGEN;*/led_on=0x0;}
void _init(void) {}
void EXTI9_5_IRQHandler(void){
if((*EXTI).PR&EXTI_PR_PR6){(*EXTI).PR|=EXTI_PR_PR6;led_on=!led_on;(*GPIOB).ODR^=GPIO_ODR_OD5;}
}
#if (0) //compiler will complain
int main_part2(void) {
//volatile int val = 0;
/*volatile*/ uint8_t botton_abajo = 0x0;
///*volatile*/ uint32_t idr_value = 0x0;
//gpio on ahb1 
//RCC_AHB1ENR.Rval 0x0010_0000 ink244
//(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
//(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
//b5out 3,3 to leds
((GPIO_TypeDef *) GPIOB_BASE)->MODER|=GPIO_MODER_MODER5_0;/*0x0400*/
(*GPIOB).OTYPER&=~GPIO_OTYPER_OT5;//0pushpull
//(*((GPIO_TypeDef *) GPIOB_BASE)).OSPEEDR|=(GPIO_OSPEEDR_OSPEED5_0|GPIO_OSPEEDR_OSPEED5_1);//11 very high
(*GPIOB).PUPDR&=~(GPIO_PUPDR_PUPD5_0|GPIO_PUPDR_PUPD5_1);//00 no pulls
(*GPIOB).ODR|=GPIO_ODR_OD5;
//b6 in pullup switch if 0(button) 
(*GPIOB).MODER&=~(GPIO_MODER_MODER6_0|GPIO_MODER_MODER6_1);//00 input
(*GPIOB).PUPDR|=GPIO_PUPDR_PUPD6_0;//01 pullUp, cuz grounded
while (1){
uint32_t idr_value = ~((*GPIOB).IDR);
if(idr_value&GPIO_IDR_ID6){if(!botton_abajo)(*GPIOB).ODR^=GPIO_ODR_OD5;botton_abajo=0x1;}
else botton_abajo=0x0;
//for(int i=0;i<100;i++)asm("nop");
}//while 1
}//main end
#endif
//def button_pin 1 ||| led_pin 3//main.h  Ω2126 ⏚ 23da
//pb02-5.1kΩ-D1Blue-⏚
//pc13-330R5-0 0-Button-Vdd33
//'Push-pull' outputs can pull a pin to either 1 or 0 while 
//open-drain outputs can only pull the pin to 0
//open-drain outputs are useful if you have multiple devices set to output on the same wire, 
//because it prevents different devices from trying to pull the signal in two different directions at once and damaging each other
// 430 typedef struct
// 431 { 
//  432   __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
//  433   __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
//  434   __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
//  435   __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
//  436   __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
//  437   __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
//  438   __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
//  439   __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
//  440   __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
//  441 } GPIO_TypeDef;
//x=Port y=pin, !!FU!! stm refManual
/*ds 3.0.11
The external interrupt/event controller consists of 23 edge-detector lines used to generate
interrupt/event requests. Each line can be independently configured to select the trigger
event (rising edge, falling edge, both) and can be masked independently. A pending register
maintains the status of the interrupt requests. The EXTI can detect an external line with a
pulse width shorter than the Internal APB2 clock period. Up to 140 GPIOs can be connected
to the 16 external interrupt lines.*/
//typedef struct
//  347 { 
//  348   __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
//  349   __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
//  350   __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
//  351   __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
//  352   __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
//  353   __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
//  354 } EXTI_TypeDef;
//typedef struct
//  448 {
//  449   __IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
//  450   __IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
//  451   __IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
//  452   uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
//  453   __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
//  454 } SYSCFG_TypeDef;
int main(void){
(*RCC).APB2ENR|=RCC_APB2ENR_SYSCFGEN;
//(*SYSCFG).EXTICR[2]|=SYSCFG_EXTICR2_EXTI6_PB;//exticr2 pin 4..7
//(*EXTI).IMR|=EXTI_IMR_MR6;//exti6_PB & IMR_MR6
//(*EXTI).FTSR|=EXTI_FTSR_TR6;
//NVIC_SetPriority(EXTI9_5_IRQn,0x03);
//NVIC_EnableIRQ(EXTI9_5_IRQn);

(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
((GPIO_TypeDef *) GPIOB_BASE)->MODER|=GPIO_MODER_MODER5_0;/*0x0400*/
(*GPIOB).OTYPER&=~GPIO_OTYPER_OT5;//0pushpull
(*GPIOB).PUPDR&=~(GPIO_PUPDR_PUPD5_0|GPIO_PUPDR_PUPD5_1);//00 no pulls
//(*GPIOB).ODR|=GPIO_ODR_OD5;
//b6 in pullup switch if 0(button) 
(*GPIOB).MODER&=~(GPIO_MODER_MODER6_0|GPIO_MODER_MODER6_1);//00 input
(*GPIOB).PUPDR|=GPIO_PUPDR_PUPD6_0;//01 pullUp, cuz grounded

(*SYSCFG).EXTICR[1]|=SYSCFG_EXTICR2_EXTI6_PB;//exticr2 pin 4..7
(*EXTI).IMR|=EXTI_IMR_MR6;//exti6_PB & IMR_MR6
(*EXTI).FTSR|=EXTI_FTSR_TR6;
NVIC_SetPriority(EXTI9_5_IRQn,0x03);
NVIC_EnableIRQ(EXTI9_5_IRQn);
while(1){
if(led_on){}
else{}
}



}
