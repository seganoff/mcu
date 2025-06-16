#include "stm32f405xx.h"

void SystemInit(void){}
void _init(void) {}

/**
 * Main program.
 */
int main(void) {
//volatile int val = 0;
while (1) {
//gpio on ahb1 
(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOCEN;

}//while 1
}//main end

//def button_pin 1 ||| led_pin 3//main.h  Ω2126 ⏚ 23da
//pb02-5.1kΩ-D1Blue-⏚
//pc13-330R5-0 0-Button-Vdd33
