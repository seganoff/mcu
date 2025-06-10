#include "hal.h"

//uint32_t SystemCoreClock = SYS_FREQUENCY;
//hal.h > stm32f767xx.h > system_stm32f7xx.h: system_exported_functions\/
//extern void SystemInit(void);extern void SystemCoreClockUpdate(void);
//Called from cmsis_f7/Source/Templates/gcc/startup_stm32f767xx.s:61 bl SystemInit
void cpqHSI(void) {
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // Enable FPU
  FLASH->ACR |= FLASH_LATENCY | BIT(8) | BIT(9);      // Flash latency, prefetch
  RCC->PLLCFGR &= ~((BIT(17) - 1));                   // Clear PLL multipliers
  RCC->PLLCFGR |= (((PLL_P - 2) / 2) & 3) << 16;      // Set PLL_P
  RCC->PLLCFGR |= PLL_M | (PLL_N << 6);               // Set PLL_M and PLL_N
  RCC->CR |= BIT(24);                                 // Enable PLL
  while ((RCC->CR & BIT(25)) == 0) spin(1);           // Wait until done
  RCC->CFGR = (APB1_PRE << 10) | (APB2_PRE << 13);    // Set prescalers
  RCC->CFGR |= 2;                                     // Set clock source to PLL
  while ((RCC->CFGR & 12) == 0) spin(1);              // Wait until done
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
}

//blog.embeddedexpert.io/?p=531
void p531(void)//void SysClockConfig(void) //set the core frequency to 216MHz
{
__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
RCC->CR |= RCC_CR_CSSON;
RCC->CR |= RCC_CR_HSEBYP;
RCC->CR |= ((uint32_t)RCC_CR_HSEON);
do{HSEStatus = RCC->CR & RCC_CR_HSERDY;StartUpCounter++;} 
while((HSEStatus == 0) && (StartUpCounter != 3000));
if ((RCC->CR & RCC_CR_HSERDY) != 0/*RESET*/) HSEStatus = (uint32_t)0x01;
else HSEStatus = (uint32_t)0x00;

if (HSEStatus == (uint32_t)0x01){
RCC->APB1ENR |= RCC_APB1ENR_PWREN;
PWR->CR1 &= (uint32_t)~(PWR_CR1_VOS);
RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
RCC->PLLCFGR = PLL_M | (PLL_N << RCC_PLLCFGR_PLLN_Pos) | (((PLL_P >> 1) -1) << RCC_PLLCFGR_PLLP_Pos) |
           (RCC_PLLCFGR_PLLSRC_HSE);
RCC->CR |= RCC_CR_PLLON;
while((RCC->CR & RCC_CR_PLLRDY) == 0){}
/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
FLASH->ACR = FLASH_ACR_LATENCY_7WS;
/* Select the main PLL as system clock source */
RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
RCC->CFGR |= RCC_CFGR_SW_PLL;
/* Wait till the main PLL is used as system clock source */
while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
{;}
}//status 0x1
else
{ /* If HSE fails to start-up, the application will have wrong clock
  configuration. User can add here some code to deal with this error */
}
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
//SystemCoreClockUpdate();
}//p531

static inline void clock_init_reworked(void){
SCB_EnableICache(); SCB_EnableDCache();
RCC->APB1ENR|=RCC_APB1ENR_PWREN;
RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;
FLASH->ACR|=(FLASH_LATENCY|FLASH_ACR_PRFTEN|FLASH_ACR_ARTEN);
while((FLASH->ACR&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_7WS){}
PWR->CR1|=(PWR_CR1_VOS_0|PWR_CR1_VOS_1|PWR_CR1_ODEN);
RCC->CR|=(RCC_CR_HSEBYP|RCC_CR_HSEON);
while((RCC->CR & RCC_CR_HSERDY)!=RCC_CR_HSERDY){}
RCC->CR|=RCC_CR_CSSON;
RCC->PLLCFGR=0x24003010;
RCC->PLLCFGR=/*0x29403604;*/ (RCC_PLLCFGR_PLLSRC_HSE)      /*0x0040_0000 0x0040_0000 */
| (PLL_M<<RCC_PLLCFGR_PLLM_Pos)              /*0x0000_0004 0x0040_0004 */
| (PLL_N << RCC_PLLCFGR_PLLN_Pos)            /*0x0000_3600 0x0040_3604 */
| (((PLL_P >> 1) -1) << RCC_PLLCFGR_PLLP_Pos)/*0x0000_0000 0x0040_3604 2:00;4:01;6:10;8:11*/
| (PLL_Q<<RCC_PLLCFGR_PLLQ_Pos)              /*0x0900_0000 0x0940_3604 */
| (PLL_R<<RCC_PLLCFGR_PLLR_Pos)              /*0x2000_0000 0x2940_3604.assert_equals(RCC_PLLCFGR) */
;
RCC->CR|=RCC_CR_PLLON;
while((RCC->CR&RCC_CR_PLLRDY)!=RCC_CR_PLLRDY){}
while((PWR->CSR1&PWR_CSR1_VOSRDY)!=PWR_CSR1_VOSRDY){}
RCC->CFGR=0x0;
RCC->CFGR|=RCC_CFGR_HPRE_DIV1;
RCC->CFGR|=RCC_CFGR_PPRE1_DIV4;
RCC->CFGR|=RCC_CFGR_PPRE2_DIV2;
RCC->CFGR|=RCC_CFGR_SW_PLL;
while((RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL){}
// systick & SystemCoreClock=216000000
//uint32_t SystemCoreClock;
//SystemCoreClock=216000000;SysTick_Config(SystemCoreClock/1000);
//LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);MODIFY_REG(RCC->DCKCFGR1, RCC_DCKCFGR1_TIMPRE, Prescaler);
}//clock init reworked end

void SystemInit(void){
//cpqHSI();
//locm();
//ll_template();
//p531();
//clock_init();// > hal.h
SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));//enable cp10 cp11 full access(FPU)
__DSB();__ISB();//data/instr sync barrier cmsis_armcc.h:430
clock_init_reworked();
//cpqHSI();
}

