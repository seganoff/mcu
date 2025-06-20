//#pragma once
#ifndef __HAL_H__
#define __HAL_H__
#include "stm32f405xx.h"
#include <stdbool.h>
#include <sys/stat.h>
  
static uint32_t sys_clock_hz;

static volatile uint32_t s_ticks;

static inline void spin(volatile uint32_t count) {while (count--) /*(void) 0;*/asm("nop");}

static inline bool timer_expired(uint32_t *t,uint32_t prd,uint32_t now) {
if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
if (*t > now) return false;                    // Not expired yet, return
*t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
return true;                                   // Expired, return true
}

static inline void rng_init(void){RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;RNG->CR |= RNG_CR_RNGEN;}
/*static inline void systick_init(uint32_t ticks) {
if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
SysTick->LOAD = ticks - 1;
SysTick->VAL = 0;
SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk);//enable
//[16]COUNTFLAG Returns 1 if timer counted to 0 since last time this was read.
//dui0533 0xE000E010 SYST_CSR 
}*/

static inline void clock_init_hse(void){
//SCB_EnableICache(); SCB_EnableDCache();//compiler complains
(*RCC).APB2ENR|=RCC_APB2ENR_SYSCFGEN;
(*RCC).APB1ENR|=RCC_APB1ENR_PWREN;//f7cube autogen pwren then syscfgen
//two funky calls ???
//NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
//NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
//SystemClockConfig from here on, calls before were in main
(*FLASH).ACR|=FLASH_ACR_DCRST;(*FLASH).ACR|=FLASH_ACR_ICRST;
(*FLASH).ACR|=(FLASH_ACR_LATENCY_5WS|FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN);
//(*FLASH).ACR|=(FLASH_ACR_LATENCY_5WS|FLASH_ACR_PRFTEN|FLASH_ACR_ICEN);
while((FLASH->ACR&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_5WS){}
(*PWR).CR|=PWR_CR_VOS;//can check with PWR_CSR_VOSRDY
(*RCC).CR|=RCC_CR_HSEON;
while((RCC->CR&RCC_CR_HSERDY)!=RCC_CR_HSERDY){}
(*RCC).CR|=RCC_CR_CSSON;
//RCC->PLLCFGR=0x24003010;//val after reset
(*RCC).PLLCFGR=((4UL<<RCC_PLLCFGR_PLLM_Pos)|(168UL<<RCC_PLLCFGR_PLLN_Pos)|(9UL<<RCC_PLLCFGR_PLLQ_Pos)|RCC_PLLCFGR_PLLSRC_HSE);
(*RCC).CR|=RCC_CR_PLLON;
while((RCC->CR&RCC_CR_PLLRDY)!=RCC_CR_PLLRDY){}
while((PWR->CSR&PWR_CSR_VOSRDY)!=PWR_CSR_VOSRDY){}
//RCC->CFGR=0x0;//reset
(*RCC).CFGR|=(RCC_CFGR_HPRE_DIV1|RCC_CFGR_PPRE2_DIV2|RCC_CFGR_PPRE1_DIV4);
//(*RCC).CFGR|=RCC_CFGR_PPRE1_DIV4;
//(*RCC).CFGR|=RCC_CFGR_PPRE2_DIV2;
(*RCC).CFGR|=RCC_CFGR_SW_PLL;
while((RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL){}
sys_clock_hz=168000000;//SysTick_Config(sys_clock_hz/1000);
//systick_init(sys_clock_hz/1000);
// systick & SystemCoreClock=216000000
//uint32_t SystemCoreClock;
//SystemCoreClock=216000000;SysTick_Config(SystemCoreClock/1000);
//LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);MODIFY_REG(RCC->DCKCFGR1, RCC_DCKCFGR1_TIMPRE, Prescaler);
}//clock init hse end

//void gpio_config(void);
//void exti_attach(void);
//void SysTick_Handler(void);


int _fstat(int fd, struct stat *st) {
  if (fd < 0) return -1;
  st->st_mode = S_IFCHR;
  return 0;
}
void *_sbrk(int incr) {
extern char _end;static unsigned char *heap = NULL;unsigned char *prev_heap;
if (heap == NULL) heap = (unsigned char *) &_end;
prev_heap = heap;heap += incr;return prev_heap;}
int _open(const char *path) {(void) path; return -1;}
int _close(int fd) {(void) fd;return -1;}
int _isatty(int fd) {(void) fd;return 1;}
int _lseek(int fd, int ptr, int dir) {(void) fd, (void) ptr, (void) dir; return 0;}
void _exit(int status) {(void) status;  for (;;) asm volatile("BKPT #0");}
void _kill(int pid, int sig) { (void) pid, (void) sig;}
int _getpid(void) {return -1;}
int _write(int fd, char *ptr, int len) {
(void) fd, (void) ptr, (void) len;
//  if (fd == 1) uart_write_buf(UART_DEBUG, ptr, (size_t) len);
return -1;}
int _read(int fd, char *ptr, int len) { (void) fd, (void) ptr, (void) len;return -1;}
int _link(const char *a, const char *b) {  (void) a, (void) b; return -1;}
int _unlink(const char *a) {(void) a;return -1;}
int _stat(const char *path, struct stat *st) {(void) path, (void) st; return -1;}
int mkdir(const char *path, mode_t mode) { (void) path, (void) mode; return -1;}
void _init(void) {}//startup.s bl __libc_init_array cpp specific//newlib...

/*interrupts*/
void SysTick_Handler(void) {s_ticks++;}
void EXTI9_5_IRQHandler(void){if((*EXTI).PR&EXTI_PR_PR6){(*EXTI).PR|=EXTI_PR_PR6;(*GPIOB).ODR^=(GPIO_ODR_OD5|GPIO_ODR_OD2);}}
void TIM2_IRQHandler(void){// Handle a timer 'update' interrupt event
if (TIM2->SR & TIM_SR_UIF) {TIM2->SR &= ~(TIM_SR_UIF);
(*GPIOB).ODR^= GPIO_ODR_OD2;}//pb2 toggle
}

void gpio_config(void){
(*RCC).AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
((GPIO_TypeDef *) GPIOB_BASE)->MODER|=GPIO_MODER_MODER5_0;
//(*((GPIO_TypeDef *) GPIOB_BASE)).OSPEEDR|=(GPIO_OSPEEDR_OSPEED5_0|GPIO_OSPEEDR_OSPEED5_1);//11 very high
(*GPIOB).OTYPER&=~GPIO_OTYPER_OT5;//out push pull
(*GPIOB).PUPDR&=~(GPIO_PUPDR_PUPD5_0|GPIO_PUPDR_PUPD5_1);//b5out,no pulls
(*GPIOB).MODER&=~(GPIO_MODER_MODER6_0|GPIO_MODER_MODER6_1);
(*GPIOB).PUPDR|=GPIO_PUPDR_PUPD6_0;//b6 in, with pullUP
(*GPIOB).MODER|=GPIO_MODER_MODER2_0;//b2out,pushPull,lowSpeed,noPulls
//(*GPIOB).ODR|=GPIO_ODR_OD2;
}
void exti_attach(void){
(*SYSCFG).EXTICR[1]|=SYSCFG_EXTICR2_EXTI6_PB;//exticr2 pin 4..7
(*EXTI).IMR|=EXTI_IMR_MR6;//exti6_PB & IMR_MR6
(*EXTI).FTSR|=EXTI_FTSR_TR6;
NVIC_SetPriority(EXTI9_5_IRQn,0x03);
NVIC_EnableIRQ(EXTI9_5_IRQn);
NVIC_SetPriority(TIM2_IRQn,0x03);
NVIC_EnableIRQ(TIM2_IRQn);
}

void SystemInit(void){
(*SCB).CPACR|=((3UL<<10*2)|(3UL<<11*2));//enable cp10 cp11 full access(FPU)
//clock_init_hse();//+calls systick_init
//NVIC_EnableIRQ(SysTick_IRQn);
}

void start_tim2(void){
(*RCC).APB1ENR|=RCC_APB1ENR_TIM2EN;
(*TIM2).CR1&=~(TIM_CR1_CEN);
(*RCC).APB1RSTR|=RCC_APB1RSTR_TIM2RST;(*RCC).APB1RSTR&=~(RCC_APB1RSTR_TIM2RST);
(*TIM2).PSC=sys_clock_hz/1000;//1ms
(*TIM2).ARR=200;//200ms
(*TIM2).EGR=TIM_EGR_UG;(*TIM2).DIER=TIM_DIER_UIE;
(*TIM2).CR1|=TIM_CR1_CEN;
}
void stop_tim2(void){(*TIM2).CR1&=~(TIM_CR1_CEN);(*TIM2).SR&=~(TIM_SR_UIF);}














#endif // __HAL_H__
