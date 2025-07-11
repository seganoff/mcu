//#pragma once
#ifndef __HAL_H__
#define __HAL_H__
#include "stm32f405xx.h"
#include <stdbool.h>
#include <stdio.h>
#include <sys/stat.h>

//#include "FreeRTOS.h"
//#include "task.h"

#define BIT(x) (1UL << (x))
#define CLRSET(reg, clear, set) ((reg) = ((reg) & ~(clear)) | (set))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 0xff) //10 to 16
#define PINBANK(pin) (pin >> 8)
//AHB1PERIPH_BASE #define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000UL),periph base 0x40000000UL 
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };
#define GPIO(N) ((GPIO_TypeDef *) (/*0x40020000*/AHB1PERIPH_BASE + 0x400 * (N)))
static GPIO_TypeDef *gpio_bank(uint16_t pin) {return GPIO(PINBANK(pin));}
static inline void gpio_toggle(uint16_t pin) {
GPIO_TypeDef *gpio = gpio_bank(pin);
uint32_t mask = BIT(PINNO(pin));
gpio->BSRR = mask << (gpio->ODR & mask ? 16 : 0);}
static inline int gpio_read(uint16_t pin) {return gpio_bank(pin)->IDR & BIT(PINNO(pin)) ? 1 : 0;}
static inline void gpio_write(uint16_t pin, bool val) {
GPIO_TypeDef *gpio = gpio_bank(pin);
gpio->BSRR = BIT(PINNO(pin)) << (val ? 0 : 16);}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
GPIO_TypeDef *gpio = gpio_bank(pin);uint8_t n = (uint8_t) (PINNO(pin));
RCC->AHB1ENR |= BIT(PINBANK(pin));  // Enable GPIO clock
CLRSET(gpio->OTYPER, 1UL << n, ((uint32_t) type) << n);
CLRSET(gpio->OSPEEDR, 3UL << (n * 2), ((uint32_t) speed) << (n * 2));
CLRSET(gpio->PUPDR, 3UL << (n * 2), ((uint32_t) pull) << (n * 2));
CLRSET(gpio->AFR[n >> 3], 15UL << ((n & 7) * 4),((uint32_t) af) << ((n & 7) * 4));
CLRSET(gpio->MODER, 3UL << (n * 2), ((uint32_t) mode) << (n * 2));}
static inline void gpio_input(uint16_t pin) {
gpio_init(pin, GPIO_MODE_INPUT,GPIO_OTYPE_PUSH_PULL,GPIO_SPEED_HIGH,GPIO_PULL_NONE, 0);}
static inline void gpio_output(uint16_t pin) {
gpio_init(pin, GPIO_MODE_OUTPUT,GPIO_OTYPE_PUSH_PULL,GPIO_SPEED_HIGH,GPIO_PULL_NONE, 0);}

static uint32_t sys_clock_hz;

static volatile uint32_t s_ticks;

static inline void spin(volatile uint32_t count) {while (count--) /*(void) 0;*/asm("nop");}

static inline bool timer_expired(uint32_t *t,uint32_t prd,uint32_t now) {
if (now + prd < *t) *t = 0;
if(*t==0) *t =now+prd;
if(*t > now) return false;
*t =(now - *t) > prd ? now+prd : *t +prd; return true;}

static inline void rng_init(void){RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;RNG->CR |= RNG_CR_RNGEN;}

static inline void clock_init_hse(void){
(*RCC).APB2ENR|=RCC_APB2ENR_SYSCFGEN;
(*RCC).APB1ENR|=RCC_APB1ENR_PWREN;
(*FLASH).ACR|=FLASH_ACR_DCRST;(*FLASH).ACR|=FLASH_ACR_ICRST;
(*FLASH).ACR|=(FLASH_ACR_LATENCY_5WS|FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_PRFTEN);
while((FLASH->ACR&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_5WS){}
(*PWR).CR|=PWR_CR_VOS;
(*RCC).CR|=RCC_CR_HSEON;
while((RCC->CR&RCC_CR_HSERDY)!=RCC_CR_HSERDY){}
(*RCC).CR|=RCC_CR_CSSON;
(*RCC).PLLCFGR=((4UL<<RCC_PLLCFGR_PLLM_Pos)|(168UL<<RCC_PLLCFGR_PLLN_Pos)|(9UL<<RCC_PLLCFGR_PLLQ_Pos)|RCC_PLLCFGR_PLLSRC_HSE);
(*RCC).CR|=RCC_CR_PLLON;
while((RCC->CR&RCC_CR_PLLRDY)!=RCC_CR_PLLRDY){}
while((PWR->CSR&PWR_CSR_VOSRDY)!=PWR_CSR_VOSRDY){}
(*RCC).CFGR|=(RCC_CFGR_HPRE_DIV1|RCC_CFGR_PPRE2_DIV2|RCC_CFGR_PPRE1_DIV4);
(*RCC).CFGR|=RCC_CFGR_SW_PLL;
while((RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL){}
sys_clock_hz=168000000;
#if !defined(FREERTOS_CONFIG_H)
SysTick_Config(sys_clock_hz/1000);
#endif
}//clock init hse end

int _fstat(int fd, struct stat *st) {
if (fd < 0) return -1;
st->st_mode = S_IFCHR;return 0;}
void *_sbrk(int incr) {
extern char _end;static unsigned char *heap = NULL;unsigned char *prev_heap;
if (heap == NULL) heap = (unsigned char *) &_end;
prev_heap = heap;heap += incr;return prev_heap;}//_sbrk
int _open(const char *path) {(void) path; return -1;}
int _close(int fd) {(void) fd;return -1;}
int _isatty(int fd) {(void) fd;return 1;}
int _lseek(int fd, int ptr, int dir) {(void) fd, (void) ptr, (void) dir; return 0;}
void _exit(int status) {(void) status;  for (;;) asm volatile("BKPT #0");}
void _kill(int pid, int sig) { (void) pid, (void) sig;}
int _getpid(void) {return -1;}
int _read(int fd, char *ptr, int len) { (void) fd, (void) ptr, (void) len;return -1;}
int _link(const char *a, const char *b) {  (void) a, (void) b; return -1;}
int _unlink(const char *a) {(void) a;return -1;}
int _stat(const char *path, struct stat *st) {(void) path, (void) st; return -1;}
int mkdir(const char *path, mode_t mode) { (void) path, (void) mode; return -1;}
void _init(void) {}//startup.s bl __libc_init_array cpp specific//newlib...

/*interrupts*/
void SysTick_Handler(void) {s_ticks++;}
void EXTI9_5_IRQHandler(void){
if((*EXTI).PR&EXTI_PR_PR6){(*EXTI).PR|=EXTI_PR_PR6;gpio_toggle(PIN('B',5));//printf("exti9_5 ticks:0x%lx",s_ticks);
}}
void TIM2_IRQHandler(void){// Handle a timer 'update' interrupt event
if (TIM2->SR & TIM_SR_UIF) {TIM2->SR &= ~(TIM_SR_UIF);gpio_toggle(PIN('B',2));}
//(*GPIOB).ODR^= GPIO_ODR_OD2;}//pb2 toggle
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

static inline void uart_init(USART_TypeDef *uart, unsigned long baud) {
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  uint32_t freq = 0;        // Bus frequency. UART1 is on APB2, rest on APB1
//  if (uart == USART1) freq = APB2_FREQUENCY, RCC->APB2ENR |= BIT(4);
//  if (uart == USART2) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(17);
//  if (uart == USART3) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(18); 
freq = (sys_clock_hz/4), RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
//  if (uart == USART1) tx = PIN('A', 9), rx = PIN('A', 10);
//  if (uart == USART2) tx = PIN('A', 2), rx = PIN('A', 3);
//purple RXD     , orange TXD debugger pins
tx = PIN('B', 10), rx = PIN('B', 11);

gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
uart->CR1=0x0;//reset this UART
uart->BRR = freq / baud;                // Set baud rate
//uart->BRR =(sys_clock_hz/4)/baud;// ??? 364 > 0x16c ???
uart->CR1 |= (BIT(13)/*enable*/|BIT(2)/*RX on*/|BIT(3)/*TX on*/);
}
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0) (void) 0;
}
static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->SR & BIT(5);  // If RXNE bit is set, data is ready
}
static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t) (uart->DR & 255);
}
int _write(int fd, char *ptr, int len) {
(void) fd, (void) ptr, (void) len;
if (fd == 1) uart_write_buf(/*UART_DEBUG*/USART3, ptr, (size_t) len);
return -1;}

//cp dmaPart9 type 2, double buffered dma, in single mode
#define _AMP(x) ( x / 8 )
const size_t SINE_SAMPLES = 32;
const uint16_t SINE_WAVE[] = {
_AMP(2048),_AMP(2447),_AMP(2831),_AMP(3185),
_AMP(3495),_AMP(3750),_AMP(3939),_AMP(4056),
_AMP(4095),_AMP(4056),_AMP(3939),_AMP(3750),
_AMP(3495),_AMP(3185),_AMP(2831),_AMP(2447),
_AMP(2048),_AMP(1649),_AMP(1265),_AMP(911),
_AMP(601), _AMP(346), _AMP(157), _AMP(40),
_AMP(0),   _AMP(40),  _AMP(157), _AMP(346),
_AMP(601), _AMP(911), _AMP(1265),_AMP(1649)
};
static void dma_sin(void){
  RCC->AHB1ENR  |= ( RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_DMA1EN );
  RCC->APB1ENR  |= ( RCC_APB1ENR_DACEN |
                     RCC_APB1ENR_TIM6EN );
  // Pin A4 output type: Analog.
  GPIOA->MODER    &= ~( 0x3 << ( 4 * 2 ) );
  GPIOA->MODER    |=  ( 0x3 << ( 4 * 2 ) );
  // DMA configuration (channel 7 / stream 5).
  // SxCR register:
  // - Memory-to-peripheral
  // - Circular mode enabled.
  // - Increment memory ptr, don't increment periph ptr.
  // - 16-bit data size for both source and destination.
  // - High priority (2/3).
  DMA1_Stream5->CR &= ~( DMA_SxCR_CHSEL |
                         DMA_SxCR_PL |
                         DMA_SxCR_MSIZE |
                         DMA_SxCR_PSIZE |
                         DMA_SxCR_PINC |
                         DMA_SxCR_EN );
  DMA1_Stream5->CR |=  ( ( 0x2 << DMA_SxCR_PL_Pos ) |
                         ( 0x1 << DMA_SxCR_MSIZE_Pos ) |
                         ( 0x1 << DMA_SxCR_PSIZE_Pos ) |
                         ( 0x7 << DMA_SxCR_CHSEL_Pos ) |
                         DMA_SxCR_MINC |
                         DMA_SxCR_CIRC |
                         ( 0x1 << DMA_SxCR_DIR_Pos ) );
  // Set DMA source and destination addresses.
  // Source: Address of the sine wave buffer in memory.
  DMA1_Stream5->M0AR  = ( uint32_t )&SINE_WAVE;
  // Dest.: DAC1 Ch1 '12-bit right-aligned data' register.
  DMA1_Stream5->PAR   = ( uint32_t )&( DAC1->DHR12R1 );
  // Set DMA data transfer length (# of sine wave samples).
  DMA1_Stream5->NDTR  = ( uint16_t )SINE_SAMPLES;
  // Enable DMA1 Stream 5.
  DMA1_Stream5->CR   |= ( DMA_SxCR_EN );
  // TIM6 configuration.
  // Set prescaler and autoreload for a 440Hz sine wave.
  TIM6->PSC  =  ( 0x0000 );
  TIM6->ARR  =  ( /*SystemCoreClock*/sys_clock_hz / ( 440 * SINE_SAMPLES ) );
  // Enable trigger output on timer update events.
  TIM6->CR2 &= ~( TIM_CR2_MMS );
  TIM6->CR2 |=  ( 0x2 << TIM_CR2_MMS_Pos );
  // Start the timer.
  TIM6->CR1 |=  ( TIM_CR1_CEN );
  // DAC configuration.
  // Set trigger sources to TIM6 TRGO.
  DAC1->CR  &= ~( DAC_CR_TSEL1 );
  // Enable DAC DMA requests.
  DAC1->CR  |=  ( DAC_CR_DMAEN1 );
  // Enable DAC Channels.
  DAC1->CR  |=  ( DAC_CR_EN1 );
  // Delay briefly to allow sampling to stabilize (?)
//  delay_cycles( 1000 );
  // Enable DAC channel trigger.
  DAC1->CR  |=  ( DAC_CR_TEN1 );
}//dma_sin end









#endif // __HAL_H__
