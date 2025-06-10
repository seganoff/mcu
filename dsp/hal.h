#pragma once

//um1974 6.5
//ld1 pb0 || pa5 ; ld2 pb7 ; ld3 pb14

//#include <stdbool.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <string.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "stm32f767xx.h"//"stm32f429xx.h"

#define BIT(x) (1UL << (x))
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
//uint32_t SystemCoreClock;
enum { APB1_PRE = 4 /* AHB clock / 4 */, APB2_PRE = 2 /* AHB clock / 2 */ };
enum {OSC_IN=8,PLL_M=4,PLL_N=216,PLL_P=2,PLL_Q=9,PLL_R=2};
//enum { PLL_HSI = 16, PLL_M = 8, PLL_N = 216, PLL_P = 2 };  // Run at 216 Mhz
//#define OSC_IN 16
#define FLASH_LATENCY 7
#define SYS_FREQUENCY ((OSC_IN * PLL_N / PLL_M / PLL_P) * 1000000)
//#define APB2_FREQUENCY (SYS_FREQUENCY / (BIT(APB2_PRE - 3)))
//#define APB1_FREQUENCY (SYS_FREQUENCY / (BIT(APB1_PRE - 3)))

static inline void spin(volatile uint32_t count) {while (count--) /*(void) 0;*/asm("nop");}

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };
#define GPIO(N) ((GPIO_TypeDef *) (0x40020000 + 0x400 * (N)))

static GPIO_TypeDef *gpio_bank(uint16_t pin) { return GPIO(PINBANK(pin)); }
static inline void gpio_toggle(uint16_t pin) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint32_t mask = BIT(PINNO(pin));
  gpio->BSRR = mask << (gpio->ODR & mask ? 16 : 0);
}
static inline int gpio_read(uint16_t pin) {
  return gpio_bank(pin)->IDR & BIT(PINNO(pin)) ? 1 : 0;
}
static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  gpio->BSRR = BIT(PINNO(pin)) << (val ? 0 : 16);
}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint8_t n = (uint8_t) (PINNO(pin));
  RCC->AHB1ENR |= BIT(PINBANK(pin));  // Enable GPIO clock
  SETBITS(gpio->OTYPER, 1UL << n, ((uint32_t) type) << n);
  SETBITS(gpio->OSPEEDR, 3UL << (n * 2), ((uint32_t) speed) << (n * 2));
  SETBITS(gpio->PUPDR, 3UL << (n * 2), ((uint32_t) pull) << (n * 2));
  SETBITS(gpio->AFR[n >> 3], 15UL << ((n & 7) * 4),
          ((uint32_t) af) << ((n & 7) * 4));
  SETBITS(gpio->MODER, 3UL << (n * 2), ((uint32_t) mode) << (n * 2));
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  RCC->AHB1ENR |= BIT(PINBANK(pin));       // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}
/* for step6 ?*/
static inline void irq_exti_attach(uint16_t pin) {
  uint8_t bank = (uint8_t) (PINBANK(pin)), n = (uint8_t) (PINNO(pin));
  SYSCFG->EXTICR[n / 4] &= ~(15UL << ((n % 4) * 4));
  SYSCFG->EXTICR[n / 4] |= (uint32_t) (bank << ((n % 4) * 4));
  EXTI->IMR |= BIT(n);
  EXTI->RTSR |= BIT(n);
  EXTI->FTSR |= BIT(n);
  int irqvec = n < 5 ? 6 + n : n < 10 ? 23 : 40;  // IRQ vector index, 10.1.2
  NVIC_SetPriority(irqvec, 3);
  NVIC_EnableIRQ(irqvec);}
//*/

/*
#ifndef UART_DEBUG
#define UART_DEBUG USART3
#endif

static inline bool uart_init(USART_TypeDef *uart, unsigned long baud) {
  // https://www.st.com/resource/en/datasheet/stm32f429zi.pdf
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  uint32_t freq = 0;        // Bus frequency. UART1 is on APB2, rest on APB1

  if (uart == USART1) {
    freq = APB2_FREQUENCY, RCC->APB2ENR |= BIT(4);
    tx = PIN('A', 9), rx = PIN('A', 10);
  } else if (uart == USART2) {
    freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(17);
    tx = PIN('A', 2), rx = PIN('A', 3);
  } else if (uart == USART3) {
    freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(18);
    tx = PIN('D', 8), rx = PIN('D', 9);
  } else {
    return false;
  }

  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  uart->CR1 = 0;                           // Disable this UART
  uart->BRR = freq / baud;                 // Set baud rate
  uart->CR1 |= BIT(13) | BIT(2) | BIT(3);  // Set UE, RE, TE
  return true;
}
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0) spin(1);
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
*/

static inline void rng_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
  RNG->CR |= RNG_CR_RNGEN;
}
static inline uint32_t rng_read(void) {
  while ((RNG->SR & RNG_SR_DRDY) == 0) (void) 0;
  return RNG->DR;
}
 //not in step7 hal.h

//#define UUID ((uint8_t *) UID_BASE)  // Unique 96-bit chip ID. TRM 39.1
/*
// Helper macro for MAC generation
#define GENERATE_LOCALLY_ADMINISTERED_MAC()                        \
  {                                                                \
    2, UUID[0] ^ UUID[1], UUID[2] ^ UUID[3], UUID[4] ^ UUID[5],    \
        UUID[6] ^ UUID[7] ^ UUID[8], UUID[9] ^ UUID[10] ^ UUID[11] \
  }
*/

static inline void systick_init(uint32_t ticks) {
if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
SysTick->LOAD = ticks - 1;
SysTick->VAL = 0;
SysTick->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}

static inline bool timer_expired(volatile uint32_t *t, uint32_t prd,
                                 uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
/*
static inline void clock_init(void){
//core_cm7.h:2229 static_inline voids
SCB_EnableICache(); SCB_EnableDCache();
// Set FLASH latency LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);
SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // Enable FPU
FLASH->ACR |= FLASH_LATENCY | BIT(8) | BIT(9);      // Flash latency, prefetch
RCC->CR |= RCC_CR_CSSON;
RCC->CR |= RCC_CR_HSEBYP;
RCC->CR |= ((uint32_t)RCC_CR_HSEON);
RCC->APB1ENR |= RCC_APB1ENR_PWREN;
PWR->CR1 &= (uint32_t)~(PWR_CR1_VOS);
RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
RCC->PLLCFGR = 
  PLL_M |
 (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
 (((PLL_P >> 1) -1) << RCC_PLLCFGR_PLLP_Pos) |
 (RCC_PLLCFGR_PLLSRC_HSE)
;
RCC->CR |= RCC_CR_PLLON;
while((RCC->CR & RCC_CR_PLLRDY) == 0){;}
RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
RCC->CFGR |= RCC_CFGR_SW_PLL;
// Wait till the main PLL is used as system clock source
while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){;}
//RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG, not in 7
//step 7 enable syscfg done in hal.h:systick_init & in main.c straight after enableirq(ETH_IRQn)
//but come_cm7.h:2564, there is no call to enable syscfg+addiotional nvic_setPriority
//SysTick_Config(SYS_FREQUENCY / 1000);  // Sys tick every 1ms
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//SystemCoreClockUpdate(); undef reference
}*/
