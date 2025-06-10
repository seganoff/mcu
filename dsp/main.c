#include "hal.h"
//#include "mongoose.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {s_ticks++;}
//uint64_t mg_millis(void) { return s_ticks; }  // For Mongoose

int main(void) {
//uint16_t ld1 = PIN('B',0x00); // PB0  green || pa5 (sb119+120)
uint16_t ld2 = PIN('B',0x07); // PB7  blue
uint16_t ld3 = PIN('B',0x0e); // PB14 red
//clock_init(); > startup_stm767.s > SystemInit 
systick_init(SYS_FREQUENCY / 1000);    // Tick every 1 ms
gpio_set_mode(ld2, GPIO_MODE_OUTPUT);
gpio_output(ld3);
volatile uint32_t timer_blue = 0, period_blue = 500;  // declareTimers
volatile uint32_t timer_red = 0, period_red = 1000;
/*
uint16_t pins[] = {PIN('A', 1),  PIN('A', 2),  PIN('A', 7),
                     PIN('B', 13), PIN('C', 1),  PIN('C', 4),
                     PIN('C', 5),  PIN('G', 11), PIN('G', 13)};
for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
    gpio_init(pins[i], GPIO_MODE_AF, 0, GPIO_SPEED_INSANE, GPIO_PULL_NONE, 11);

NVIC_EnableIRQ(ETH_IRQn);                     // Setup Ethernet IRQ handler
//RCC->APB2ENR |= BIT(14);                      // Enable SYSCFG
SYSCFG->PMC|=SYSCFG_PMC_MII_RMII_SEL;         // Use RMII. Goes first!
//RCC->AHB1ENR |= BIT(25) | BIT(26) | BIT(27);  // Enable Ethernet clocks
RCC->AHB1ENR|=(RCC_AHB1ENR_ETHMACEN|RCC_AHB1ENR_ETHMACTXEN|RCC_AHB1ENR_ETHMACRXEN);
RCC->AHB1RSTR|=RCC_AHB1RSTR_ETHMACRST;          // ETHMAC force reset
RCC->AHB1RSTR&=~(RCC_AHB1RSTR_ETHMACRST); 

struct mg_mgr mgr;        // Initialise Mongoose event manager
mg_mgr_init(&mgr);        // and attach it to the MIP interface
mg_log_set(MG_LL_DEBUG);  // Set log level
struct mip_driver_stm32 driver_data = {.mdc_cr = 5};  // See driver_stm32.h
struct mip_if mif = {.mac = {2, 0, 1, 2, 3, 5},.use_dhcp = true,
      .driver = &mip_driver_stm32,.driver_data = &driver_data,
      //.ip=192168003015,.mask=255255255000,.gw=192168003001
};
mip_init(&mgr, &mif);
extern void device_dashboard_fn(struct mg_connection *, int, void *, void *);
mg_http_listen(&mgr, "http://0.0.0.0", device_dashboard_fn, &mgr);
MG_INFO(("Init done, starting main loop"));
*/
for (;;) {

if (timer_expired(&timer_blue, period_blue, s_ticks)) {
static bool on; gpio_write(ld2, on);on=!on;}

if (timer_expired(&timer_red, period_red, s_ticks)) {
static bool on; gpio_write(ld3, on);on=!on;}

//mg_mgr_poll(&mgr, 0);  // Handle networking

}//for loop
return 0;}
