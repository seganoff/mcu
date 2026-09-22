#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

int
main(void) {

rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"
rcc_periph_clock_enable(RCC_GPIOC);
gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

//usb_start(); usbcdc.c/.h

//xTaskCreate(adventure,"game",300,NULL,configMAX_PRIORITIES-1,NULL);
//xTaskCreate(flasher,"flash",100,NULL,configMAX_PRIORITIES-1,NULL);

sem_flash = xSemaphoreCreateMutex();
//set_lamp(Drop);

vTaskStartScheduler();
for (;;);
return 0;
}

// End main.c
