
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

////<C-q u hex> 00bf -v unix -q pinche wsl

/// ¿comprendable 4 everybody?¿
/* Library includes. */
/* #include "stm32f10x_lib.h" */

//// from 0. + better explained
/* Here is a good place to include header files that are required across
   your application.
#include "something.h"
*/

/*-----------------------------------------------------------
* 0. https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/02-Customization
* 1. <rtos kernel git sysroot >/ examples/template_configuration/FreeRTOSConfig.h every define doxyCommented
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
///hsi16m lsi32k hse25m lse32.768k blackpill
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )	
///// tha 8 is not aMagicNumber, rm0368rev5 inkPage94/847 Figure 12. Clock tree + foll. page /////
///The RCC feeds the external clock of the Cortex System Timer (SysTick) with the AHB clock
///(HCLK) divided by 8
#define configSYSTICK_CLOCK_HZ ( configCPU_CLOCK_HZ / 8 )  /* fix for vTaskDelay() */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES			( 5 )
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 17 * 1024 ) )
#define configMAX_TASK_NAME_LEN			( 16 )
#define configUSE_TRACE_FACILITY		0
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_MUTEXES			1

#define configUSE_STREAM_BUFFERS                1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         1


/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 			0
#define configMAX_CO_ROUTINE_PRIORITIES 	2

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete			1
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend			0
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 	255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

#endif /* FREERTOS_CONFIG_H */

