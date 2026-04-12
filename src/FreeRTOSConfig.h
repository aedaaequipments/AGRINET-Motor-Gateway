/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS configuration for STM32F103C8 Motor Gateway
 *
 * Most values are passed via platformio.ini build_flags (-D).
 * This file provides any remaining defaults and hook declarations.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* These are defined via build_flags in platformio.ini — only
 * provide them here as fallbacks if not already set. */

#ifndef configUSE_PREEMPTION
#define configUSE_PREEMPTION            1
#endif

#ifndef configCPU_CLOCK_HZ
#define configCPU_CLOCK_HZ              72000000UL
#endif

#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ              1000
#endif

#ifndef configMAX_PRIORITIES
#define configMAX_PRIORITIES            5
#endif

#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE        128
#endif

#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE           0
#endif

#ifndef configSUPPORT_STATIC_ALLOCATION
#define configSUPPORT_STATIC_ALLOCATION 1
#endif

#ifndef configSUPPORT_DYNAMIC_ALLOCATION
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#endif

#ifndef configMAX_TASK_NAME_LEN
#define configMAX_TASK_NAME_LEN         12
#endif

#ifndef configUSE_MUTEXES
#define configUSE_MUTEXES               1
#endif

#ifndef configUSE_COUNTING_SEMAPHORES
#define configUSE_COUNTING_SEMAPHORES   1
#endif

#ifndef configQUEUE_REGISTRY_SIZE
#define configQUEUE_REGISTRY_SIZE       8
#endif

#ifndef configUSE_TIMERS
#define configUSE_TIMERS                1
#endif

#ifndef configTIMER_TASK_PRIORITY
#define configTIMER_TASK_PRIORITY       2
#endif

#ifndef configTIMER_QUEUE_LENGTH
#define configTIMER_QUEUE_LENGTH        10
#endif

#ifndef configTIMER_TASK_STACK_DEPTH
#define configTIMER_TASK_STACK_DEPTH    256
#endif

#ifndef configCHECK_FOR_STACK_OVERFLOW
#define configCHECK_FOR_STACK_OVERFLOW  2
#endif

#ifndef configUSE_IDLE_HOOK
#define configUSE_IDLE_HOOK             0
#endif

#ifndef configUSE_TICK_HOOK
#define configUSE_TICK_HOOK             0
#endif

/* Cortex-M3 interrupt priority definitions */
#define configPRIO_BITS                 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Map FreeRTOS port interrupt handlers to STM32 HAL names.
 * NOTE: SysTick_Handler is NOT mapped here — it's defined manually in main.c
 * so HAL_IncTick() runs before the scheduler starts (prevents HardFault
 * from xTaskIncrementTick accessing uninitialized data). */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
/* #define xPortSysTickHandler SysTick_Handler  -- see main.c */

/* Optional: include for assert */
#define configASSERT(x) if (!(x)) { taskDISABLE_INTERRUPTS(); for(;;); }

/* CMSIS-RTOS v2 adaptation */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* Include optional API functions used by the firmware */
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_xTaskGetSchedulerState          1

#endif /* FREERTOS_CONFIG_H */
