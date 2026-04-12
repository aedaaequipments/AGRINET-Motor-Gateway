/**
 * @file main.c
 * @brief AGRINET 3-Phase Motor Controller - Master Gateway
 *
 * FreeRTOS-based firmware for STM32F103C8T6 (Blue Pill)
 * Phase 1+2: Motor control + power monitoring + LoRa + GSM/Firebase
 *
 * 5 Tasks (static allocation, no dynamic heap):
 *   Task_PowerMonitor  (prio 3) - ADC sampling, V/I/PF calculation
 *   Task_MotorControl  (prio 3) - Star-delta FSM, relay control
 *   Task_LoRaManager   (prio 2) - LoRa TX/RX, AGRINET protocol
 *   Task_CloudSync     (prio 1) - GSM/Firebase data push + command poll
 *   Task_Display       (prio 0) - OLED/UART rendering
 */

#include "config.h"
#include "flash_config.h"
#include "power_monitor.h"
#include "motor_control.h"
#include "watchdog.h"
#include "lora_radio.h"
#include "lora_manager.h"
#include "cloud_sync.h"
#include "gsm_driver.h"
#include "node_registry.h"
#include "automation.h"
#include "display.h"
#include "offline_queue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * SYSTICK HANDLER (safe for pre-scheduler + FreeRTOS)
 *
 * FreeRTOSConfig.h no longer maps xPortSysTickHandler→SysTick_Handler.
 * We define SysTick_Handler here so HAL_IncTick() runs from power-on
 * (needed for HAL_Delay), and xPortSysTickHandler only runs after the
 * scheduler starts. Without this, xTaskIncrementTick touches uninitialized
 * FreeRTOS structures → HardFault on clone chips (CKS32/GD32/APM32).
 * ═══════════════════════════════════════════════════════════════════════════ */
static volatile bool s_schedulerRunning = false;

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (s_schedulerRunning) {
        xPortSysTickHandler();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATIC TASK BUFFERS (no dynamic allocation)
 * ═══════════════════════════════════════════════════════════════════════════ */

static StaticTask_t xTaskPowerBuf;
static StackType_t  xTaskPowerStack[TASK_POWER_STACK];

static StaticTask_t xTaskMotorBuf;
static StackType_t  xTaskMotorStack[TASK_MOTOR_STACK];

static StaticTask_t xTaskLoraBuf;
static StackType_t  xTaskLoraStack[TASK_LORA_STACK];

static StaticTask_t xTaskCloudBuf;
static StackType_t  xTaskCloudStack[TASK_CLOUD_STACK];

static StaticTask_t xTaskDisplayBuf;
static StackType_t  xTaskDisplayStack[TASK_DISPLAY_STACK];

/* Idle task memory (required for static allocation) */
static StaticTask_t xIdleTaskBuf;
static StackType_t  xIdleTaskStack[configMINIMAL_STACK_SIZE];

/* Timer task memory (required when configUSE_TIMERS=1) */
static StaticTask_t xTimerTaskBuf;
static StackType_t  xTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

/* ═══════════════════════════════════════════════════════════════════════════
 * DEBUG UART
 * ═══════════════════════════════════════════════════════════════════════════ */

static UART_HandleTypeDef huart1;   // Debug UART

static void Debug_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_DEBUG_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PIN_DEBUG_TX_PORT, &gpio);

    gpio.Pin  = PIN_DEBUG_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PIN_DEBUG_RX_PORT, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
}

static void Debug_Print(const char* msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* UART provisioning buffer */
static char g_uartRxBuf[CMD_BUF_SIZE];
static uint8_t g_uartRxIdx = 0;

static void ProcessUartInput(void)
{
    uint8_t byte;
    while (HAL_UART_Receive(&huart1, &byte, 1, 0) == HAL_OK) {
        if (byte == '\n' || byte == '\r') {
            if (g_uartRxIdx > 0) {
                g_uartRxBuf[g_uartRxIdx] = '\0';
                char response[128];
                FlashConfig_ProcessCommand(g_uartRxBuf, response, sizeof(response));
                Debug_Print(response);
                g_uartRxIdx = 0;
            }
        } else if (g_uartRxIdx < CMD_BUF_SIZE - 1) {
            g_uartRxBuf[g_uartRxIdx++] = (char)byte;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LED
 * ═══════════════════════════════════════════════════════════════════════════ */

static void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_LED_PORT, &gpio);
}

static inline void LED_Toggle(void) {
    HAL_GPIO_TogglePin(PIN_LED_PORT, PIN_LED_PIN);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DWT CYCLE COUNTER (for precise ADC timing)
 * GD32/CKS32 clones may have TRCENA locked when no debug probe attached.
 * If CYCCNT doesn't increment, power_monitor falls back to a NOP loop.
 * ═══════════════════════════════════════════════════════════════════════════ */

bool g_dwtAvailable = false;  /* Shared — power_monitor.c reads this */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Verify CYCCNT actually increments (fails on some GD32 clones) */
    volatile uint32_t before = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    volatile uint32_t after = DWT->CYCCNT;

    g_dwtAvailable = (after != before);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SYSTEM CLOCK CONFIG (72MHz from 8MHz HSE)
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool s_hseOk = false;  /* Track if HSE started (for debug output) */

static void SystemClock_Config(void)
{
    HAL_StatusTypeDef status;

    /* Try HSE first (8MHz external crystal → PLL → 72MHz) */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;  // 8MHz * 9 = 72MHz
    status = HAL_RCC_OscConfig(&osc);

    if (status != HAL_OK) {
        /* HSE failed (common on GD32 clones with bad crystals).
         * Fall back to HSI (8MHz internal RC) → PLL → 64MHz.
         * UART/timers will still work, just slightly slower. */
        memset(&osc, 0, sizeof(osc));
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState       = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState   = RCC_PLL_ON;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
        osc.PLL.PLLMUL     = RCC_PLL_MUL16;  // (8MHz/2) * 16 = 64MHz
        HAL_RCC_OscConfig(&osc);
        s_hseOk = false;
    } else {
        s_hseOk = true;
    }

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;   // APB1 max 36MHz
    clk.APB2CLKDivider = RCC_HCLK_DIV1;   // APB2 = SYSCLK
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);

    /* ADC prescaler: keep under 14MHz */
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FREERTOS TASKS
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Task_PowerMonitor: Samples ADC and computes V/I/PF/kW
 * Priority 3 (highest) - safety critical
 */
static void Task_PowerMonitor(void* pvParam)
{
    (void)pvParam;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        PowerMonitor_Sample();
        Watchdog_Feed();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(POWER_SAMPLE_PERIOD_MS));
    }
}

/**
 * Task_MotorControl: Runs star-delta FSM and protection logic
 * Priority 3 - safety critical
 */
static void Task_MotorControl(void* pvParam)
{
    (void)pvParam;
    TickType_t xLastWake = xTaskGetTickCount();

    /* Stats save counter (save to flash every 5 minutes) */
    uint32_t statsSaveCounter = 0;

    for (;;) {
        MotorControl_Update();

        /* Periodically save stats to flash */
        statsSaveCounter++;
        if (statsSaveCounter >= (300000 / MOTOR_CHECK_PERIOD_MS)) {  // Every ~5 min
            statsSaveCounter = 0;
            MotorStats_t stats;
            MotorControl_GetStats(&stats);
            FlashConfig_UpdateStats(&stats);
            FlashConfig_Save();
        }

        Watchdog_Feed();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(MOTOR_CHECK_PERIOD_MS));
    }
}

/**
 * Task_LoRaManager: LoRa TX/RX, AGRINET protocol, and automation
 * Priority 2 - receives weather data, runs irrigation logic, sends valve cmds
 */
static void Task_LoRaManager(void* pvParam)
{
    (void)pvParam;

    /* Init LoRa from task context (uses HAL_Delay, can't block main) */
    bool loraOk = LoRaManager_Init();

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        if (!loraOk) {
            Watchdog_Feed();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        /* C1 FIX: Process LoRa IRQ from task context (ISR only sets flag) */
        LoRa_PollIRQ();

        LoRaManager_Update();

        /* Run automation when new weather data arrives */
        if (LoRaManager_HasNewWeatherData()) {
            WeatherPayload_t weather;
            uint64_t stationUid;
            if (LoRaManager_GetLastWeatherData(&weather, &stationUid)) {
                bool changed = Automation_Evaluate(&weather);
                if (changed) {
                    /* Valve state changed - send command to first valve node */
                    NodeEntry_t* valve = NodeRegistry_FindByType(NODE_TYPE_VALVE);
                    if (valve) {
                        uint8_t pos = Automation_ShouldValveOpen() ? 100 : 0;
                        LoRaManager_SendValveCommand(valve->uid, pos);
                    }
                    /* Auto motor control */
                    if (Automation_ShouldMotorRun() && !MotorControl_IsRunning()) {
                        MotorControl_Start();
                    } else if (!Automation_ShouldMotorRun() && MotorControl_IsRunning()) {
                        MotorControl_Stop(FAULT_REMOTE_STOP);
                    }
                }
            }
        }

        Watchdog_Feed();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));
    }
}

/**
 * Task_CloudSync: GSM/Firebase data push and command polling
 * Priority 1 - pushes motor/weather data, polls commands
 */
static void Task_CloudSync(void* pvParam)
{
    (void)pvParam;

    /* Init GSM from task context (uses HAL_Delay, can't block main) */
    if (GSM_Init()) {
        Debug_Print("GSM: SIM7670C OK\r\n");
    } else {
        Debug_Print("GSM: INIT FAILED (will retry)\r\n");
    }
    CloudSync_Init();

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        CloudSync_Update();
        Watchdog_Feed();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1000));  // 1s cycle
    }
}

/**
 * Task_Display: OLED rendering + debug UART output
 * Priority 0 (lowest) - cosmetic only
 */
static void Task_Display(void* pvParam)
{
    (void)pvParam;
    TickType_t xLastWake = xTaskGetTickCount();

    char line[128];
    uint8_t serialDiv = 0;

    for (;;) {
        /* OLED display (6-page auto-cycling) */
        Display_Update();

        /* Serial telemetry every 3 seconds (3 x DISPLAY_UPDATE_MS) */
        if (++serialDiv >= 3) {
            serialDiv = 0;

            PowerSnapshot_t snap;
            PowerMonitor_GetSnapshot(&snap);
            FlashConfig_t* cfg = FlashConfig_Get();

            /* Line 1: Calibrated readings */
            snprintf(line, sizeof(line),
                "[%s] %s V:%d,%d,%d A:%.1f,%.1f,%.1f PF:%.2f %.2fkW\r\n",
                cfg->deviceId,
                MotorControl_IsRunning() ? "RUN" : "STOP",
                (int)snap.r.voltage, (int)snap.y.voltage, (int)snap.b.voltage,
                snap.r.current, snap.y.current, snap.b.current,
                snap.avgPF, snap.totalPower
            );
            Debug_Print(line);

            /* Line 2: Raw ADC values for calibration */
            uint16_t rv[3], ri[3];
            PowerMonitor_GetRawADC(rv, ri);
            snprintf(line, sizeof(line),
                " RAW V:%u,%u,%u I:%u,%u,%u\r\n",
                rv[0], rv[1], rv[2], ri[0], ri[1], ri[2]);
            Debug_Print(line);

            FaultCode_t fault = MotorControl_GetFault();
            if (fault != FAULT_NONE && fault != FAULT_MANUAL_STOP) {
                snprintf(line, sizeof(line), "  FAULT: %s\r\n",
                         MotorControl_FaultString(fault));
                Debug_Print(line);
            }

            /* Show offline queue status */
            uint8_t qLen = OfflineQueue_Count();
            if (qLen > 0) {
                snprintf(line, sizeof(line), "  QUEUE: %u pkts pending\r\n", qLen);
                Debug_Print(line);
            }
        }

        LED_Toggle();
        Watchdog_Feed();
        ProcessUartInput();

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FREERTOS CALLBACKS (required for static allocation)
 * ═══════════════════════════════════════════════════════════════════════════ */

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                    StackType_t** ppxIdleTaskStackBuffer,
                                    uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskBuf;
    *ppxIdleTaskStackBuffer = xIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                     StackType_t** ppxTimerTaskStackBuffer,
                                     uint32_t* pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &xTimerTaskBuf;
    *ppxTimerTaskStackBuffer = xTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    /* Stack overflow detected -- halt with LED on */
    AllRelaysOff();  // Safety: turn off all relays
    HAL_GPIO_WritePin(PIN_LED_PORT, PIN_LED_PIN, GPIO_PIN_RESET);  // LED on (active low)

    char msg[64];
    snprintf(msg, sizeof(msg), "STACK OVERFLOW: %s\r\n", pcTaskName);
    Debug_Print(msg);

    for (;;) { __NOP(); }  // Halt -- watchdog will reset
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERRUPT HANDLERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Button EXTI interrupt (PB8=Start, PB9=Stop)
 */
void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(PIN_BTN_START_PIN)) {
        __HAL_GPIO_EXTI_CLEAR_IT(PIN_BTN_START_PIN);
        MotorControl_ButtonPress(true);
    }
    if (__HAL_GPIO_EXTI_GET_IT(PIN_BTN_STOP_PIN)) {
        __HAL_GPIO_EXTI_CLEAR_IT(PIN_BTN_STOP_PIN);
        MotorControl_ButtonPress(false);
    }
}

/**
 * LoRa DIO0 EXTI interrupt (PB12)
 */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(PIN_LORA_DIO0_PIN)) {
        __HAL_GPIO_EXTI_CLEAR_IT(PIN_LORA_DIO0_PIN);
        LoRa_HandleDIO0_IRQ();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    /* HAL init */
    HAL_Init();
    SystemClock_Config();
    DWT_Init();

    /* Peripheral init */
    LED_Init();

    /* UART + OLED early init */
    Debug_Init();
    Debug_Print("\r\n=== SASYAMITHRA Motor Gateway v" FW_VERSION_STR " ===\r\n");

    bool oledOk = Display_Init();
    if (oledOk) {
        Display_Splash();
        Debug_Print("OLED: 1.3\" 128x64 OK\r\n");
    } else {
        Debug_Print("OLED: NOT FOUND\r\n");
    }

    /* Banner continued */
    if (s_hseOk) {
        Debug_Print("CLK: HSE 72MHz OK\r\n");
    } else {
        Debug_Print("CLK: HSE FAILED - using HSI 64MHz (GD32 clone?)\r\n");
    }
    Debug_Print(g_dwtAvailable ? "DWT: CYCCNT OK\r\n" : "DWT: CYCCNT LOCKED (NOP fallback)\r\n");

    /* Load config from flash */
    bool configLoaded = FlashConfig_Init();
    FlashConfig_t* cfg = FlashConfig_Get();

    if (configLoaded) {
        char msg[80];
        snprintf(msg, sizeof(msg), "Config loaded: %s HP:%.1f\r\n",
                 cfg->deviceId, cfg->calibration.ratedHP);
        Debug_Print(msg);
    } else {
        Debug_Print("No config found - using defaults\r\n");
        Debug_Print("Use SET_ID/SET_UID/SET_TOKEN/SET_URL to provision\r\n");
    }

    if (Watchdog_WasReset()) {
        Debug_Print("WARNING: Watchdog reset detected!\r\n");
    }

    /* Init fast subsystems only — LoRa/GSM init moved into their tasks
     * to avoid blocking the scheduler (they use HAL_Delay which can
     * hang for seconds if hardware is absent). */
    PowerMonitor_Init();
    MotorControl_Init();
    Automation_Init();
    OfflineQueue_Init();

    if (oledOk) {
        Debug_Print("OLED: 1.3\" 128x64 OK\r\n");
    } else {
        Debug_Print("OLED: NOT FOUND\r\n");
    }

    Debug_Print("Starting scheduler...\r\n");

    /* Create FreeRTOS tasks (static allocation) */
    xTaskCreateStatic(Task_PowerMonitor, "Power",
                      TASK_POWER_STACK, NULL, TASK_POWER_PRIO,
                      xTaskPowerStack, &xTaskPowerBuf);

    xTaskCreateStatic(Task_MotorControl, "Motor",
                      TASK_MOTOR_STACK, NULL, TASK_MOTOR_PRIO,
                      xTaskMotorStack, &xTaskMotorBuf);

    xTaskCreateStatic(Task_LoRaManager, "LoRa",
                      TASK_LORA_STACK, NULL, TASK_LORA_PRIO,
                      xTaskLoraStack, &xTaskLoraBuf);

    xTaskCreateStatic(Task_CloudSync, "Cloud",
                      TASK_CLOUD_STACK, NULL, TASK_CLOUD_PRIO,
                      xTaskCloudStack, &xTaskCloudBuf);

    xTaskCreateStatic(Task_Display, "Display",
                      TASK_DISPLAY_STACK, NULL, TASK_DISPLAY_PRIO,
                      xTaskDisplayStack, &xTaskDisplayBuf);

    Debug_Print("Tasks created. Starting scheduler...\r\n");

    /* Init watchdog LAST (after all setup complete) */
    Watchdog_Init();

    /* Start FreeRTOS scheduler -- never returns */
    s_schedulerRunning = true;
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) { __NOP(); }
}
