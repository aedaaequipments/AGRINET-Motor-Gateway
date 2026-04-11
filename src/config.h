/**
 * @file config.h
 * @brief AGRINET 3-Phase Motor Controller - Pin definitions, constants, and configuration
 *
 * Hardware: STM32F103C8T6 (Blue Pill)
 * All pin assignments, thresholds, timing constants, and struct definitions.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * FIRMWARE VERSION
 * ═══════════════════════════════════════════════════════════════════════════ */
#define FW_VERSION_STR      "1.0.0"

/* ═══════════════════════════════════════════════════════════════════════════
 * PIN ASSIGNMENTS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* --- ADC Pins (Analog In) --- */
#define PIN_V_CT_R_PORT     GPIOA
#define PIN_V_CT_R_PIN      GPIO_PIN_0      // PA0 - Voltage CT Phase R
#define PIN_V_CT_R_CH       ADC_CHANNEL_0

#define PIN_V_CT_Y_PORT     GPIOA
#define PIN_V_CT_Y_PIN      GPIO_PIN_1      // PA1 - Voltage CT Phase Y
#define PIN_V_CT_Y_CH       ADC_CHANNEL_1

#define PIN_V_CT_B_PORT     GPIOA
#define PIN_V_CT_B_PIN      GPIO_PIN_2      // PA2 - Voltage CT Phase B
#define PIN_V_CT_B_CH       ADC_CHANNEL_2

#define PIN_I_CT_R_PORT     GPIOA
#define PIN_I_CT_R_PIN      GPIO_PIN_3      // PA3 - Current CT Phase R
#define PIN_I_CT_R_CH       ADC_CHANNEL_3

#define PIN_I_CT_Y_PORT     GPIOB
#define PIN_I_CT_Y_PIN      GPIO_PIN_0      // PB0 - Current CT Phase Y
#define PIN_I_CT_Y_CH       ADC_CHANNEL_8

#define PIN_I_CT_B_PORT     GPIOB
#define PIN_I_CT_B_PIN      GPIO_PIN_1      // PB1 - Current CT Phase B
#define PIN_I_CT_B_CH       ADC_CHANNEL_9

/* --- Relay Outputs --- */
#define PIN_RELAY_MAIN_PORT GPIOB
#define PIN_RELAY_MAIN_PIN  GPIO_PIN_15     // PB15 - Main contactor relay

#define PIN_RELAY_STAR_PORT GPIOB
#define PIN_RELAY_STAR_PIN  GPIO_PIN_14     // PB14 - Star contactor relay

#define PIN_RELAY_DELTA_PORT GPIOA
#define PIN_RELAY_DELTA_PIN GPIO_PIN_8      // PA8 - Delta contactor relay

/* --- Button Inputs (EXTI) --- */
#define PIN_BTN_START_PORT  GPIOB
#define PIN_BTN_START_PIN   GPIO_PIN_8      // PB8 - Start button
#define PIN_BTN_START_IRQ   EXTI9_5_IRQn

#define PIN_BTN_STOP_PORT   GPIOB
#define PIN_BTN_STOP_PIN    GPIO_PIN_9      // PB9 - Stop button
#define PIN_BTN_STOP_IRQ    EXTI9_5_IRQn

/* --- LoRa SX1276 (SPI1) --- */
#define PIN_LORA_NSS_PORT   GPIOA
#define PIN_LORA_NSS_PIN    GPIO_PIN_4      // PA4 - LoRa chip select

#define PIN_LORA_SCK_PORT   GPIOA
#define PIN_LORA_SCK_PIN    GPIO_PIN_5      // PA5 - SPI1 SCK

#define PIN_LORA_MISO_PORT  GPIOA
#define PIN_LORA_MISO_PIN   GPIO_PIN_6      // PA6 - SPI1 MISO

#define PIN_LORA_MOSI_PORT  GPIOA
#define PIN_LORA_MOSI_PIN   GPIO_PIN_7      // PA7 - SPI1 MOSI

#define PIN_LORA_DIO0_PORT  GPIOB
#define PIN_LORA_DIO0_PIN   GPIO_PIN_12     // PB12 - LoRa DIO0 (RX done IRQ)
#define PIN_LORA_DIO0_IRQ   EXTI15_10_IRQn

#define PIN_LORA_RST_PORT   GPIOB
#define PIN_LORA_RST_PIN    GPIO_PIN_13     // PB13 - LoRa reset

/* --- GSM SIM7670C (USART3) --- */
#define PIN_GSM_TX_PORT     GPIOB
#define PIN_GSM_TX_PIN      GPIO_PIN_10     // PB10 - USART3 TX

#define PIN_GSM_RX_PORT     GPIOB
#define PIN_GSM_RX_PIN      GPIO_PIN_11     // PB11 - USART3 RX

/* --- OLED SSD1306 (I2C1) --- */
#define PIN_OLED_SDA_PORT   GPIOB
#define PIN_OLED_SDA_PIN    GPIO_PIN_6      // PB6 - I2C1 SDA

#define PIN_OLED_SCL_PORT   GPIOB
#define PIN_OLED_SCL_PIN    GPIO_PIN_7      // PB7 - I2C1 SCL

#define OLED_I2C_ADDR       0x3C            // SSD1306 7-bit address

/* --- Debug UART (USART1) --- */
#define PIN_DEBUG_TX_PORT   GPIOA
#define PIN_DEBUG_TX_PIN    GPIO_PIN_9      // PA9 - USART1 TX

#define PIN_DEBUG_RX_PORT   GPIOA
#define PIN_DEBUG_RX_PIN    GPIO_PIN_10     // PA10 - USART1 RX

/* --- Status LED --- */
#define PIN_LED_PORT        GPIOC
#define PIN_LED_PIN         GPIO_PIN_13     // PC13 - Onboard LED (active low)

/* ═══════════════════════════════════════════════════════════════════════════
 * ADC CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ADC_SAMPLES_PER_CYCLE   150         // Samples per 50Hz cycle (3 cycles)
#define ADC_VREF                3.3f        // ADC reference voltage
#define ADC_RESOLUTION          4096        // 12-bit ADC
#define MAINS_FREQUENCY         50          // 50Hz India

/* --- Calibration constants (adjust per CT ratio) --- */
#define V_CT_RATIO              230.0f      // Voltage CT turns ratio (adjust for actual CT)
#define I_CT_RATIO              30.0f       // Current CT turns ratio (e.g., SCT-013-030)
#define V_CT_BURDEN             62.0f       // Burden resistor ohms
#define I_CT_BURDEN             33.0f       // Burden resistor ohms

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR PROTECTION DEFAULTS (auto-calibration will override these)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define DEFAULT_OVER_VOLTAGE    260.0f      // Overvoltage trip (Volts)
#define DEFAULT_UNDER_VOLTAGE   180.0f      // Undervoltage trip (Volts)
#define DEFAULT_OVERLOAD_AMPS   15.0f       // Overload trip (Amps)
#define DEFAULT_DRY_RUN_SEC     30          // Dry run timeout (seconds)
#define DEFAULT_RESTART_DELAY   60          // Restart delay after trip (seconds)
#define DEFAULT_STAR_DELTA_SEC  10          // Star-to-delta transition (seconds)
#define DEAD_TIME_MS            50          // Dead time between star-off and delta-on (ms)

/* --- Auto-calibration multipliers --- */
#define OVERLOAD_MULTIPLIER     1.3f        // Trip at 130% of rated current
#define DRY_RUN_MULTIPLIER      0.3f        // No-load < 30% of rated current
#define SAFE_MODE_MULTIPLIER    1.1f        // Safe mode: trip at 110%

/* --- Star-delta auto-timing by HP --- */
#define SD_DELAY_PER_HP         1.5f        // Extra seconds per HP
#define SD_DELAY_BASE           3.0f        // Minimum star time (seconds)
#define SD_DELAY_MAX            20.0f       // Maximum star time (seconds)

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR HEALTH THRESHOLDS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define HEALTH_PF_WARNING       0.75f       // PF below this = warning
#define HEALTH_PF_CRITICAL      0.60f       // PF below this = critical
#define HEALTH_IMBALANCE_WARN   10.0f       // Current imbalance % = warning
#define HEALTH_IMBALANCE_CRIT   20.0f       // Current imbalance % = critical

/* ═══════════════════════════════════════════════════════════════════════════
 * TIMING CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define POWER_SAMPLE_PERIOD_MS  200         // Power monitor sampling period
#define MOTOR_CHECK_PERIOD_MS   500         // Motor FSM check period
#define DISPLAY_UPDATE_MS       1000        // OLED refresh rate
#define CLOUD_PUSH_PERIOD_MS    5000        // MQTT telemetry push interval
#define CLOUD_POLL_PERIOD_MS    3000        // (unused — MQTT subscribe replaces polling)
#define LORA_POLL_PERIOD_MS     15000       // LoRa weather station poll
#define HEARTBEAT_PERIOD_MS     30000       // MQTT heartbeat interval
#define BUTTON_DEBOUNCE_MS      200         // Button debounce time

/* ═══════════════════════════════════════════════════════════════════════════
 * FREERTOS TASK CONFIG
 * ═══════════════════════════════════════════════════════════════════════════ */
#define TASK_POWER_STACK        256         // words (1024 bytes)
#define TASK_MOTOR_STACK        384         // words (1536 bytes)
#define TASK_LORA_STACK         384         // words (1536 bytes)
#define TASK_CLOUD_STACK        512         // words (2048 bytes)
#define TASK_DISPLAY_STACK      320         // words (1280 bytes) — C6 FIX: snprintf+I2C needs more

#define TASK_POWER_PRIO         3           // Highest: safety-critical
#define TASK_MOTOR_PRIO         3           // Same as power (co-operative)
#define TASK_LORA_PRIO          2           // Medium: field network
#define TASK_CLOUD_PRIO         1           // Lower: can wait
#define TASK_DISPLAY_PRIO       0           // Lowest: cosmetic

/* ═══════════════════════════════════════════════════════════════════════════
 * BUFFER SIZES
 * ═══════════════════════════════════════════════════════════════════════════ */
#define GSM_RX_BUF_SIZE         384     // Reduced from 512 — MQTT AT responses shorter than HTTP
#define GSM_TX_BUF_SIZE         256     // Reduced from 384 — longest MQTT AT cmd ~200 chars
#define JSON_BUF_SIZE           384     // Reduced from 512 — MQTT payloads < 300 chars
#define LORA_PKT_BUF_SIZE      256
#define CMD_BUF_SIZE            128

/* ═══════════════════════════════════════════════════════════════════════════
 * DEVICE ID
 * ═══════════════════════════════════════════════════════════════════════════ */
#define DEVICE_ID_MAX_LEN       16          // "MOT-001-A1\0" = 11 chars
#define MQTT_BROKER_IP_MAX_LEN  32          // "192.168.1.100\0"
#define MQTT_USERNAME_MAX_LEN   32
#define MQTT_PASSWORD_MAX_LEN   32
#define FARM_ID_MAX_LEN         16          // "farm01\0"

/* ═══════════════════════════════════════════════════════════════════════════
 * FLASH CONFIG
 * ═══════════════════════════════════════════════════════════════════════════ */
#define FLASH_CONFIG_PAGE       63          // Last page of 64KB flash
#define FLASH_CONFIG_ADDR       (0x08000000 + (FLASH_CONFIG_PAGE * 1024))
#define FLASH_CONFIG_MAGIC      0xAC3E      // Magic number to validate config

/* ═══════════════════════════════════════════════════════════════════════════
 * WATCHDOG
 * ═══════════════════════════════════════════════════════════════════════════ */
#define IWDG_TIMEOUT_MS         4000        // 4 second watchdog timeout

/* ═══════════════════════════════════════════════════════════════════════════
 * DATA STRUCTURES
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Operating mode of the motor controller
 */
typedef enum {
    MODE_MANUAL = 0,        // User controls start/stop only
    MODE_AUTO   = 1,        // Responds to soil moisture thresholds
} MotorMode_t;

/**
 * @brief Motor controller state machine states
 */
typedef enum {
    MOTOR_IDLE = 0,         // Motor stopped, ready
    MOTOR_STAR,             // Star contactor energized
    MOTOR_STAR_TO_DELTA,    // Dead-time transition
    MOTOR_DELTA,            // Delta contactor energized (running)
    MOTOR_STOPPING,         // Shutting down
    MOTOR_FAULT,            // Protection tripped
    MOTOR_RESTART_WAIT,     // Waiting before auto-restart
} MotorState_t;

/**
 * @brief Fault codes
 */
typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERLOAD,
    FAULT_DRY_RUN,
    FAULT_PHASE_LOSS,
    FAULT_CURRENT_IMBALANCE,
    FAULT_MANUAL_STOP,
    FAULT_REMOTE_STOP,
} FaultCode_t;

/**
 * @brief Motor health status
 */
typedef enum {
    HEALTH_GOOD = 0,
    HEALTH_WARNING,
    HEALTH_CRITICAL,
} HealthStatus_t;

/**
 * @brief Protection thresholds (configurable from app)
 */
typedef struct {
    float overVoltage;      // Overvoltage trip (V)
    float underVoltage;     // Undervoltage trip (V)
    float overloadAmps;     // Overload trip (A)
    uint16_t dryRunSec;     // Dry run timeout (s)
    uint16_t restartDelay;  // Restart delay (s)
} ProtectionConfig_t;

/**
 * @brief Auto-calibration data
 */
typedef struct {
    float ratedHP;          // Detected motor HP
    float ratedVoltage;     // Measured line voltage
    float ratedCurrent;     // Measured delta current
    uint8_t phases;         // Always 3
    uint32_t calibratedAt;  // Unix timestamp
    bool isCalibrated;      // Has been calibrated
} CalibrationData_t;

/**
 * @brief Per-phase power readings
 */
typedef struct {
    float voltage;          // RMS voltage (V)
    float current;          // RMS current (A)
    float power;            // Real power (W)
    float powerFactor;      // Power factor (0-1)
    float apparentPower;    // VA
} PhaseReading_t;

/**
 * @brief Complete 3-phase power snapshot
 */
typedef struct {
    PhaseReading_t r;       // Phase R (Red)
    PhaseReading_t y;       // Phase Y (Yellow)
    PhaseReading_t b;       // Phase B (Blue)
    float totalPower;       // Total kW
    float avgPF;            // Average power factor
    float avgVoltage;       // Average voltage
    float avgCurrent;       // Average current
    float currentImbalance; // Current imbalance %
    uint32_t timestamp;     // Reading timestamp
} PowerSnapshot_t;

/**
 * @brief Motor runtime statistics (persisted to flash)
 */
typedef struct {
    float totalKwh;         // Cumulative energy (kWh)
    float totalHours;       // Cumulative runtime (hours)
    uint32_t startCount;    // Total number of starts
    uint32_t lastStartTs;   // Unix timestamp of last start
    uint32_t lastStopTs;    // Unix timestamp of last stop
} MotorStats_t;

/**
 * @brief Persistent flash configuration
 */
typedef struct {
    uint16_t magic;                             // FLASH_CONFIG_MAGIC
    uint16_t version;                           // Config struct version (1)

    /* Device identity */
    char deviceId[DEVICE_ID_MAX_LEN];           // "MOT-001-A1"

    /* MQTT broker (replaces Firebase HTTP — C1/C2 fix) */
    char mqttBrokerIp[MQTT_BROKER_IP_MAX_LEN];  // Pi4 Mosquitto IP
    uint16_t mqttBrokerPort;                     // 1883 default
    char mqttUsername[MQTT_USERNAME_MAX_LEN];     // Mosquitto auth
    char mqttPassword[MQTT_PASSWORD_MAX_LEN];     // Mosquitto auth
    char farmId[FARM_ID_MAX_LEN];                // Topic prefix: data/{farmId}/...

    /* Motor config */
    ProtectionConfig_t protection;
    CalibrationData_t calibration;
    uint16_t starDeltaDelay;                    // Star-delta time (seconds x 10 for 0.1s resolution)
    MotorMode_t mode;

    /* Mode flags */
    bool safeMode;
    bool forceRun;

    /* Runtime stats */
    MotorStats_t stats;

    /* Sync timestamp (for bidirectional config sync) */
    uint32_t configTimestamp;                    // Unix timestamp of last config change

    /* CRC for validation */
    uint16_t crc;
} FlashConfig_t;

/**
 * @brief Motor command from cloud or local button
 */
typedef struct {
    enum { CMD_START, CMD_STOP, CMD_CONFIG_UPDATE } type;
    uint32_t timestamp;
} MotorCommand_t;

#endif /* CONFIG_H */
