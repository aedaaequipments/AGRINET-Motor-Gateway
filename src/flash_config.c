/**
 * @file flash_config.c
 * @brief Persistent configuration in STM32 internal flash (last 1KB page)
 *
 * C1/C2 fix: Firebase credentials replaced with MQTT broker config.
 * UART provisioning updated: SET_BROKER, SET_MQTT_USER, SET_MQTT_PASS, SET_FARM
 */

#include "flash_config.h"
#include "credentials.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static FlashConfig_t g_config;  // RAM copy of flash config
static bool g_dirty = false;    // Config modified since last save

/* ═══════════════════════════════════════════════════════════════════════════
 * CRC-16 (CCITT)
 * ═══════════════════════════════════════════════════════════════════════════ */

uint16_t FlashConfig_CRC16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FACTORY DEFAULTS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ApplyDefaults(FlashConfig_t* cfg)
{
    memset(cfg, 0, sizeof(FlashConfig_t));

    cfg->magic   = FLASH_CONFIG_MAGIC;
    cfg->version = 2;  /* Bumped from 1 — MQTT replaces Firebase */

    /* Device identity */
    strncpy(cfg->deviceId, DEFAULT_DEVICE_ID, DEVICE_ID_MAX_LEN - 1);

    /* MQTT broker defaults from credentials.h */
    strncpy(cfg->mqttBrokerIp, MQTT_DEFAULT_BROKER_IP, MQTT_BROKER_IP_MAX_LEN - 1);
    cfg->mqttBrokerPort = MQTT_DEFAULT_BROKER_PORT;
    strncpy(cfg->mqttUsername, MQTT_DEFAULT_USERNAME, MQTT_USERNAME_MAX_LEN - 1);
    strncpy(cfg->mqttPassword, MQTT_DEFAULT_PASSWORD, MQTT_PASSWORD_MAX_LEN - 1);
    strncpy(cfg->farmId, MQTT_DEFAULT_FARM_ID, FARM_ID_MAX_LEN - 1);

    /* Protection defaults */
    cfg->protection.overVoltage  = DEFAULT_OVER_VOLTAGE;
    cfg->protection.underVoltage = DEFAULT_UNDER_VOLTAGE;
    cfg->protection.overloadAmps = DEFAULT_OVERLOAD_AMPS;
    cfg->protection.dryRunSec    = DEFAULT_DRY_RUN_SEC;
    cfg->protection.restartDelay = DEFAULT_RESTART_DELAY;

    /* Calibration (not yet calibrated) */
    cfg->calibration.isCalibrated = false;
    cfg->calibration.ratedHP      = 0.0f;
    cfg->calibration.ratedVoltage = 0.0f;
    cfg->calibration.ratedCurrent = 0.0f;
    cfg->calibration.phases       = 3;
    cfg->calibration.calibratedAt = 0;

    /* Motor config */
    cfg->starDeltaDelay = DEFAULT_STAR_DELTA_SEC * 10;
    cfg->mode           = MODE_MANUAL;
    cfg->safeMode       = false;
    cfg->forceRun       = false;

    /* Stats */
    cfg->stats.totalKwh    = 0.0f;
    cfg->stats.totalHours  = 0.0f;
    cfg->stats.startCount  = 0;
    cfg->stats.lastStartTs = 0;
    cfg->stats.lastStopTs  = 0;

    cfg->configTimestamp = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FLASH READ / WRITE
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool ReadFromFlash(FlashConfig_t* cfg)
{
    const FlashConfig_t* flash_cfg = (const FlashConfig_t*)FLASH_CONFIG_ADDR;

    if (flash_cfg->magic != FLASH_CONFIG_MAGIC) return false;

    /* Reject old v1 config (Firebase layout) — force re-provisioning */
    if (flash_cfg->version < 2) return false;

    memcpy(cfg, flash_cfg, sizeof(FlashConfig_t));

    uint16_t calcCrc = FlashConfig_CRC16(
        (const uint8_t*)cfg,
        sizeof(FlashConfig_t) - sizeof(uint16_t)
    );

    return (calcCrc == cfg->crc);
}

static bool WriteToFlash(const FlashConfig_t* cfg)
{
    HAL_StatusTypeDef status;
    bool result = false;

    /* H-NEW-6 fix: STM32F103 flash erase/program stalls the instruction bus.
     * Any ISR that reads flash during this window will hard fault.
     * Enter critical section to mask all interrupts during flash operations. */
    taskENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) { taskEXIT_CRITICAL(); return false; }

    FLASH_EraseInitTypeDef erase;
    uint32_t pageError = 0;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_CONFIG_ADDR;
    erase.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        taskEXIT_CRITICAL();
        return false;
    }

    const uint16_t* src = (const uint16_t*)cfg;
    uint32_t addr = FLASH_CONFIG_ADDR;
    uint16_t words = (sizeof(FlashConfig_t) + 1) / 2;

    result = true;
    for (uint16_t i = 0; i < words; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, src[i]);
        if (status != HAL_OK) {
            result = false;
            break;
        }
        addr += 2;
    }

    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
    return result;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool FlashConfig_Init(void)
{
    if (ReadFromFlash(&g_config)) {
        g_dirty = false;
        return true;
    }

    ApplyDefaults(&g_config);
    g_dirty = true;
    FlashConfig_Save();
    return false;
}

FlashConfig_t* FlashConfig_Get(void) { return &g_config; }

bool FlashConfig_Save(void)
{
    g_config.crc = FlashConfig_CRC16(
        (const uint8_t*)&g_config,
        sizeof(FlashConfig_t) - sizeof(uint16_t)
    );

    bool ok = WriteToFlash(&g_config);
    if (ok) g_dirty = false;
    return ok;
}

void FlashConfig_FactoryReset(void)
{
    ApplyDefaults(&g_config);
    FlashConfig_Save();
}

/* ─── Setters ──────────────────────────────────────────────────── */

bool FlashConfig_SetDeviceId(const char* id)
{
    if (!id || strlen(id) >= DEVICE_ID_MAX_LEN) return false;
    memset(g_config.deviceId, 0, DEVICE_ID_MAX_LEN);
    strncpy(g_config.deviceId, id, DEVICE_ID_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetMqttBrokerIp(const char* ip)
{
    if (!ip || strlen(ip) >= MQTT_BROKER_IP_MAX_LEN) return false;
    memset(g_config.mqttBrokerIp, 0, MQTT_BROKER_IP_MAX_LEN);
    strncpy(g_config.mqttBrokerIp, ip, MQTT_BROKER_IP_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetMqttBrokerPort(uint16_t port)
{
    g_config.mqttBrokerPort = port;
    g_dirty = true;
    return true;
}

bool FlashConfig_SetMqttUsername(const char* username)
{
    if (!username || strlen(username) >= MQTT_USERNAME_MAX_LEN) return false;
    memset(g_config.mqttUsername, 0, MQTT_USERNAME_MAX_LEN);
    strncpy(g_config.mqttUsername, username, MQTT_USERNAME_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetMqttPassword(const char* password)
{
    if (!password || strlen(password) >= MQTT_PASSWORD_MAX_LEN) return false;
    memset(g_config.mqttPassword, 0, MQTT_PASSWORD_MAX_LEN);
    strncpy(g_config.mqttPassword, password, MQTT_PASSWORD_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetFarmId(const char* farmId)
{
    if (!farmId || strlen(farmId) >= FARM_ID_MAX_LEN) return false;
    memset(g_config.farmId, 0, FARM_ID_MAX_LEN);
    strncpy(g_config.farmId, farmId, FARM_ID_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

void FlashConfig_SetProtection(const ProtectionConfig_t* prot)
{
    memcpy(&g_config.protection, prot, sizeof(ProtectionConfig_t));
    g_dirty = true;
}

void FlashConfig_SetCalibration(const CalibrationData_t* cal)
{
    memcpy(&g_config.calibration, cal, sizeof(CalibrationData_t));
    g_dirty = true;
}

void FlashConfig_SetStarDeltaDelay(float delaySec)
{
    if (delaySec < 1.0f) delaySec = 1.0f;
    if (delaySec > SD_DELAY_MAX) delaySec = SD_DELAY_MAX;
    g_config.starDeltaDelay = (uint16_t)(delaySec * 10.0f);
    g_dirty = true;
}

void FlashConfig_SetMode(MotorMode_t mode)
{
    g_config.mode = mode;
    g_dirty = true;
}

void FlashConfig_UpdateStats(const MotorStats_t* stats)
{
    memcpy(&g_config.stats, stats, sizeof(MotorStats_t));
    g_dirty = true;
}

void FlashConfig_SetConfigTimestamp(uint32_t ts)
{
    g_config.configTimestamp = ts;
    g_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UART PROVISIONING COMMANDS
 *
 * Format: "COMMAND ARGS\n"
 *   SET_ID MOT-001-A1          — Device ID
 *   SET_BROKER 192.168.1.100   — MQTT broker IP
 *   SET_PORT 1883              — MQTT broker port
 *   SET_MQTT_USER mqtt_user    — MQTT username
 *   SET_MQTT_PASS mqtt_pass    — MQTT password
 *   SET_FARM farm01            — Farm ID for MQTT topics
 *   GET_CONFIG                 — Print current config
 *   FACTORY_RESET              — Reset to compiled defaults
 *   SAVE                       — Write current config to flash
 * ═══════════════════════════════════════════════════════════════════════════ */

void FlashConfig_ProcessCommand(const char* cmd, char* response, uint16_t respLen)
{
    if (!cmd || !response || respLen < 32) return;

    if (strncmp(cmd, "SET_ID ", 7) == 0) {
        if (FlashConfig_SetDeviceId(cmd + 7)) {
            snprintf(response, respLen, "OK ID=%s\r\n", g_config.deviceId);
        } else {
            snprintf(response, respLen, "ERR ID too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_BROKER ", 11) == 0) {
        if (FlashConfig_SetMqttBrokerIp(cmd + 11)) {
            snprintf(response, respLen, "OK BROKER=%s\r\n", g_config.mqttBrokerIp);
        } else {
            snprintf(response, respLen, "ERR IP too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_PORT ", 9) == 0) {
        uint16_t port = (uint16_t)atoi(cmd + 9);
        if (port > 0) {
            FlashConfig_SetMqttBrokerPort(port);
            snprintf(response, respLen, "OK PORT=%u\r\n", g_config.mqttBrokerPort);
        } else {
            snprintf(response, respLen, "ERR Invalid port\r\n");
        }
    }
    else if (strncmp(cmd, "SET_MQTT_USER ", 14) == 0) {
        if (FlashConfig_SetMqttUsername(cmd + 14)) {
            snprintf(response, respLen, "OK MQTT_USER set\r\n");
        } else {
            snprintf(response, respLen, "ERR Username too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_MQTT_PASS ", 14) == 0) {
        if (FlashConfig_SetMqttPassword(cmd + 14)) {
            snprintf(response, respLen, "OK MQTT_PASS set\r\n");
        } else {
            snprintf(response, respLen, "ERR Password too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_FARM ", 9) == 0) {
        if (FlashConfig_SetFarmId(cmd + 9)) {
            snprintf(response, respLen, "OK FARM=%s\r\n", g_config.farmId);
        } else {
            snprintf(response, respLen, "ERR Farm ID too long\r\n");
        }
    }
    else if (strncmp(cmd, "GET_CONFIG", 10) == 0) {
        snprintf(response, respLen,
            "ID=%s BROKER=%s:%u FARM=%s HP=%.1f I=%.1fA SD=%u.%us MODE=%d\r\n",
            g_config.deviceId,
            g_config.mqttBrokerIp,
            g_config.mqttBrokerPort,
            g_config.farmId,
            g_config.calibration.ratedHP,
            g_config.calibration.ratedCurrent,
            g_config.starDeltaDelay / 10,
            g_config.starDeltaDelay % 10,
            g_config.mode
        );
    }
    else if (strncmp(cmd, "FACTORY_RESET", 13) == 0) {
        FlashConfig_FactoryReset();
        snprintf(response, respLen, "OK Factory reset done\r\n");
    }
    else if (strncmp(cmd, "SAVE", 4) == 0) {
        if (FlashConfig_Save()) {
            snprintf(response, respLen, "OK Config saved to flash\r\n");
        } else {
            snprintf(response, respLen, "ERR Flash write failed\r\n");
        }
    }
    else {
        snprintf(response, respLen, "ERR Unknown cmd\r\n");
    }
}
