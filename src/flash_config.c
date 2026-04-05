/**
 * @file flash_config.c
 * @brief Persistent configuration in STM32 internal flash (last 1KB page)
 */

#include "flash_config.h"
#include "credentials.h"
#include <string.h>
#include <stdio.h>

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
    cfg->version = 1;

    /* Device identity defaults from credentials.h */
    strncpy(cfg->deviceId,      DEFAULT_DEVICE_ID,       DEVICE_ID_MAX_LEN - 1);
    strncpy(cfg->firebaseUid,   FIREBASE_DEFAULT_UID,    FIREBASE_UID_MAX_LEN - 1);
    strncpy(cfg->firebaseToken, FIREBASE_DEFAULT_TOKEN,   FIREBASE_TOKEN_MAX_LEN - 1);
    strncpy(cfg->firebaseUrl,   FIREBASE_DEFAULT_URL,    FIREBASE_URL_MAX_LEN - 1);

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
    cfg->starDeltaDelay = DEFAULT_STAR_DELTA_SEC * 10;  // tenths of seconds
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
    /* Read directly from flash memory-mapped address */
    const FlashConfig_t* flash_cfg = (const FlashConfig_t*)FLASH_CONFIG_ADDR;

    /* Validate magic */
    if (flash_cfg->magic != FLASH_CONFIG_MAGIC) {
        return false;
    }

    /* Copy to RAM */
    memcpy(cfg, flash_cfg, sizeof(FlashConfig_t));

    /* Validate CRC (CRC field itself is excluded) */
    uint16_t calcCrc = FlashConfig_CRC16(
        (const uint8_t*)cfg,
        sizeof(FlashConfig_t) - sizeof(uint16_t)  // exclude crc field
    );

    if (calcCrc != cfg->crc) {
        return false;
    }

    return true;
}

static bool WriteToFlash(const FlashConfig_t* cfg)
{
    HAL_StatusTypeDef status;

    /* Unlock flash */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;

    /* Erase the config page */
    FLASH_EraseInitTypeDef erase;
    uint32_t pageError = 0;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_CONFIG_ADDR;
    erase.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write config as half-words (STM32F1 flash programs in 16-bit units) */
    const uint16_t* src = (const uint16_t*)cfg;
    uint32_t addr = FLASH_CONFIG_ADDR;
    uint16_t words = (sizeof(FlashConfig_t) + 1) / 2;  // round up

    for (uint16_t i = 0; i < words; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 2;
    }

    HAL_FLASH_Lock();
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool FlashConfig_Init(void)
{
    if (ReadFromFlash(&g_config)) {
        g_dirty = false;
        return true;  // Valid config loaded
    }

    /* No valid config -- apply factory defaults */
    ApplyDefaults(&g_config);
    g_dirty = true;
    FlashConfig_Save();  // Write defaults to flash
    return false;
}

FlashConfig_t* FlashConfig_Get(void)
{
    return &g_config;
}

bool FlashConfig_Save(void)
{
    /* Calculate CRC before writing */
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

bool FlashConfig_SetDeviceId(const char* id)
{
    if (!id || strlen(id) >= DEVICE_ID_MAX_LEN) return false;
    memset(g_config.deviceId, 0, DEVICE_ID_MAX_LEN);
    strncpy(g_config.deviceId, id, DEVICE_ID_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetFirebaseUid(const char* uid)
{
    if (!uid || strlen(uid) >= FIREBASE_UID_MAX_LEN) return false;
    memset(g_config.firebaseUid, 0, FIREBASE_UID_MAX_LEN);
    strncpy(g_config.firebaseUid, uid, FIREBASE_UID_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetFirebaseToken(const char* token)
{
    if (!token || strlen(token) >= FIREBASE_TOKEN_MAX_LEN) return false;
    memset(g_config.firebaseToken, 0, FIREBASE_TOKEN_MAX_LEN);
    strncpy(g_config.firebaseToken, token, FIREBASE_TOKEN_MAX_LEN - 1);
    g_dirty = true;
    return true;
}

bool FlashConfig_SetFirebaseUrl(const char* url)
{
    if (!url || strlen(url) >= FIREBASE_URL_MAX_LEN) return false;
    memset(g_config.firebaseUrl, 0, FIREBASE_URL_MAX_LEN);
    strncpy(g_config.firebaseUrl, url, FIREBASE_URL_MAX_LEN - 1);
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
 *   SET_ID MOT-001-A1
 *   SET_UID abc123def456
 *   SET_TOKEN ffMQgzF2kXETIU...
 *   SET_URL https://project.firebaseio.com
 *   GET_CONFIG
 *   FACTORY_RESET
 *   SAVE
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
    else if (strncmp(cmd, "SET_UID ", 8) == 0) {
        if (FlashConfig_SetFirebaseUid(cmd + 8)) {
            snprintf(response, respLen, "OK UID set\r\n");
        } else {
            snprintf(response, respLen, "ERR UID too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_TOKEN ", 10) == 0) {
        if (FlashConfig_SetFirebaseToken(cmd + 10)) {
            snprintf(response, respLen, "OK TOKEN set\r\n");
        } else {
            snprintf(response, respLen, "ERR TOKEN too long\r\n");
        }
    }
    else if (strncmp(cmd, "SET_URL ", 8) == 0) {
        if (FlashConfig_SetFirebaseUrl(cmd + 8)) {
            snprintf(response, respLen, "OK URL set\r\n");
        } else {
            snprintf(response, respLen, "ERR URL too long\r\n");
        }
    }
    else if (strncmp(cmd, "GET_CONFIG", 10) == 0) {
        snprintf(response, respLen,
            "ID=%s UID=%s HP=%.1f I=%.1fA SD=%u.%us MODE=%d SAFE=%d FORCE=%d\r\n",
            g_config.deviceId,
            g_config.firebaseUid,
            g_config.calibration.ratedHP,
            g_config.calibration.ratedCurrent,
            g_config.starDeltaDelay / 10,
            g_config.starDeltaDelay % 10,
            g_config.mode,
            g_config.safeMode,
            g_config.forceRun
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
