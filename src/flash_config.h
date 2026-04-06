/**
 * @file flash_config.h
 * @brief Persistent configuration storage in STM32 flash
 *
 * Manages read/write of FlashConfig_t to the last flash page (63).
 * Handles config validation (magic + CRC), factory defaults, and
 * UART provisioning for device ID and MQTT broker credentials.
 */

#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize flash config - read from flash or set defaults
 * @return true if valid config loaded from flash, false if defaults applied
 */
bool FlashConfig_Init(void);

/**
 * @brief Get pointer to current config (in RAM)
 */
FlashConfig_t* FlashConfig_Get(void);

/**
 * @brief Save current config to flash
 * @return true on success
 */
bool FlashConfig_Save(void);

/**
 * @brief Reset config to factory defaults and save
 */
void FlashConfig_FactoryReset(void);

/* ─── Setters ──────────────────────────────────────────────────── */

bool FlashConfig_SetDeviceId(const char* id);
bool FlashConfig_SetMqttBrokerIp(const char* ip);
bool FlashConfig_SetMqttBrokerPort(uint16_t port);
bool FlashConfig_SetMqttUsername(const char* username);
bool FlashConfig_SetMqttPassword(const char* password);
bool FlashConfig_SetFarmId(const char* farmId);

void FlashConfig_SetProtection(const ProtectionConfig_t* prot);
void FlashConfig_SetCalibration(const CalibrationData_t* cal);
void FlashConfig_SetStarDeltaDelay(float delaySec);
void FlashConfig_SetMode(MotorMode_t mode);
void FlashConfig_UpdateStats(const MotorStats_t* stats);
void FlashConfig_SetConfigTimestamp(uint32_t ts);

/**
 * @brief Process UART provisioning command
 * @param cmd Command string (e.g., "SET_ID MOT-001-A1", "SET_BROKER 192.168.1.100")
 * @param response Buffer for response message
 * @param respLen Size of response buffer
 */
void FlashConfig_ProcessCommand(const char* cmd, char* response, uint16_t respLen);

/**
 * @brief Calculate CRC-16 for config validation
 */
uint16_t FlashConfig_CRC16(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CONFIG_H */
