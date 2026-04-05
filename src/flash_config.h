/**
 * @file flash_config.h
 * @brief Persistent configuration storage in STM32 flash
 *
 * Manages read/write of FlashConfig_t to the last flash page (63).
 * Handles config validation (magic + CRC), factory defaults, and
 * UART provisioning for device ID and Firebase credentials.
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
 * @return Pointer to the active FlashConfig_t
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

/**
 * @brief Update device ID
 * @param id Null-terminated string (max DEVICE_ID_MAX_LEN-1 chars)
 * @return true on success
 */
bool FlashConfig_SetDeviceId(const char* id);

/**
 * @brief Update Firebase UID
 * @param uid Null-terminated string
 * @return true on success
 */
bool FlashConfig_SetFirebaseUid(const char* uid);

/**
 * @brief Update Firebase auth token
 * @param token Null-terminated string
 * @return true on success
 */
bool FlashConfig_SetFirebaseToken(const char* token);

/**
 * @brief Update Firebase database URL
 * @param url Null-terminated string
 * @return true on success
 */
bool FlashConfig_SetFirebaseUrl(const char* url);

/**
 * @brief Update protection thresholds
 * @param prot New protection config
 */
void FlashConfig_SetProtection(const ProtectionConfig_t* prot);

/**
 * @brief Update calibration data from auto-detect
 * @param cal New calibration data
 */
void FlashConfig_SetCalibration(const CalibrationData_t* cal);

/**
 * @brief Update star-delta delay (in tenths of seconds)
 * @param delaySec Delay in seconds (float, stored as x10 uint16)
 */
void FlashConfig_SetStarDeltaDelay(float delaySec);

/**
 * @brief Update operating mode
 * @param mode MODE_MANUAL or MODE_AUTO
 */
void FlashConfig_SetMode(MotorMode_t mode);

/**
 * @brief Update motor stats (call periodically to persist kWh, hours)
 * @param stats Motor statistics
 */
void FlashConfig_UpdateStats(const MotorStats_t* stats);

/**
 * @brief Update config sync timestamp
 * @param ts Unix timestamp
 */
void FlashConfig_SetConfigTimestamp(uint32_t ts);

/**
 * @brief Process UART provisioning command
 * @param cmd Command string (e.g., "SET_ID MOT-001-A1")
 * @param response Buffer for response message
 * @param respLen Size of response buffer
 */
void FlashConfig_ProcessCommand(const char* cmd, char* response, uint16_t respLen);

/**
 * @brief Calculate CRC-16 for config validation
 * @param data Pointer to data
 * @param len Length in bytes
 * @return CRC-16 value
 */
uint16_t FlashConfig_CRC16(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CONFIG_H */
