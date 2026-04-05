/**
 * @file lora_manager.h
 * @brief LoRa Network Manager - coordinates weather station and valve nodes
 *
 * Master role: polls weather stations, relays valve commands, sends ACKs.
 * Uses AGRINET protocol with CRC-16 validation.
 * IN865 channel hopping with 1% duty cycle tracking.
 */

#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LoRa manager (radio + protocol)
 * @return true if radio initialized
 */
bool LoRaManager_Init(void);

/**
 * @brief Run LoRa manager cycle (call from Task_LoRaManager)
 *
 * - Check for received packets
 * - Parse and dispatch by type (weather data, valve ACK, etc.)
 * - Handle channel hopping
 * - Process pending TX queue
 */
void LoRaManager_Update(void);

/**
 * @brief Send valve command via LoRa
 * @param dstUid Target valve node UID
 * @param position Target position (0-100)
 * @return true if sent (ACK not guaranteed)
 */
bool LoRaManager_SendValveCommand(uint64_t dstUid, uint8_t position);

/**
 * @brief Request weather data from a station (sends poll command)
 * @param dstUid Target weather station UID
 * @return true if poll sent
 */
bool LoRaManager_PollWeatherStation(uint64_t dstUid);

/**
 * @brief Get last weather data received from any station
 * @param data Output weather payload
 * @param stationUid Output station UID
 * @return true if valid data available
 */
bool LoRaManager_GetLastWeatherData(WeatherPayload_t* data, uint64_t* stationUid);

/**
 * @brief Check if new weather data is available (since last read)
 */
bool LoRaManager_HasNewWeatherData(void);

/**
 * @brief Get master UID
 */
uint64_t LoRaManager_GetMasterUid(void);

#ifdef __cplusplus
}
#endif

#endif /* LORA_MANAGER_H */
