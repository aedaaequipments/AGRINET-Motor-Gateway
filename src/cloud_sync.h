/**
 * @file cloud_sync.h
 * @brief MQTT Cloud Sync — data push, command subscribe, config sync
 *
 * C1/C2 fix: Replaces Firebase HTTP polling with MQTT publish/subscribe.
 * Matches Sasyamithra self-hosted server MQTT topic schema:
 *   data/{farmId}/{deviceId}/telemetry  - motor telemetry (publish)
 *   data/{farmId}/{stationId}/weather   - weather readings (publish)
 *   cmd/+/{deviceId}/set                - motor commands (subscribe)
 *   alert/{farmId}/{deviceId}/warn      - fault alerts (publish)
 *   status/{farmId}/{deviceId}/heartbeat - online status (publish)
 */

#ifndef CLOUD_SYNC_H
#define CLOUD_SYNC_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize cloud sync + MQTT connection (call after GSM_Init)
 */
void CloudSync_Init(void);

/**
 * @brief Push motor telemetry via MQTT
 * Publishes to: data/{farmId}/{deviceId}/telemetry
 */
bool CloudSync_PushMotorData(const PowerSnapshot_t* power,
                              bool running, HealthStatus_t health);

/**
 * @brief Push weather station data via MQTT
 * Publishes to: data/{farmId}/{stationId}/weather
 */
bool CloudSync_PushWeatherData(const WeatherPayload_t* weather,
                                const char* stationId);

/**
 * @brief Push valve status via MQTT
 * Publishes to: data/{farmId}/{valveId}/telemetry
 */
bool CloudSync_PushValveStatus(const ValvePayload_t* valve,
                                const char* valveId);

/**
 * @brief Push alert via MQTT
 * Publishes to: alert/{farmId}/{deviceId}/warn
 */
bool CloudSync_PushAlert(const char* name, const char* type,
                          const char* severity, const char* action);

/**
 * @brief Log command via MQTT
 * Publishes to: data/{farmId}/{deviceId}/cmdlog
 */
bool CloudSync_LogCommand(const char* deviceType, const char* deviceId,
                           const char* action);

/**
 * @brief Update heartbeat (online status + signal quality)
 * Publishes to: status/{farmId}/{deviceId}/heartbeat
 */
bool CloudSync_Heartbeat(void);

/**
 * @brief Run the cloud sync cycle (call from Task_CloudSync)
 * Handles: MQTT connection, push telemetry, process incoming, heartbeat
 */
void CloudSync_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOUD_SYNC_H */
