/**
 * @file cloud_sync.h
 * @brief Firebase Cloud Sync - data push, command polling, config sync
 *
 * Matches Sasyamithra App v6.2 Firebase schema:
 *   /users/{uid}/motors/{motorId}/   - motor telemetry + config
 *   /users/{uid}/valves/{valveId}/   - valve status
 *   /users/{uid}/sensorLog/{stationId}/ - weather readings
 *   /users/{uid}/alerts/             - fault alerts
 *   /users/{uid}/cmdLog/             - command history
 */

#ifndef CLOUD_SYNC_H
#define CLOUD_SYNC_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize cloud sync (call after GSM_Init)
 */
void CloudSync_Init(void);

/**
 * @brief Push motor telemetry to Firebase
 * PATCH /users/{uid}/motors/{motorId}
 */
bool CloudSync_PushMotorData(const PowerSnapshot_t* power,
                              bool running, HealthStatus_t health);

/**
 * @brief Poll for motor commands from Firebase
 * GET /users/{uid}/motors/{motorId}
 * Reads: run, forceRun, safeMode, mode, prot.*, starDelta
 * @return true if new command received
 */
bool CloudSync_PollMotorCommands(void);

/**
 * @brief Push weather station data to Firebase
 * POST /users/{uid}/sensorLog/{stationId}
 */
bool CloudSync_PushWeatherData(const WeatherPayload_t* weather,
                                const char* stationId);

/**
 * @brief Push valve status to Firebase
 * PATCH /users/{uid}/valves/{valveId}
 */
bool CloudSync_PushValveStatus(const ValvePayload_t* valve,
                                const char* valveId);

/**
 * @brief Poll valve commands from Firebase
 * GET /users/{uid}/valves/{valveId}
 * @param valveId Valve device ID
 * @param isOpen Output: requested valve state
 * @return true if command received
 */
bool CloudSync_PollValveCommand(const char* valveId, bool* isOpen);

/**
 * @brief Push alert to Firebase
 * POST /users/{uid}/alerts
 */
bool CloudSync_PushAlert(const char* name, const char* type,
                          const char* severity, const char* action);

/**
 * @brief Log command to Firebase
 * POST /users/{uid}/cmdLog
 */
bool CloudSync_LogCommand(const char* deviceType, const char* deviceId,
                           const char* action);

/**
 * @brief Update heartbeat (online status + lastSeen)
 */
bool CloudSync_Heartbeat(void);

/**
 * @brief Sync config with Firebase (bidirectional)
 * Compares timestamps, pulls if app is newer, pushes if local is newer
 */
bool CloudSync_SyncConfig(void);

/**
 * @brief Run the cloud sync cycle (call from Task_CloudSync)
 * Handles: push telemetry, poll commands, heartbeat, config sync
 */
void CloudSync_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOUD_SYNC_H */
