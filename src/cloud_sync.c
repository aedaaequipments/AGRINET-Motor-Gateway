/**
 * @file cloud_sync.c
 * @brief Firebase Cloud Sync matching Sasyamithra App v6.2 schema
 *
 * All JSON built with snprintf (no String class, no dynamic allocation).
 * Firebase REST API via GSM HTTP driver.
 */

#include "cloud_sync.h"
#include "gsm_driver.h"
#include "flash_config.h"
#include "motor_control.h"
#include "power_monitor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static char s_jsonBuf[JSON_BUF_SIZE];
static char s_respBuf[256];
static char s_pathBuf[128];
static uint32_t s_lastPushTick = 0;
static uint32_t s_lastPollTick = 0;
static uint32_t s_lastHeartbeatTick = 0;
static uint32_t s_lastConfigSyncTick = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT
 * ═══════════════════════════════════════════════════════════════════════════ */

void CloudSync_Init(void)
{
    s_lastPushTick = HAL_GetTick();
    s_lastPollTick = HAL_GetTick();
    s_lastHeartbeatTick = HAL_GetTick();
    s_lastConfigSyncTick = HAL_GetTick();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * JSON BUILDERS (snprintf, no dynamic allocation)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Build motor telemetry JSON matching app schema:
 * {
 *   "run": true, "phV": {"r": 231, "y": 229, "b": 230},
 *   "phI": {"r": 4.2, "y": 4.1, "b": 4.3},
 *   "pf": {"o": 0.92, "r": 0.91, "y": 0.93, "b": 0.92},
 *   "pwr": 2.5, "hp": "good"
 * }
 */
static uint16_t BuildMotorJson(const PowerSnapshot_t* p, bool running,
                                HealthStatus_t health)
{
    const char* hpStr = "good";
    if (health == HEALTH_WARNING) hpStr = "warning";
    else if (health == HEALTH_CRITICAL) hpStr = "critical";

    FlashConfig_t* cfg = FlashConfig_Get();

    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"run\":%s,"
        "\"phV\":{\"r\":%.0f,\"y\":%.0f,\"b\":%.0f},"
        "\"phI\":{\"r\":%.1f,\"y\":%.1f,\"b\":%.1f},"
        "\"pf\":{\"o\":%.2f,\"r\":%.2f,\"y\":%.2f,\"b\":%.2f},"
        "\"pwr\":%.2f,"
        "\"kwh\":%.1f,"
        "\"hrs\":%.1f,"
        "\"hp\":\"%s\","
        "\"mode\":\"%s\","
        "\"forceRun\":%s,"
        "\"safeMode\":%s}",
        running ? "true" : "false",
        p->r.voltage, p->y.voltage, p->b.voltage,
        p->r.current, p->y.current, p->b.current,
        p->avgPF, p->r.powerFactor, p->y.powerFactor, p->b.powerFactor,
        p->totalPower,
        cfg->stats.totalKwh,
        cfg->stats.totalHours,
        hpStr,
        cfg->mode == MODE_AUTO ? "auto" : "manual",
        cfg->forceRun ? "true" : "false",
        cfg->safeMode ? "true" : "false"
    );
}

/**
 * Build weather sensor JSON matching app sensorLog schema
 */
static uint16_t BuildWeatherJson(const WeatherPayload_t* w)
{
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"airT\":%.1f,\"soilT\":%.1f,\"humid\":%.1f,"
        "\"soilM\":%.1f,\"soilM1\":%.1f,\"soilM2\":%.1f,\"soilM3\":%.1f,"
        "\"light\":%.1f,\"leafW\":%u,\"leafW2\":%u,"
        "\"wind\":%.1f,\"rain\":%.1f,\"batt\":%.0f}",
        (float)w->airTemp / 100.0f,
        (float)w->soilTemp / 100.0f,
        (float)w->humidity / 100.0f,
        (float)w->soilM2 / 100.0f,   // soilM = L2 (primary)
        (float)w->soilM1 / 100.0f,
        (float)w->soilM2 / 100.0f,
        (float)w->soilM3 / 100.0f,
        (float)w->lightLux * 10.0f / 655.35f,  // Normalize to 0-100%
        w->leafWet1,
        w->leafWet2,
        (float)w->windSpeed / 100.0f,
        (float)w->rainMm / 10.0f,
        (float)w->batteryMv / 1000.0f * 100.0f  // mV to %
    );
}

/**
 * Build valve status JSON
 */
static uint16_t BuildValveJson(const ValvePayload_t* v)
{
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"isOpen\":%s,\"flow\":%.1f}",
        v->position > 50 ? "true" : "false",
        (float)v->flowRate / 10.0f
    );
}

/**
 * Build alert JSON
 */
static uint16_t BuildAlertJson(const char* name, const char* type,
                                const char* severity, const char* action)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"name\":\"%s\",\"type\":\"%s\",\"sev\":\"%s\","
        "\"area\":\"Motor %s\",\"action\":\"%s\",\"ack\":false}",
        name, type, severity, cfg->deviceId, action
    );
}

/**
 * Build command log JSON
 */
static uint16_t BuildCmdLogJson(const char* devType, const char* devId,
                                 const char* action)
{
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"type\":\"%s\",\"deviceId\":\"%s\",\"action\":\"%s\"}",
        devType, devId, action
    );
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool CloudSync_PushMotorData(const PowerSnapshot_t* power,
                              bool running, HealthStatus_t health)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    BuildMotorJson(power, running, health);

    snprintf(s_pathBuf, sizeof(s_pathBuf), "motors/%s", cfg->deviceId);
    uint16_t status = GSM_HttpRequest(HTTP_PUT, s_pathBuf, s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_PollMotorCommands(void)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    snprintf(s_pathBuf, sizeof(s_pathBuf), "motors/%s", cfg->deviceId);

    uint16_t status = GSM_HttpRequest(HTTP_GET, s_pathBuf, NULL,
                                       s_respBuf, sizeof(s_respBuf));
    if (status < 200 || status >= 300) return false;

    /* Parse "run" field */
    char* p = strstr(s_respBuf, "\"run\":");
    if (p) {
        p += 6;
        bool remoteRun = (strncmp(p, "true", 4) == 0);
        bool localRun = MotorControl_IsRunning();

        if (remoteRun && !localRun) {
            MotorControl_Start();
            CloudSync_LogCommand("motor", cfg->deviceId, "start");
        } else if (!remoteRun && localRun) {
            MotorControl_Stop(FAULT_REMOTE_STOP);
            CloudSync_LogCommand("motor", cfg->deviceId, "stop");
        }
    }

    /* Parse "forceRun" */
    p = strstr(s_respBuf, "\"forceRun\":");
    if (p) {
        p += 11;
        MotorControl_SetForceRun(strncmp(p, "true", 4) == 0);
    }

    /* Parse "safeMode" -- field name in app may be just part of motor obj */
    p = strstr(s_respBuf, "\"safeMode\":");
    if (p) {
        p += 11;
        MotorControl_SetSafeMode(strncmp(p, "true", 4) == 0);
    }

    /* Parse "starDelta" */
    p = strstr(s_respBuf, "\"starDelta\":");
    if (p) {
        p += 12;
        float sd = strtof(p, NULL);
        if (sd >= 1.0f && sd <= 30.0f) {
            FlashConfig_SetStarDeltaDelay(sd);
        }
    }

    /* Parse protection thresholds */
    p = strstr(s_respBuf, "\"overV\":");
    if (p) {
        p += 8;
        float v = strtof(p, NULL);
        if (v > 200.0f && v < 500.0f) {
            ProtectionConfig_t prot;
            memcpy(&prot, &cfg->protection, sizeof(prot));
            prot.overVoltage = v;

            /* Also parse other prot fields */
            char* q = strstr(s_respBuf, "\"underV\":");
            if (q) { q += 9; prot.underVoltage = strtof(q, NULL); }

            q = strstr(s_respBuf, "\"overload\":");
            if (q) { q += 11; prot.overloadAmps = strtof(q, NULL); }

            q = strstr(s_respBuf, "\"dryRun\":");
            if (q) { q += 9; prot.dryRunSec = (uint16_t)strtof(q, NULL); }

            q = strstr(s_respBuf, "\"restart\":");
            if (q) { q += 10; prot.restartDelay = (uint16_t)strtof(q, NULL); }

            FlashConfig_SetProtection(&prot);
        }
    }

    return true;
}

bool CloudSync_PushWeatherData(const WeatherPayload_t* weather,
                                const char* stationId)
{
    if (!GSM_IsReady()) return false;

    BuildWeatherJson(weather);
    snprintf(s_pathBuf, sizeof(s_pathBuf), "sensorLog/%s", stationId);
    uint16_t status = GSM_HttpRequest(HTTP_POST, s_pathBuf, s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_PushValveStatus(const ValvePayload_t* valve,
                                const char* valveId)
{
    if (!GSM_IsReady()) return false;

    BuildValveJson(valve);
    snprintf(s_pathBuf, sizeof(s_pathBuf), "valves/%s", valveId);
    uint16_t status = GSM_HttpRequest(HTTP_PUT, s_pathBuf, s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_PollValveCommand(const char* valveId, bool* isOpen)
{
    if (!GSM_IsReady()) return false;

    snprintf(s_pathBuf, sizeof(s_pathBuf), "valves/%s", valveId);
    uint16_t status = GSM_HttpRequest(HTTP_GET, s_pathBuf, NULL,
                                       s_respBuf, sizeof(s_respBuf));
    if (status < 200 || status >= 300) return false;

    char* p = strstr(s_respBuf, "\"isOpen\":");
    if (p) {
        p += 9;
        *isOpen = (strncmp(p, "true", 4) == 0);
        return true;
    }
    return false;
}

bool CloudSync_PushAlert(const char* name, const char* type,
                          const char* severity, const char* action)
{
    if (!GSM_IsReady()) return false;

    BuildAlertJson(name, type, severity, action);
    uint16_t status = GSM_HttpRequest(HTTP_POST, "alerts", s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_LogCommand(const char* deviceType, const char* deviceId,
                           const char* action)
{
    if (!GSM_IsReady()) return false;

    BuildCmdLogJson(deviceType, deviceId, action);
    uint16_t status = GSM_HttpRequest(HTTP_POST, "cmdLog", s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_Heartbeat(void)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"online\":true,\"signal\":%u}",
        GSM_GetSignalQuality());

    snprintf(s_pathBuf, sizeof(s_pathBuf), "motors/%s/connectivity", cfg->deviceId);
    uint16_t status = GSM_HttpRequest(HTTP_PUT, s_pathBuf, s_jsonBuf, NULL, 0);
    return (status >= 200 && status < 300);
}

bool CloudSync_SyncConfig(void)
{
    /* For now, just pull config from Firebase.
     * Full bidirectional sync with timestamps will be added in Phase 3. */
    return CloudSync_PollMotorCommands();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN UPDATE LOOP (called from Task_CloudSync)
 * ═══════════════════════════════════════════════════════════════════════════ */

void CloudSync_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* Run GSM state machine (error recovery) */
    GSM_Update();

    if (!GSM_IsReady()) return;

    /* Push motor telemetry every CLOUD_PUSH_PERIOD_MS */
    if ((now - s_lastPushTick) >= CLOUD_PUSH_PERIOD_MS) {
        s_lastPushTick = now;

        PowerSnapshot_t snap;
        PowerMonitor_GetSnapshot(&snap);
        bool running = MotorControl_IsRunning();
        HealthStatus_t health = PowerMonitor_AssessHealth();

        CloudSync_PushMotorData(&snap, running, health);

        /* Push fault alert if motor is in fault state */
        FaultCode_t fault = MotorControl_GetFault();
        if (fault != FAULT_NONE && fault != FAULT_MANUAL_STOP &&
            fault != FAULT_REMOTE_STOP) {
            CloudSync_PushAlert(
                MotorControl_FaultString(fault),
                "device", "critical",
                "Check motor and power supply"
            );
        }
    }

    /* Poll commands every CLOUD_POLL_PERIOD_MS */
    if ((now - s_lastPollTick) >= CLOUD_POLL_PERIOD_MS) {
        s_lastPollTick = now;
        CloudSync_PollMotorCommands();
    }

    /* Heartbeat every HEARTBEAT_PERIOD_MS */
    if ((now - s_lastHeartbeatTick) >= HEARTBEAT_PERIOD_MS) {
        s_lastHeartbeatTick = now;
        CloudSync_Heartbeat();
    }

    /* Config sync every 60 seconds */
    if ((now - s_lastConfigSyncTick) >= 60000) {
        s_lastConfigSyncTick = now;
        CloudSync_SyncConfig();
    }
}
