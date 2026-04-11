/**
 * @file cloud_sync.c
 * @brief MQTT Cloud Sync for Sasyamithra self-hosted server (C1/C2 fix)
 *
 * Replaces Firebase HTTP REST with MQTT publish/subscribe.
 * All JSON built with snprintf (no String class, no dynamic allocation).
 * Motor commands received via subscription callback instead of polling.
 */

#include "cloud_sync.h"
#include "gsm_driver.h"
#include "flash_config.h"
#include "motor_control.h"
#include "power_monitor.h"
#include "offline_queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static char s_jsonBuf[JSON_BUF_SIZE];
static char s_topicBuf[128];
static uint32_t s_lastPushTick = 0;
static uint32_t s_lastHeartbeatTick = 0;
static bool s_mqttSubscribed = false;

/* ═══════════════════════════════════════════════════════════════════════════
 * MQTT INCOMING MESSAGE HANDLER
 * Replaces Firebase HTTP polling — commands arrive via subscription
 * ════════════════════════════════════════════════��══════════════════════════ */

static void OnMqttMessage(const char* topic, const char* payload, uint16_t payloadLen)
{
    FlashConfig_t* cfg = FlashConfig_Get();

    /* Only process cmd/.../set topics for this device */
    if (strstr(topic, "/set") == NULL) return;
    if (strstr(topic, cfg->deviceId) == NULL) return;

    /* Parse "run" field — same logic as old CloudSync_PollMotorCommands */
    const char* p = strstr(payload, "\"run\":");
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
    p = strstr(payload, "\"forceRun\":");
    if (p) {
        p += 11;
        MotorControl_SetForceRun(strncmp(p, "true", 4) == 0);
    }

    /* Parse "safeMode" */
    p = strstr(payload, "\"safeMode\":");
    if (p) {
        p += 11;
        MotorControl_SetSafeMode(strncmp(p, "true", 4) == 0);
    }

    /* Parse "starDelta" */
    p = strstr(payload, "\"starDelta\":");
    if (p) {
        p += 12;
        float sd = strtof(p, NULL);
        if (sd >= 1.0f && sd <= 30.0f) {
            FlashConfig_SetStarDeltaDelay(sd);
        }
    }

    /* Parse protection thresholds */
    p = strstr(payload, "\"overV\":");
    if (p) {
        p += 8;
        float v = strtof(p, NULL);
        if (v > 200.0f && v < 500.0f) {
            ProtectionConfig_t prot;
            memcpy(&prot, &cfg->protection, sizeof(prot));
            prot.overVoltage = v;

            const char* q = strstr(payload, "\"underV\":");
            if (q) { q += 9; prot.underVoltage = strtof(q, NULL); }

            q = strstr(payload, "\"overload\":");
            if (q) { q += 11; prot.overloadAmps = strtof(q, NULL); }

            q = strstr(payload, "\"dryRun\":");
            if (q) { q += 9; prot.dryRunSec = (uint16_t)strtof(q, NULL); }

            q = strstr(payload, "\"restart\":");
            if (q) { q += 10; prot.restartDelay = (uint16_t)strtof(q, NULL); }

            FlashConfig_SetProtection(&prot);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TOPIC BUILDERS — match server mqtt-handler.js topic schema
 * ═══════════════════════════════════════════════════════════════════════════ */

static void BuildTopic(const char* type, const char* deviceId, const char* suffix)
{
    FlashConfig_t* cfg = FlashConfig_Get();

    /* M2 FIX: Use "unassigned" if farmId is empty (prevents invalid MQTT topics) */
    const char* farm = (cfg->farmId[0] != '\0') ? cfg->farmId : "unassigned";

    snprintf(s_topicBuf, sizeof(s_topicBuf), "%s/%s/%s/%s",
             type, farm, deviceId, suffix);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * JSON BUILDERS (same payload format as before — transport-agnostic)
 * ═══════════════════════════════════��═══════════════════════════════════════ */

/**
 * Build motor telemetry JSON — field names match server ALLOWED_FIELDS
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
        "\"forceRun\":%s,"
        "\"safeMode\":%s,"
        "\"isRunning\":%s}",
        running ? "true" : "false",
        p->r.voltage, p->y.voltage, p->b.voltage,
        p->r.current, p->y.current, p->b.current,
        p->avgPF, p->r.powerFactor, p->y.powerFactor, p->b.powerFactor,
        p->totalPower,
        cfg->stats.totalKwh,
        cfg->stats.totalHours,
        hpStr,
        cfg->forceRun ? "true" : "false",
        cfg->safeMode ? "true" : "false",
        running ? "true" : "false"
    );
}

static uint16_t BuildWeatherJson(const WeatherPayload_t* w)
{
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"airTemperature\":%.1f,\"soilTemperature\":%.1f,\"humidity\":%.1f,"
        "\"soilMoisture\":%.1f,\"soilMoisture2\":%.1f,\"soilMoisture3\":%.1f,"
        "\"lightIntensity\":%.1f,\"leafWetness\":%u,\"leafWetness2\":%u}",
        (float)w->airTemp / 100.0f,
        (float)w->soilTemp / 100.0f,
        (float)w->humidity / 100.0f,
        (float)w->soilM1 / 100.0f,   /* M3 FIX: was soilM2 (copy-paste bug) */
        (float)w->soilM2 / 100.0f,
        (float)w->soilM3 / 100.0f,
        (float)w->lightLux * 10.0f / 655.35f,
        w->leafWet1,
        w->leafWet2
    );
}

static uint16_t BuildValveJson(const ValvePayload_t* v)
{
    return snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"isOpen\":%s,\"flow\":%.1f}",
        v->position > 50 ? "true" : "false",
        (float)v->flowRate / 10.0f
    );
}

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

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT
 * ════════════════════════════════════════════════���══════════════════════════ */

void CloudSync_Init(void)
{
    s_lastPushTick = HAL_GetTick();
    s_lastHeartbeatTick = HAL_GetTick();
    s_mqttSubscribed = false;

    /* Register incoming message callback */
    GSM_MqttSetCallback(OnMqttMessage);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MQTT CONNECTION + SUBSCRIPTION (called from Update loop)
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool EnsureMqttConnected(void)
{
    if (GSM_IsReady()) return true;

    /* Network ready but MQTT not connected — try to connect */
    if (GSM_GetState() != GSM_STATE_NETWORK_READY) return false;

    FlashConfig_t* cfg = FlashConfig_Get();

    bool ok = GSM_MqttConnect(
        cfg->mqttBrokerIp,
        cfg->mqttBrokerPort,
        cfg->deviceId,          /* clientId = device ID */
        cfg->mqttUsername,
        cfg->mqttPassword
    );

    if (ok) {
        s_mqttSubscribed = false;  /* Need to re-subscribe after connect */
    }

    return ok;
}

static void EnsureSubscribed(void)
{
    if (s_mqttSubscribed) return;

    FlashConfig_t* cfg = FlashConfig_Get();

    /* Subscribe to command topic for this device: cmd/+/{deviceId}/set
     * The + wildcard matches any userId (server publishes cmd/{userId}/{deviceId}/set) */
    snprintf(s_topicBuf, sizeof(s_topicBuf), "cmd/+/%s/set", cfg->deviceId);

    if (GSM_MqttSubscribe(s_topicBuf, 1)) {
        s_mqttSubscribed = true;
    }
}

/* ══════════════════════════════════════════���════════════════════════════════
 * PUBLIC API
 * ═════════════════���═══════════════════════════��═════════════════════════════ */

bool CloudSync_PushMotorData(const PowerSnapshot_t* power,
                              bool running, HealthStatus_t health)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    BuildMotorJson(power, running, health);
    BuildTopic("data", cfg->deviceId, "telemetry");

    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 1);
}

bool CloudSync_PushWeatherData(const WeatherPayload_t* weather,
                                const char* stationId)
{
    if (!GSM_IsReady()) return false;

    BuildWeatherJson(weather);
    BuildTopic("data", stationId, "weather");

    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 1);
}

bool CloudSync_PushValveStatus(const ValvePayload_t* valve,
                                const char* valveId)
{
    if (!GSM_IsReady()) return false;

    BuildValveJson(valve);
    BuildTopic("data", valveId, "telemetry");

    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 1);
}

bool CloudSync_PushAlert(const char* name, const char* type,
                          const char* severity, const char* action)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    BuildAlertJson(name, type, severity, action);
    BuildTopic("alert", cfg->deviceId, "warn");

    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 1);
}

bool CloudSync_LogCommand(const char* deviceType, const char* deviceId,
                           const char* action)
{
    if (!GSM_IsReady()) return false;

    snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"type\":\"%s\",\"deviceId\":\"%s\",\"action\":\"%s\"}",
        deviceType, deviceId, action);
    BuildTopic("data", deviceId, "cmdlog");

    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 0);
}

bool CloudSync_Heartbeat(void)
{
    if (!GSM_IsReady()) return false;

    FlashConfig_t* cfg = FlashConfig_Get();
    snprintf(s_jsonBuf, JSON_BUF_SIZE,
        "{\"online\":true,\"signal\":%u,\"fw\":\"%s\"}",
        GSM_GetSignalQuality(), FW_VERSION_STR);

    BuildTopic("status", cfg->deviceId, "heartbeat");
    return GSM_MqttPublish(s_topicBuf, s_jsonBuf, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN UPDATE LOOP (called from Task_CloudSync every 1000ms)
 * ═════════════════════════════════════════════════════════════════════��═════ */

void CloudSync_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* Run GSM state machine (error recovery) */
    GSM_Update();

    /* Ensure MQTT connected (auto-reconnect on failure) */
    bool online = EnsureMqttConnected();

    if (online) {
        /* Ensure subscribed to command topic */
        EnsureSubscribed();

        /* Process incoming MQTT messages (command subscription) */
        GSM_MqttProcessIncoming();

        /* M5 FIX: Drain offline queue when back online */
        if (!OfflineQueue_IsEmpty()) {
            OfflineQueue_Flush();
        }
    }

    /* Push motor telemetry every CLOUD_PUSH_PERIOD_MS */
    if ((now - s_lastPushTick) >= CLOUD_PUSH_PERIOD_MS) {
        s_lastPushTick = now;

        PowerSnapshot_t snap;
        PowerMonitor_GetSnapshot(&snap);
        bool running = MotorControl_IsRunning();
        HealthStatus_t health = PowerMonitor_AssessHealth();

        if (online) {
            CloudSync_PushMotorData(&snap, running, health);
        } else {
            /* M5 FIX: Queue telemetry for later delivery when offline */
            FlashConfig_t* cfg = FlashConfig_Get();
            BuildMotorJson(&snap, running, health);
            snprintf(s_topicBuf, sizeof(s_topicBuf), "data/%s/%s/telemetry",
                     cfg->farmId, cfg->deviceId);
            OfflineQueue_Enqueue(s_topicBuf, s_jsonBuf, 1);
        }

        /* Push fault alert if motor is in fault state */
        FaultCode_t fault = MotorControl_GetFault();
        if (fault != FAULT_NONE && fault != FAULT_MANUAL_STOP &&
            fault != FAULT_REMOTE_STOP) {
            if (online) {
                CloudSync_PushAlert(
                    MotorControl_FaultString(fault),
                    "device", "critical",
                    "Check motor and power supply"
                );
            }
            /* Alerts not queued offline — stale alerts are not useful */
        }
    }

    /* Heartbeat every HEARTBEAT_PERIOD_MS */
    if ((now - s_lastHeartbeatTick) >= HEARTBEAT_PERIOD_MS) {
        s_lastHeartbeatTick = now;
        if (online) {
            CloudSync_Heartbeat();
        }
    }
}
