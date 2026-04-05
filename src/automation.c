/**
 * @file automation.c
 * @brief Local Automation Engine - irrigation decisions + risk assessment
 *
 * Runs locally on the master gateway so irrigation works even without
 * cloud connectivity. Thresholds are synced from Firebase when available.
 */

#include "automation.h"
#include "lora_manager.h"
#include "motor_control.h"
#include "flash_config.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static AutoIrrigationConfig_t s_config;
static RiskAssessment_t s_risk;
static bool s_valveRequest = false;
static bool s_motorRequest = false;
static bool s_configured = false;

/* Hysteresis: prevent rapid toggling */
static bool s_irrigating = false;
static uint32_t s_lastToggleTick = 0;
#define MIN_TOGGLE_INTERVAL_MS  60000   // 1 minute minimum between toggles

/* ═══════════════════════════════════════════════════════════════════════════
 * DEFAULTS (used until app sends config)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ApplyDefaults(void)
{
    s_config.L1.min = 25.0f;  s_config.L1.max = 55.0f;  s_config.L1.enabled = true;
    s_config.L2.min = 35.0f;  s_config.L2.max = 70.0f;  s_config.L2.enabled = true;
    s_config.L3.min = 30.0f;  s_config.L3.max = 60.0f;  s_config.L3.enabled = false;
    s_config.envAdjust = true;
    s_configured = true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RISK CALCULATION
 *
 * Based on Sasyamithra App v6.2 logic:
 * - Humidity >85% = +30 pest risk
 * - Leaf wetness >60% = +35 pest risk
 * - Temp 20-30°C = +20 pest risk
 * - Air temp 20-28°C AND humidity >90% = high disease risk
 * ═══════════════════════════════════════════════════════════════════════════ */

static void CalcRisk(const WeatherPayload_t* w)
{
    float airT    = (float)w->airTemp / 100.0f;
    float humid   = (float)w->humidity / 100.0f;
    float leafWet = (float)w->leafWet1;

    /* Pest risk */
    uint8_t pest = 0;
    if (humid > 85.0f) pest += 30;
    if (leafWet > 60.0f) pest += 35;
    if (airT >= 20.0f && airT <= 30.0f) pest += 20;
    /* Additional factors */
    if ((float)w->soilM2 / 100.0f > 80.0f) pest += 10;  // Waterlogged
    if (pest > 100) pest = 100;
    s_risk.pestRisk = pest;

    /* Disease risk */
    uint8_t disease = 0;
    if (airT >= 20.0f && airT <= 28.0f && humid > 90.0f) {
        disease = 80;
    } else if (airT >= 18.0f && airT <= 30.0f && humid > 80.0f) {
        disease = 50;
    } else if (humid > 75.0f) {
        disease = 25;
    }
    if (leafWet > 70.0f) disease += 15;
    if (disease > 100) disease = 100;
    s_risk.diseaseRisk = disease;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * IRRIGATION DECISION
 *
 * Logic:
 * 1. L2 (root zone) is the PRIMARY decision point
 * 2. If L2 moisture < L2.min -> START irrigation
 * 3. If L2 moisture > L2.max -> STOP irrigation
 * 4. L1 and L3 are secondary (if enabled, used for confirmation)
 * 5. Environmental adjustment: reduce irrigation if rain detected
 * ═══════════════════════════════════════════════════════════════════════════ */

static void EvalIrrigation(const WeatherPayload_t* w)
{
    if (!s_configured) return;

    float soilM1 = (float)w->soilM1 / 100.0f;
    float soilM2 = (float)w->soilM2 / 100.0f;
    float soilM3 = (float)w->soilM3 / 100.0f;
    float rain   = (float)w->rainMm / 10.0f;

    /* Primary decision: L2 root zone */
    bool needWater = false;
    bool stopWater = false;

    if (s_config.L2.enabled) {
        if (soilM2 < s_config.L2.min) needWater = true;
        if (soilM2 > s_config.L2.max) stopWater = true;
    }

    /* Secondary confirmation from L1 if enabled */
    if (s_config.L1.enabled && !needWater) {
        if (soilM1 < s_config.L1.min) needWater = true;
    }

    /* Environmental adjustment */
    if (s_config.envAdjust) {
        /* If it's raining, don't irrigate */
        if (rain > 2.0f) {
            needWater = false;
            stopWater = true;
        }
        /* If humidity very high, reduce irrigation */
        float humid = (float)w->humidity / 100.0f;
        if (humid > 90.0f && needWater) {
            /* Only irrigate if soil is critically dry */
            if (soilM2 > (s_config.L2.min - 5.0f)) {
                needWater = false;  // Not critical, skip
            }
        }
    }

    /* Excess water detection */
    s_risk.excessWater = (soilM2 > 85.0f) || (soilM1 > 90.0f);

    /* Apply hysteresis */
    uint32_t now = HAL_GetTick();
    if ((now - s_lastToggleTick) < MIN_TOGGLE_INTERVAL_MS) {
        return;  // Too soon to toggle
    }

    /* State transition */
    if (!s_irrigating && needWater) {
        s_irrigating = true;
        s_lastToggleTick = now;
    } else if (s_irrigating && stopWater) {
        s_irrigating = false;
        s_lastToggleTick = now;
    }

    s_risk.irrigationNeeded = needWater;
    s_valveRequest = s_irrigating;
    s_motorRequest = s_irrigating;  // Motor must run when irrigating
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

void Automation_Init(void)
{
    memset(&s_risk, 0, sizeof(s_risk));
    s_valveRequest = false;
    s_motorRequest = false;
    s_irrigating = false;
    ApplyDefaults();
}

void Automation_SetThresholds(const AutoIrrigationConfig_t* config)
{
    memcpy(&s_config, config, sizeof(AutoIrrigationConfig_t));
    s_configured = true;
}

bool Automation_Evaluate(const WeatherPayload_t* weather)
{
    if (!weather) return false;

    bool prevValve = s_valveRequest;

    CalcRisk(weather);
    EvalIrrigation(weather);

    /* Check if motor mode is AUTO */
    FlashConfig_t* cfg = FlashConfig_Get();
    if (cfg->mode != MODE_AUTO) {
        s_valveRequest = false;
        s_motorRequest = false;
        return false;
    }

    /* Return true if valve state changed */
    return (s_valveRequest != prevValve);
}

void Automation_GetRisk(RiskAssessment_t* risk)
{
    memcpy(risk, &s_risk, sizeof(RiskAssessment_t));
}

bool Automation_ShouldValveOpen(void)  { return s_valveRequest; }
bool Automation_ShouldMotorRun(void)   { return s_motorRequest; }
