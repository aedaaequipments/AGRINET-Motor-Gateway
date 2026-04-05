/**
 * @file automation.h
 * @brief Local Automation Engine - soil moisture-based irrigation + pest/disease risk
 *
 * Uses app-configured thresholds from Firebase:
 * - L1 (surface 0-10cm), L2 (root zone 15-20cm), L3 (deep 25-30cm)
 * - Auto-open valve if L2 < min, close if L2 > max
 * - Pest risk: humidity >85% (+30), leaf wetness >60% (+35), temp 20-30°C (+20)
 * - Disease risk: temp 20-28°C AND humidity >90%
 */

#ifndef AUTOMATION_H
#define AUTOMATION_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_AUTO_RULES      8

/**
 * @brief Irrigation thresholds for one soil level
 */
typedef struct {
    float min;          // Open valve if moisture < min
    float max;          // Close valve if moisture > max
    bool  enabled;
} MoistureThreshold_t;

/**
 * @brief Auto-irrigation config (from app valve settings)
 */
typedef struct {
    MoistureThreshold_t L1;     // Surface 0-10cm
    MoistureThreshold_t L2;     // Root zone 15-20cm (PRIMARY)
    MoistureThreshold_t L3;     // Deep 25-30cm
    bool envAdjust;             // Environmental adjustment enabled
} AutoIrrigationConfig_t;

/**
 * @brief Risk assessment result
 */
typedef struct {
    uint8_t pestRisk;       // 0-100
    uint8_t diseaseRisk;    // 0-100
    bool    irrigationNeeded;
    bool    excessWater;
} RiskAssessment_t;

/**
 * @brief Initialize automation engine
 */
void Automation_Init(void);

/**
 * @brief Set irrigation thresholds (from Firebase valve config)
 * @param config Auto-irrigation configuration
 */
void Automation_SetThresholds(const AutoIrrigationConfig_t* config);

/**
 * @brief Evaluate automation rules against current sensor data
 * @param weather Latest weather station data
 * @return true if any action was taken
 */
bool Automation_Evaluate(const WeatherPayload_t* weather);

/**
 * @brief Get latest risk assessment
 * @param risk Output risk data
 */
void Automation_GetRisk(RiskAssessment_t* risk);

/**
 * @brief Check if valve should be open based on automation
 * @return true if automation wants valve open
 */
bool Automation_ShouldValveOpen(void);

/**
 * @brief Check if motor should run based on automation
 * (Motor runs when any valve needs water)
 */
bool Automation_ShouldMotorRun(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOMATION_H */
