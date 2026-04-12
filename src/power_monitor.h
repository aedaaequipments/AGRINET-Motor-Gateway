/**
 * @file power_monitor.h
 * @brief 3-Phase power monitoring with real power factor calculation
 *
 * Samples 6 ADC channels (3 voltage CTs, 3 current CTs) and computes:
 * - RMS voltage per phase
 * - RMS current per phase
 * - Real power per phase (from V*I product)
 * - Power factor per phase (from P / (Vrms * Irms))
 * - Total power, average PF, current imbalance
 */

#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize ADC and DMA for power monitoring
 */
void PowerMonitor_Init(void);

/**
 * @brief Sample all 6 channels and compute power readings
 *
 * Call this periodically (every POWER_SAMPLE_PERIOD_MS).
 * Samples ADC_SAMPLES_PER_CYCLE points across ~3 mains cycles.
 */
void PowerMonitor_Sample(void);

/**
 * @brief Get latest power snapshot (thread-safe copy)
 * @param snapshot Destination for copy
 */
void PowerMonitor_GetSnapshot(PowerSnapshot_t* snapshot);

/**
 * @brief Get average voltage across all phases
 * @return Average RMS voltage
 */
float PowerMonitor_GetAvgVoltage(void);

/**
 * @brief Get average current across all phases
 * @return Average RMS current
 */
float PowerMonitor_GetAvgCurrent(void);

/**
 * @brief Get average power factor
 * @return Average PF (0-1)
 */
float PowerMonitor_GetAvgPF(void);

/**
 * @brief Get total power in kW
 * @return Total real power
 */
float PowerMonitor_GetTotalPowerKW(void);

/**
 * @brief Check if voltage is within acceptable range
 * @param overV Overvoltage threshold
 * @param underV Undervoltage threshold
 * @return FAULT_NONE, FAULT_OVERVOLTAGE, FAULT_UNDERVOLTAGE, or FAULT_PHASE_LOSS
 */
FaultCode_t PowerMonitor_CheckVoltage(float overV, float underV);

/**
 * @brief Check if current indicates overload or dry run
 * @param overloadAmps Overload threshold
 * @param dryRunAmps Dry run threshold (current below this = no load)
 * @return FAULT_NONE, FAULT_OVERLOAD, FAULT_DRY_RUN, or FAULT_CURRENT_IMBALANCE
 */
FaultCode_t PowerMonitor_CheckCurrent(float overloadAmps, float dryRunAmps);

/**
 * @brief Assess motor health based on PF and current balance
 * @return HEALTH_GOOD, HEALTH_WARNING, or HEALTH_CRITICAL
 */
HealthStatus_t PowerMonitor_AssessHealth(void);

/**
 * @brief Accumulate energy (call periodically while motor is running)
 * @param deltaMs Milliseconds since last call
 * @return Energy consumed in this interval (kWh)
 */
float PowerMonitor_AccumulateEnergy(uint32_t deltaMs);

/**
 * @brief Load V/I calibration factors from flash config
 */
void PowerMonitor_LoadCalibration(void);

/**
 * @brief Set voltage CT ratio (saved to flash config, takes effect immediately)
 * @param vCtRatio e.g. 230.0 for ZMPT101B with default burden
 */
void PowerMonitor_SetVCal(float vCtRatio);

/**
 * @brief Set current CT ratio (saved to flash config, takes effect immediately)
 * @param iCtRatio e.g. 30.0 for SCT-013-030
 */
void PowerMonitor_SetICal(float iCtRatio);

/**
 * @brief Get raw ADC readings for debug/calibration
 * @param rawV[3] Raw 12-bit ADC values for voltage phases R,Y,B
 * @param rawI[3] Raw 12-bit ADC values for current phases R,Y,B
 */
void PowerMonitor_GetRawADC(uint16_t rawV[3], uint16_t rawI[3]);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MONITOR_H */
