/**
 * @file power_monitor.c
 * @brief 3-Phase power monitoring with REAL power factor calculation
 *
 * Uses simultaneous V and I sampling to compute real power P = avg(v*i),
 * then derives PF = P / (Vrms * Irms). This gives true power factor
 * unlike the prototype which hardcoded PF=1.0.
 */

#include "power_monitor.h"
#include "watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

/* DWT availability flag set by main.c DWT_Init() */
extern bool g_dwtAvailable;

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static ADC_HandleTypeDef hadc1;
static PowerSnapshot_t g_snapshot;
static SemaphoreHandle_t g_snapshotMutex;
static StaticSemaphore_t g_snapshotMutexBuf;

/* Raw ADC sample arrays */
static uint16_t g_vSamples[3][ADC_SAMPLES_PER_CYCLE];  // V phases R,Y,B
static uint16_t g_iSamples[3][ADC_SAMPLES_PER_CYCLE];  // I phases R,Y,B

/* Calibration factors */
static float g_vCal = V_CT_RATIO / (ADC_RESOLUTION / 2.0f);
static float g_iCal = I_CT_RATIO / (ADC_RESOLUTION / 2.0f);

/* ═══════════════════════════════════════════════════════════════════════════
 * ADC INITIALIZATION
 * ═══════════════════════════════════════════════════════════════════════════ */

void PowerMonitor_Init(void)
{
    /* Create snapshot mutex */
    g_snapshotMutex = xSemaphoreCreateMutexStatic(&g_snapshotMutexBuf);

    memset(&g_snapshot, 0, sizeof(PowerSnapshot_t));

    /* Enable ADC1 clock */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure analog pins */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* PA0, PA1, PA2, PA3 = Voltage R,Y,B + Current R */
    gpio.Pin = PIN_V_CT_R_PIN | PIN_V_CT_Y_PIN | PIN_V_CT_B_PIN | PIN_I_CT_R_PIN;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB0, PB1 = Current Y, B */
    gpio.Pin = PIN_I_CT_Y_PIN | PIN_I_CT_B_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Configure ADC */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;  // Single channel per call
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    /* Calibrate ADC (GD32 needs short settling after calibration) */
    HAL_ADCEx_Calibration_Start(&hadc1);
    for (volatile uint32_t d = 0; d < 200; d++) { __NOP(); }  /* ~10us settle */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ADC SINGLE-CHANNEL READ
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t ReadADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;  // Fast for 50Hz sampling
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 2);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return val;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PHASE CALCULATION
 *
 * For each phase:
 *   Vrms = sqrt(mean(v^2))
 *   Irms = sqrt(mean(i^2))
 *   P_real = mean(v * i)          <-- TRUE real power
 *   PF = P_real / (Vrms * Irms)   <-- TRUE power factor
 * ═══════════════════════════════════════════════════════════════════════════ */

static void CalcPhase(const uint16_t* vRaw, const uint16_t* iRaw,
                      uint16_t nSamples, PhaseReading_t* out)
{
    float sumV2 = 0.0f;
    float sumI2 = 0.0f;
    float sumVI = 0.0f;

    /* DC offset is at ADC midpoint (Vref/2) */
    const float vOffset = (float)(ADC_RESOLUTION / 2);
    const float iOffset = (float)(ADC_RESOLUTION / 2);

    for (uint16_t i = 0; i < nSamples; i++) {
        /* Remove DC offset and apply calibration */
        float v = ((float)vRaw[i] - vOffset) * g_vCal;
        float iVal = ((float)iRaw[i] - iOffset) * g_iCal;

        sumV2 += v * v;
        sumI2 += iVal * iVal;
        sumVI += v * iVal;  /* Instantaneous power product */
    }

    float n = (float)nSamples;

    /* RMS values */
    out->voltage = sqrtf(sumV2 / n);
    out->current = sqrtf(sumI2 / n);

    /* Real power (average of instantaneous power) */
    out->power = sumVI / n;

    /* Apparent power */
    out->apparentPower = out->voltage * out->current;

    /* Power factor = P / S (clamped to 0-1) */
    if (out->apparentPower > 0.1f) {
        out->powerFactor = fabsf(out->power) / out->apparentPower;
        if (out->powerFactor > 1.0f) out->powerFactor = 1.0f;
    } else {
        out->powerFactor = 0.0f;  // No load
    }

    /* Make power positive (we don't measure direction) */
    out->power = fabsf(out->power);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN SAMPLING FUNCTION
 * ═══════════════════════════════════════════════════════════════════════════ */

void PowerMonitor_Sample(void)
{
    /* Sample all 6 channels interleaved for timing accuracy.
     * For 50Hz at 150 samples = 3 full cycles = 60ms total.
     * Sample interval = 60ms / 150 = 400us per sample set.
     */
    const uint32_t channels_v[3] = { PIN_V_CT_R_CH, PIN_V_CT_Y_CH, PIN_V_CT_B_CH };
    const uint32_t channels_i[3] = { PIN_I_CT_R_CH, PIN_I_CT_Y_CH, PIN_I_CT_B_CH };

    for (uint16_t s = 0; s < ADC_SAMPLES_PER_CYCLE; s++) {
        /* Sample all 6 channels as quickly as possible */
        for (uint8_t ph = 0; ph < 3; ph++) {
            g_vSamples[ph][s] = ReadADC(channels_v[ph]);
            g_iSamples[ph][s] = ReadADC(channels_i[ph]);
        }

        /* Wait for next sample point (~400us interval for 50Hz x 3 cycles).
         * GD32/CKS32 clones may have DWT CYCCNT locked (reads 0 always).
         * Fall back to calibrated NOP loop if DWT unavailable. */
        if (g_dwtAvailable) {
            /* Precise: 72MHz * 400us = 28800 cycles (or 64MHz * 400us = 25600) */
            uint32_t startTick = DWT->CYCCNT;
            uint32_t target = (SystemCoreClock / 2500);  /* SYSCLK / 2500 = 400us worth of cycles */
            while ((DWT->CYCCNT - startTick) < target) {
                __NOP();
            }
        } else {
            /* Fallback: ~400us NOP delay (calibrated for 64-72MHz) */
            for (volatile uint32_t d = 0; d < 2400; d++) { __NOP(); }
        }

        /* C3 FIX: Feed watchdog mid-sampling and yield to same-priority tasks
         * every 50 samples (~20ms) to prevent starving the motor control task */
        if ((s % 50) == 49) {
            Watchdog_Feed();
            taskYIELD();
        }
    }

    /* Calculate per-phase readings */
    PowerSnapshot_t snap;
    CalcPhase(g_vSamples[0], g_iSamples[0], ADC_SAMPLES_PER_CYCLE, &snap.r);
    CalcPhase(g_vSamples[1], g_iSamples[1], ADC_SAMPLES_PER_CYCLE, &snap.y);
    CalcPhase(g_vSamples[2], g_iSamples[2], ADC_SAMPLES_PER_CYCLE, &snap.b);

    /* Derived values */
    snap.totalPower = (snap.r.power + snap.y.power + snap.b.power) / 1000.0f;  // kW
    snap.avgVoltage = (snap.r.voltage + snap.y.voltage + snap.b.voltage) / 3.0f;
    snap.avgCurrent = (snap.r.current + snap.y.current + snap.b.current) / 3.0f;

    /* Average PF (weighted by current) */
    float totalCurrent = snap.r.current + snap.y.current + snap.b.current;
    if (totalCurrent > 0.1f) {
        snap.avgPF = (snap.r.powerFactor * snap.r.current +
                      snap.y.powerFactor * snap.y.current +
                      snap.b.powerFactor * snap.b.current) / totalCurrent;
    } else {
        snap.avgPF = 0.0f;
    }

    /* Current imbalance: max deviation from average as percentage */
    if (snap.avgCurrent > 0.1f) {
        float maxDev = 0.0f;
        float devR = fabsf(snap.r.current - snap.avgCurrent);
        float devY = fabsf(snap.y.current - snap.avgCurrent);
        float devB = fabsf(snap.b.current - snap.avgCurrent);
        if (devR > maxDev) maxDev = devR;
        if (devY > maxDev) maxDev = devY;
        if (devB > maxDev) maxDev = devB;
        snap.currentImbalance = (maxDev / snap.avgCurrent) * 100.0f;
    } else {
        snap.currentImbalance = 0.0f;
    }

    snap.timestamp = HAL_GetTick();

    /* Update shared snapshot (thread-safe) */
    if (xSemaphoreTake(g_snapshotMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&g_snapshot, &snap, sizeof(PowerSnapshot_t));
        xSemaphoreGive(g_snapshotMutex);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GETTERS
 * ═══════════════════════════════════════════════════════════════════════════ */

void PowerMonitor_GetSnapshot(PowerSnapshot_t* snapshot)
{
    if (xSemaphoreTake(g_snapshotMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(snapshot, &g_snapshot, sizeof(PowerSnapshot_t));
        xSemaphoreGive(g_snapshotMutex);
    }
}

float PowerMonitor_GetAvgVoltage(void) { return g_snapshot.avgVoltage; }
float PowerMonitor_GetAvgCurrent(void) { return g_snapshot.avgCurrent; }
float PowerMonitor_GetAvgPF(void)      { return g_snapshot.avgPF; }
float PowerMonitor_GetTotalPowerKW(void) { return g_snapshot.totalPower; }

/* ═══════════════════════════════════════════════════════════════════════════
 * PROTECTION CHECKS
 * ═══════════════════════════════════════════════════════════════════════════ */

FaultCode_t PowerMonitor_CheckVoltage(float overV, float underV)
{
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Check each phase */
    float voltages[3] = { snap.r.voltage, snap.y.voltage, snap.b.voltage };

    for (int i = 0; i < 3; i++) {
        /* Phase loss: voltage below 50V */
        if (voltages[i] < 50.0f && snap.avgVoltage > 100.0f) {
            return FAULT_PHASE_LOSS;
        }
        /* Overvoltage */
        if (voltages[i] > overV) {
            return FAULT_OVERVOLTAGE;
        }
        /* Undervoltage (only check if voltage is present) */
        if (voltages[i] < underV && voltages[i] > 50.0f) {
            return FAULT_UNDERVOLTAGE;
        }
    }

    return FAULT_NONE;
}

FaultCode_t PowerMonitor_CheckCurrent(float overloadAmps, float dryRunAmps)
{
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Overload check */
    if (snap.r.current > overloadAmps ||
        snap.y.current > overloadAmps ||
        snap.b.current > overloadAmps) {
        return FAULT_OVERLOAD;
    }

    /* Current imbalance check */
    if (snap.currentImbalance > HEALTH_IMBALANCE_CRIT) {
        return FAULT_CURRENT_IMBALANCE;
    }

    /* Dry run: all currents below threshold */
    if (snap.r.current < dryRunAmps &&
        snap.y.current < dryRunAmps &&
        snap.b.current < dryRunAmps) {
        return FAULT_DRY_RUN;
    }

    return FAULT_NONE;
}

HealthStatus_t PowerMonitor_AssessHealth(void)
{
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Critical conditions */
    if (snap.avgPF < HEALTH_PF_CRITICAL ||
        snap.currentImbalance > HEALTH_IMBALANCE_CRIT) {
        return HEALTH_CRITICAL;
    }

    /* Warning conditions */
    if (snap.avgPF < HEALTH_PF_WARNING ||
        snap.currentImbalance > HEALTH_IMBALANCE_WARN) {
        return HEALTH_WARNING;
    }

    return HEALTH_GOOD;
}

float PowerMonitor_AccumulateEnergy(uint32_t deltaMs)
{
    float kw = g_snapshot.totalPower;
    float hours = (float)deltaMs / 3600000.0f;
    return kw * hours;  // kWh
}
