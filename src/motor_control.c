/**
 * @file motor_control.c
 * @brief Star-delta motor FSM with auto-calibration and multi-mode protection
 */

#include "motor_control.h"
#include "power_monitor.h"
#include "flash_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static MotorState_t g_state = MOTOR_IDLE;
static FaultCode_t  g_lastFault = FAULT_NONE;
static MotorStats_t g_stats;

/* Timing */
static uint32_t g_stateEnterTick = 0;   // Tick when current state entered
static uint32_t g_lastEnergyTick = 0;   // For kWh accumulation
static uint32_t g_dryRunStartTick = 0;  // When dry-run condition first detected

/* Flags */
static bool g_startRequested = false;
static bool g_stopRequested = false;
static bool g_calibrationPending = false;
static bool g_calibrationActive = false;
static uint32_t g_calibrationStartTick = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * RELAY CONTROL (Active HIGH)
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline void RelayMain(bool on) {
    HAL_GPIO_WritePin(PIN_RELAY_MAIN_PORT, PIN_RELAY_MAIN_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void RelayStar(bool on) {
    HAL_GPIO_WritePin(PIN_RELAY_STAR_PORT, PIN_RELAY_STAR_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void RelayDelta(bool on) {
    HAL_GPIO_WritePin(PIN_RELAY_DELTA_PORT, PIN_RELAY_DELTA_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AllRelaysOff(void) {
    RelayMain(false);
    RelayStar(false);
    RelayDelta(false);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE TRANSITION
 * ═══════════════════════════════════════════════════════════════════════════ */

static void EnterState(MotorState_t newState)
{
    g_state = newState;
    g_stateEnterTick = HAL_GetTick();
}

static uint32_t TimeInState(void)
{
    return HAL_GetTick() - g_stateEnterTick;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * AUTO-CALIBRATION
 *
 * After star-delta transition, wait 5 seconds for current to stabilize,
 * then measure steady-state delta current to compute:
 * - HP = (sqrt(3) * V_avg * I_avg * PF) / 746
 * - Auto protection thresholds
 * - Auto star-delta timing
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CALIBRATION_SETTLE_MS   5000    // Wait 5s after delta for stabilization

static void PerformCalibration(void)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Compute HP */
    float sqrt3 = 1.732f;
    float watts = sqrt3 * snap.avgVoltage * snap.avgCurrent * snap.avgPF;
    float hp = watts / 746.0f;

    /* Store calibration */
    CalibrationData_t cal;
    cal.ratedHP      = hp;
    cal.ratedVoltage  = snap.avgVoltage;
    cal.ratedCurrent  = snap.avgCurrent;
    cal.phases        = 3;
    cal.calibratedAt  = HAL_GetTick() / 1000;  // Approximate (real timestamp from GSM later)
    cal.isCalibrated  = true;
    FlashConfig_SetCalibration(&cal);

    /* Auto-set protection thresholds */
    ProtectionConfig_t prot;
    prot.overVoltage  = DEFAULT_OVER_VOLTAGE;
    prot.underVoltage = DEFAULT_UNDER_VOLTAGE;
    prot.overloadAmps = snap.avgCurrent * OVERLOAD_MULTIPLIER;
    prot.dryRunSec    = DEFAULT_DRY_RUN_SEC;
    prot.restartDelay = DEFAULT_RESTART_DELAY;
    FlashConfig_SetProtection(&prot);

    /* Auto star-delta timing: base + per-HP scaling */
    float sdDelay = SD_DELAY_BASE + (hp * SD_DELAY_PER_HP);
    if (sdDelay > SD_DELAY_MAX) sdDelay = SD_DELAY_MAX;
    FlashConfig_SetStarDeltaDelay(sdDelay);

    /* Save to flash */
    FlashConfig_Save();

    g_calibrationActive = false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PROTECTION LOGIC
 * ═══════════════════════════════════════════════════════════════════════════ */

static FaultCode_t RunProtectionChecks(void)
{
    FlashConfig_t* cfg = FlashConfig_Get();

    /* Voltage protection is ALWAYS active (even in force run mode) */
    FaultCode_t vFault = PowerMonitor_CheckVoltage(
        cfg->protection.overVoltage,
        cfg->protection.underVoltage
    );
    if (vFault != FAULT_NONE) return vFault;

    /* In force run mode, skip current-based protection */
    if (cfg->forceRun) return FAULT_NONE;

    /* Determine overload threshold based on mode */
    float overloadAmps = cfg->protection.overloadAmps;
    if (cfg->safeMode) {
        /* Safe mode: tighter threshold (110% instead of 130%) */
        if (cfg->calibration.isCalibrated) {
            overloadAmps = cfg->calibration.ratedCurrent * SAFE_MODE_MULTIPLIER;
        } else {
            overloadAmps *= (SAFE_MODE_MULTIPLIER / OVERLOAD_MULTIPLIER);
        }
    }

    /* Dry run threshold */
    float dryRunAmps = 0.5f;  // Default minimum
    if (cfg->calibration.isCalibrated) {
        dryRunAmps = cfg->calibration.ratedCurrent * DRY_RUN_MULTIPLIER;
    }

    FaultCode_t iFault = PowerMonitor_CheckCurrent(overloadAmps, dryRunAmps);

    /* Dry run needs sustained condition (not just momentary) */
    if (iFault == FAULT_DRY_RUN) {
        if (g_dryRunStartTick == 0) {
            g_dryRunStartTick = HAL_GetTick();  // Start counting
        }
        if ((HAL_GetTick() - g_dryRunStartTick) < (cfg->protection.dryRunSec * 1000)) {
            return FAULT_NONE;  // Not yet timed out
        }
        return FAULT_DRY_RUN;  // Timed out!
    } else {
        g_dryRunStartTick = 0;  // Reset dry-run timer
    }

    return iFault;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT
 * ═══════════════════════════════════════════════════════════════════════════ */

void MotorControl_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Relay outputs */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = PIN_RELAY_DELTA_PIN;
    HAL_GPIO_Init(PIN_RELAY_DELTA_PORT, &gpio);

    gpio.Pin = PIN_RELAY_STAR_PIN;
    HAL_GPIO_Init(PIN_RELAY_STAR_PORT, &gpio);

    gpio.Pin = PIN_RELAY_MAIN_PIN;
    HAL_GPIO_Init(PIN_RELAY_MAIN_PORT, &gpio);

    AllRelaysOff();

    /* Button inputs with pull-up and EXTI */
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;

    gpio.Pin = PIN_BTN_START_PIN;
    HAL_GPIO_Init(PIN_BTN_START_PORT, &gpio);

    gpio.Pin = PIN_BTN_STOP_PIN;
    HAL_GPIO_Init(PIN_BTN_STOP_PORT, &gpio);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);  // Below FreeRTOS max syscall prio
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* Load stats from flash config */
    FlashConfig_t* cfg = FlashConfig_Get();
    memcpy(&g_stats, &cfg->stats, sizeof(MotorStats_t));

    /* If not calibrated, request calibration on first start */
    if (!cfg->calibration.isCalibrated) {
        g_calibrationPending = true;
    }

    EnterState(MOTOR_IDLE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MAIN FSM UPDATE
 * ═══════════════════════════════════════════════════════════════════════════ */

void MotorControl_Update(void)
{
    FlashConfig_t* cfg = FlashConfig_Get();

    /* Get star-delta delay from config (stored as tenths of seconds) */
    uint32_t starDeltaMs = (uint32_t)cfg->starDeltaDelay * 100;

    switch (g_state) {

    /* ── IDLE ── */
    case MOTOR_IDLE:
        if (g_startRequested) {
            g_startRequested = false;

            /* Pre-start voltage check */
            FaultCode_t vFault = PowerMonitor_CheckVoltage(
                cfg->protection.overVoltage,
                cfg->protection.underVoltage
            );
            if (vFault != FAULT_NONE && !cfg->forceRun) {
                g_lastFault = vFault;
                EnterState(MOTOR_FAULT);
                break;
            }

            /* Begin star-delta sequence */
            if (g_calibrationPending) {
                g_calibrationActive = true;
                g_calibrationPending = false;
            }

            /* Energize: MAIN + STAR (never DELTA during star) */
            RelayDelta(false);
            RelayStar(true);
            RelayMain(true);

            g_stats.startCount++;
            g_stats.lastStartTs = HAL_GetTick() / 1000;
            g_lastEnergyTick = HAL_GetTick();
            g_dryRunStartTick = 0;

            EnterState(MOTOR_STAR);
        }
        break;

    /* ── STAR (reduced voltage starting) ── */
    case MOTOR_STAR:
        if (g_stopRequested) {
            g_stopRequested = false;
            AllRelaysOff();
            g_lastFault = FAULT_MANUAL_STOP;
            EnterState(MOTOR_IDLE);
            break;
        }

        if (TimeInState() >= starDeltaMs) {
            /* Transition: disconnect STAR first (dead time) */
            RelayStar(false);
            EnterState(MOTOR_STAR_TO_DELTA);
        }
        break;

    /* ── DEAD TIME (star off, delta not yet on) ── */
    case MOTOR_STAR_TO_DELTA:
        if (TimeInState() >= DEAD_TIME_MS) {
            /* SAFETY: verify star is definitely off before engaging delta */
            RelayStar(false);  // Redundant safety

            /* Engage delta */
            RelayDelta(true);

            if (g_calibrationActive) {
                g_calibrationStartTick = HAL_GetTick();
            }

            EnterState(MOTOR_DELTA);
        }
        break;

    /* ── DELTA (full voltage running) ── */
    case MOTOR_DELTA:
        if (g_stopRequested) {
            g_stopRequested = false;
            AllRelaysOff();
            g_lastFault = FAULT_MANUAL_STOP;
            g_stats.lastStopTs = HAL_GetTick() / 1000;
            EnterState(MOTOR_IDLE);
            break;
        }

        /* Auto-calibration: after settle time, measure and compute */
        if (g_calibrationActive &&
            (HAL_GetTick() - g_calibrationStartTick) >= CALIBRATION_SETTLE_MS) {
            PerformCalibration();
        }

        /* Run protection checks */
        FaultCode_t fault = RunProtectionChecks();
        if (fault != FAULT_NONE) {
            AllRelaysOff();
            g_lastFault = fault;
            g_stats.lastStopTs = HAL_GetTick() / 1000;
            EnterState(MOTOR_FAULT);
            break;
        }

        /* Accumulate energy and runtime */
        {
            uint32_t now = HAL_GetTick();
            uint32_t deltaMs = now - g_lastEnergyTick;
            if (deltaMs >= 1000) {  // Update every second
                g_stats.totalKwh += PowerMonitor_AccumulateEnergy(deltaMs);
                g_stats.totalHours += (float)deltaMs / 3600000.0f;
                g_lastEnergyTick = now;
            }
        }
        break;

    /* ── STOPPING ── */
    case MOTOR_STOPPING:
        AllRelaysOff();
        g_stats.lastStopTs = HAL_GetTick() / 1000;
        EnterState(MOTOR_IDLE);
        break;

    /* ── FAULT ── */
    case MOTOR_FAULT:
        AllRelaysOff();

        /* Auto-restart after delay (if not force-stopped) */
        if (g_lastFault != FAULT_MANUAL_STOP &&
            g_lastFault != FAULT_REMOTE_STOP) {
            if (TimeInState() >= (cfg->protection.restartDelay * 1000)) {
                EnterState(MOTOR_RESTART_WAIT);
            }
        }

        /* Manual reset */
        if (g_startRequested) {
            g_startRequested = false;
            g_lastFault = FAULT_NONE;
            EnterState(MOTOR_IDLE);
        }
        break;

    /* ── RESTART WAIT ── */
    case MOTOR_RESTART_WAIT:
        g_lastFault = FAULT_NONE;
        g_startRequested = true;  // Auto-trigger start
        EnterState(MOTOR_IDLE);
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

void MotorControl_Start(void)
{
    if (g_state == MOTOR_IDLE || g_state == MOTOR_FAULT) {
        g_startRequested = true;
    }
}

void MotorControl_Stop(FaultCode_t fault)
{
    if (fault == FAULT_NONE) {
        fault = FAULT_MANUAL_STOP;
    }
    g_lastFault = fault;
    g_stopRequested = true;
}

MotorState_t MotorControl_GetState(void)   { return g_state; }
bool MotorControl_IsRunning(void)          { return g_state == MOTOR_DELTA; }
FaultCode_t MotorControl_GetFault(void)    { return g_lastFault; }

const char* MotorControl_FaultString(FaultCode_t fault)
{
    switch (fault) {
        case FAULT_NONE:              return "OK";
        case FAULT_OVERVOLTAGE:       return "Overvoltage";
        case FAULT_UNDERVOLTAGE:      return "Undervoltage";
        case FAULT_OVERLOAD:          return "Overload";
        case FAULT_DRY_RUN:           return "Dry Run";
        case FAULT_PHASE_LOSS:        return "Phase Loss";
        case FAULT_CURRENT_IMBALANCE: return "I Imbalance";
        case FAULT_MANUAL_STOP:       return "Manual Stop";
        case FAULT_REMOTE_STOP:       return "Remote Stop";
        default:                      return "Unknown";
    }
}

void MotorControl_SetSafeMode(bool enable)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    cfg->safeMode = enable;
}

void MotorControl_SetForceRun(bool enable)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    cfg->forceRun = enable;
}

bool MotorControl_GetSafeMode(void)  { return FlashConfig_Get()->safeMode; }
bool MotorControl_GetForceRun(void)  { return FlashConfig_Get()->forceRun; }

void MotorControl_RequestCalibration(void) { g_calibrationPending = true; }
bool MotorControl_IsCalibrationPending(void) { return g_calibrationPending; }

void MotorControl_GetStats(MotorStats_t* stats)
{
    memcpy(stats, &g_stats, sizeof(MotorStats_t));
}

void MotorControl_ButtonPress(bool isStart)
{
    static uint32_t lastPress = 0;
    uint32_t now = HAL_GetTick();

    /* Debounce */
    if ((now - lastPress) < BUTTON_DEBOUNCE_MS) return;
    lastPress = now;

    if (isStart) {
        MotorControl_Start();
    } else {
        MotorControl_Stop(FAULT_MANUAL_STOP);
    }
}
