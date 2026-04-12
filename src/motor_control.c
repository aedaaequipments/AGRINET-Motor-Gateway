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

/* C4 FIX: Flags written from ISR/multiple tasks — must be volatile */
static volatile bool g_startRequested = false;
static volatile bool g_stopRequested = false;
static volatile bool g_calibrationPending = false;
static bool g_calibrationActive = false;
static uint32_t g_calibrationStartTick = 0;

/* Advanced fault detection state */
static float g_prevAvgCurrent = 0;          /* Previous cycle avg current for di/dt */
static uint32_t g_prevCurrentTick = 0;      /* Timestamp of previous reading */
static uint32_t g_supplyFaultStart = 0;     /* When supply sag/swell first detected */
static float g_ratedVoltageRef = 230.0f;    /* Reference voltage for sag/swell calc */

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
    (void)FlashConfig_Get();
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Compute HP from 3-phase power measurement */
    float sqrt3 = 1.732f;
    float watts = sqrt3 * snap.avgVoltage * snap.avgCurrent * snap.avgPF;
    float hp = watts / 746.0f;

    /* Clamp to supported range: 7HP - 25HP */
    if (hp < 7.0f) hp = 7.0f;
    if (hp > 25.0f) hp = 25.0f;

    /* Store calibration */
    CalibrationData_t cal;
    cal.ratedHP      = hp;
    cal.ratedVoltage  = snap.avgVoltage;
    cal.ratedCurrent  = snap.avgCurrent;
    cal.phases        = 3;
    cal.calibratedAt  = HAL_GetTick() / 1000;
    cal.isCalibrated  = true;
    FlashConfig_SetCalibration(&cal);

    /* Auto-set protection thresholds with 10-20% adaptive bands:
     *   Upper trip  = rated * (1 + THRESH_UPPER_BAND) = +20%
     *   Lower alarm = rated * (1 - THRESH_LOWER_BAND) = -10%
     *   Voltage: +15% over / -15% under from measured line voltage */
    ProtectionConfig_t prot;
    prot.overVoltage  = snap.avgVoltage * SUPPLY_SWELL_THRESHOLD;  /* +15% of actual */
    prot.underVoltage = snap.avgVoltage * SUPPLY_SAG_THRESHOLD;    /* -15% of actual */
    prot.overloadAmps = snap.avgCurrent * (1.0f + THRESH_UPPER_BAND);  /* +20% */
    prot.dryRunSec    = DEFAULT_DRY_RUN_SEC;
    prot.restartDelay = DEFAULT_RESTART_DELAY;
    FlashConfig_SetProtection(&prot);

    /* Store reference voltage for supply analysis */
    g_ratedVoltageRef = snap.avgVoltage;

    /* Auto star-delta timing: base + per-HP scaling */
    float sdDelay = SD_DELAY_BASE + (hp * SD_DELAY_PER_HP);
    if (sdDelay > SD_DELAY_MAX) sdDelay = SD_DELAY_MAX;
    FlashConfig_SetStarDeltaDelay(sdDelay);

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
            g_dryRunStartTick = HAL_GetTick();
        }
        if ((HAL_GetTick() - g_dryRunStartTick) < (cfg->protection.dryRunSec * 1000)) {
            return FAULT_NONE;
        }
        return FAULT_DRY_RUN;
    } else {
        g_dryRunStartTick = 0;
    }

    if (iFault != FAULT_NONE) return iFault;

    /* === ADVANCED FAULT DETECTION (only when motor running in delta) === */
    if (g_state != MOTOR_DELTA) return FAULT_NONE;

    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);
    float avgI = snap.avgCurrent;
    float avgV = snap.avgVoltage;
    uint32_t now = HAL_GetTick();

    /* Use calibrated voltage as reference for supply analysis */
    if (cfg->calibration.isCalibrated && cfg->calibration.ratedVoltage > 100.0f) {
        g_ratedVoltageRef = cfg->calibration.ratedVoltage;
    }

    /* --- Sand Jam / Stall detection ---
     * Very high current (>250% rated) sustained = mechanical jam */
    if (cfg->calibration.isCalibrated && avgI > cfg->calibration.ratedCurrent * STALL_CURRENT_MULT) {
        return FAULT_SAND_JAM;
    }

    /* --- Sudden Load Rise (di/dt > +50% in <2s) ---
     * Belt seized, foreign object, pump blockage */
    if (g_prevCurrentTick > 0 && cfg->calibration.isCalibrated &&
        (now - g_prevCurrentTick) < LOAD_CHANGE_WINDOW_MS && g_prevAvgCurrent > 0.5f) {
        float rise = (avgI - g_prevAvgCurrent) / g_prevAvgCurrent;
        if (rise > LOAD_RISE_THRESHOLD) {
            return FAULT_SUDDEN_LOAD_RISE;
        }
        /* --- Sudden Load Drop (di/dt < -40% in <2s) ---
         * Belt break, coupling failure, pipe burst */
        if (rise < -LOAD_DROP_THRESHOLD) {
            return FAULT_SUDDEN_LOAD_DROP;
        }
    }

    /* --- Supply Sag/Swell (sustained >3s) ---
     * Input voltage problem, grid instability */
    float vRatio = avgV / g_ratedVoltageRef;
    if (vRatio < SUPPLY_SAG_THRESHOLD || vRatio > SUPPLY_SWELL_THRESHOLD) {
        if (g_supplyFaultStart == 0) g_supplyFaultStart = now;
        if ((now - g_supplyFaultStart) > SUPPLY_FAULT_HOLD_MS) {
            return FAULT_SUPPLY_FAULT;
        }
    } else {
        g_supplyFaultStart = 0;
    }

    /* --- Ground Leak (residual current imbalance) ---
     * Sum of 3 phase currents should be ~0. If not, current leaking to ground.
     * Note: Software estimation — real ELCB/RCD uses dedicated CT on neutral. */
    float sumI = snap.r.current + snap.y.current + snap.b.current;
    float residual = fabsf(sumI - 3.0f * avgI);  /* Deviation from balanced */
    if (avgI > 1.0f && residual > (avgI * GROUND_LEAK_THRESHOLD * 3.0f)) {
        return FAULT_GROUND_LEAK;
    }

    /* Update previous reading for next cycle di/dt */
    g_prevAvgCurrent = avgI;
    g_prevCurrentTick = now;

    return FAULT_NONE;
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
/* M9 FIX: Motor is running in both STAR and DELTA states */
bool MotorControl_IsRunning(void)          { return (g_state == MOTOR_STAR || g_state == MOTOR_DELTA); }
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
        case FAULT_MANUAL_STOP:       return "Manual";
        case FAULT_REMOTE_STOP:       return "Remote";
        case FAULT_SAND_JAM:          return "JAMMED!";
        case FAULT_SUDDEN_LOAD_RISE:  return "LOAD SPIKE";
        case FAULT_SUDDEN_LOAD_DROP:  return "LOAD DROP";
        case FAULT_SUPPLY_FAULT:      return "SUPPLY BAD";
        case FAULT_GROUND_LEAK:       return "GND LEAK!";
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
    /* M8 FIX: Separate debounce timers for start and stop buttons.
     * Prevents emergency stop being ignored if start was just pressed. */
    static uint32_t lastStartPress = 0;
    static uint32_t lastStopPress = 0;
    uint32_t now = HAL_GetTick();

    if (isStart) {
        if ((now - lastStartPress) < BUTTON_DEBOUNCE_MS) return;
        lastStartPress = now;
        MotorControl_Start();
    } else {
        if ((now - lastStopPress) < BUTTON_DEBOUNCE_MS) return;
        lastStopPress = now;
        MotorControl_Stop(FAULT_MANUAL_STOP);
    }
}
