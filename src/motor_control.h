/**
 * @file motor_control.h
 * @brief Star-delta motor control FSM with auto-calibration and operating modes
 *
 * Features:
 * - Star-delta starting with configurable timing
 * - Auto-calibration: detects motor HP from delta current
 * - Operating modes: Normal, Safe Mode, Force Run
 * - Protection: overvoltage, undervoltage, overload, dry run, phase loss
 * - Runtime tracking (hours, kWh, start count)
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize motor control (GPIO, relays, buttons)
 */
void MotorControl_Init(void);

/**
 * @brief Run the motor FSM (call every MOTOR_CHECK_PERIOD_MS)
 *
 * Handles state transitions, relay timing, protection checks.
 */
void MotorControl_Update(void);

/**
 * @brief Request motor start
 * @param source CMD source for logging
 */
void MotorControl_Start(void);

/**
 * @brief Request motor stop
 * @param fault Fault code (FAULT_NONE for normal stop)
 */
void MotorControl_Stop(FaultCode_t fault);

/**
 * @brief Get current motor state
 * @return MotorState_t
 */
MotorState_t MotorControl_GetState(void);

/**
 * @brief Check if motor is running (in DELTA state)
 * @return true if running
 */
bool MotorControl_IsRunning(void);

/**
 * @brief Get last fault code
 * @return FaultCode_t
 */
FaultCode_t MotorControl_GetFault(void);

/**
 * @brief Get fault description string
 * @param fault Fault code
 * @return Static string describing the fault
 */
const char* MotorControl_FaultString(FaultCode_t fault);

/**
 * @brief Set safe mode (tighter thresholds)
 * @param enable true to enable
 */
void MotorControl_SetSafeMode(bool enable);

/**
 * @brief Set force run mode (bypass dry-run + overload)
 * @param enable true to enable
 */
void MotorControl_SetForceRun(bool enable);

/**
 * @brief Get current safe mode state
 */
bool MotorControl_GetSafeMode(void);

/**
 * @brief Get current force run state
 */
bool MotorControl_GetForceRun(void);

/**
 * @brief Trigger auto-calibration on next start
 *
 * Will measure delta current after transition and compute:
 * - Rated HP, rated current
 * - Protection thresholds
 * - Star-delta timing
 */
void MotorControl_RequestCalibration(void);

/**
 * @brief Check if calibration is pending
 */
bool MotorControl_IsCalibrationPending(void);

/**
 * @brief Get motor runtime stats
 * @param stats Destination
 */
void MotorControl_GetStats(MotorStats_t* stats);

/**
 * @brief Handle button press (called from EXTI ISR via deferred processing)
 * @param isStart true = start button, false = stop button
 */
void MotorControl_ButtonPress(bool isStart);

/**
 * @brief Turn off all relay outputs (safety function)
 * Called from stack overflow hook and fault handlers.
 */
void AllRelaysOff(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
