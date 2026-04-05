/**
 * @file watchdog.h
 * @brief Independent Watchdog (IWDG) for system recovery
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize IWDG with configured timeout
 */
void Watchdog_Init(void);

/**
 * @brief Feed (refresh) the watchdog -- call regularly from each task
 */
void Watchdog_Feed(void);

/**
 * @brief Check if last reset was caused by watchdog
 * @return true if IWDG caused the reset
 */
bool Watchdog_WasReset(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
