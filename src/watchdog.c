/**
 * @file watchdog.c
 * @brief Independent Watchdog (IWDG) - resets MCU if not fed within 4 seconds
 *
 * The IWDG uses the 40kHz LSI oscillator and runs independently of the
 * main system clock. This ensures the MCU recovers from hangs even if
 * the HSE fails.
 *
 * IWDG timeout = (Prescaler * Reload) / LSI_freq
 * For 4s: prescaler=64, reload=2500 -> (64 * 2500) / 40000 = 4.0s
 */

#include "watchdog.h"

static IWDG_HandleTypeDef hiwdg;
static bool g_wasWdgReset = false;

void Watchdog_Init(void)
{
    /* Check if this boot was caused by watchdog reset */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        g_wasWdgReset = true;
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    /* Configure IWDG for ~4 second timeout */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload    = 2500;  // (64 * 2500) / 40000 = 4.0s

    HAL_IWDG_Init(&hiwdg);
}

void Watchdog_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

bool Watchdog_WasReset(void)
{
    return g_wasWdgReset;
}
