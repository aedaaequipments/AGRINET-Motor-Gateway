/**
 * @file watchdog.c
 * @brief Independent Watchdog (IWDG) - resets MCU if not fed within 8 seconds
 *
 * The IWDG uses the 40kHz LSI oscillator and runs independently of the
 * main system clock. This ensures the MCU recovers from hangs even if
 * the HSE fails.
 *
 * IWDG timeout = (Prescaler * Reload) / LSI_freq
 * For 8s: prescaler=64, reload=5000 -> (64 * 5000) / 40000 = 8.0s
 *
 * C2 FIX: Increased from 4s to 8s. LoRa TX at SF12/BW125 takes up to 5s
 * airtime, so 4s was too tight and caused spurious watchdog resets during
 * legitimate radio transmissions.
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

    /* Configure IWDG for ~8 second timeout (C2 fix: was 4s, too short for SF12 TX) */
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload    = 5000;  // (64 * 5000) / 40000 = 8.0s

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
