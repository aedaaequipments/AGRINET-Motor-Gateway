/**
 * @file display.h
 * @brief SSD1306 OLED Display - multi-page rendering (128x64, I2C1)
 *
 * 5 display pages cycled every 3 seconds:
 * 1. Power: V/I/PF per phase + total kW
 * 2. Motor: state, HP, star-delta, mode, fault
 * 3. Weather: soil moisture L1/L2/L3, temp, humidity
 * 4. Valves: open/close status, flow rate
 * 5. Network: GSM signal, LoRa nodes, Firebase status
 *
 * Uses page-mode rendering (no full framebuffer) to save 1KB RAM.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PAGE_POWER = 0,
    PAGE_MOTOR,
    PAGE_WEATHER,
    PAGE_VALVES,
    PAGE_NETWORK,
    PAGE_COUNT
} DisplayPage_t;

/**
 * @brief Initialize SSD1306 OLED via I2C1
 * @return true if OLED detected
 */
bool Display_Init(void);

/**
 * @brief Render current page (call every DISPLAY_UPDATE_MS)
 */
void Display_Update(void);

/**
 * @brief Switch to next page
 */
void Display_NextPage(void);

/**
 * @brief Set specific page
 */
void Display_SetPage(DisplayPage_t page);

/**
 * @brief Get current page
 */
DisplayPage_t Display_GetPage(void);

/**
 * @brief Show boot splash screen
 */
void Display_Splash(void);

/**
 * @brief Show error message (overrides current page)
 * @param line1 First line text
 * @param line2 Second line text
 */
void Display_Error(const char* line1, const char* line2);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
