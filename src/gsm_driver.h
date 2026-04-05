/**
 * @file gsm_driver.h
 * @brief SIM7670C 4G GSM AT Command Driver (State Machine)
 *
 * Provides HTTP GET/POST/PUT/PATCH operations for Firebase REST API.
 * Thread-safe via FreeRTOS mutex. Implements exponential backoff on errors.
 */

#ifndef GSM_DRIVER_H
#define GSM_DRIVER_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GSM_STATE_INIT,
    GSM_STATE_READY,
    GSM_STATE_HTTP_ACTIVE,
    GSM_STATE_ERROR,
    GSM_STATE_RESETTING,
} GsmState_t;

typedef enum {
    HTTP_GET    = 0,
    HTTP_POST   = 1,
    HTTP_PATCH  = 3,    // Note: SIM7670 may not support PATCH natively
    HTTP_PUT    = 4,
} HttpMethod_t;

/**
 * @brief Initialize GSM module (USART3, GPIO)
 * @return true if module responds to AT
 */
bool GSM_Init(void);

/**
 * @brief Get current GSM state
 */
GsmState_t GSM_GetState(void);

/**
 * @brief Check if GSM is ready for HTTP operations
 */
bool GSM_IsReady(void);

/**
 * @brief Get network time from GSM (AT+CCLK?)
 * @param buf Buffer for time string (min 32 bytes)
 * @return true if time retrieved
 */
bool GSM_GetNetworkTime(char* buf, uint16_t bufLen);

/**
 * @brief Get signal quality
 * @return RSSI value (0-31, 99=unknown)
 */
uint8_t GSM_GetSignalQuality(void);

/**
 * @brief Perform HTTP request to Firebase
 * @param method GET, POST, PUT, or PATCH
 * @param path Firebase path (e.g., "/users/abc/motors/MOT-001-A1")
 * @param jsonBody JSON body for POST/PUT/PATCH (NULL for GET)
 * @param response Buffer for response body
 * @param respLen Max response buffer length
 * @return HTTP status code (200=OK), 0 on failure
 */
uint16_t GSM_HttpRequest(HttpMethod_t method, const char* path,
                          const char* jsonBody,
                          char* response, uint16_t respLen);

/**
 * @brief Reset GSM module (called on persistent errors)
 */
void GSM_Reset(void);

/**
 * @brief Run GSM state machine (call periodically to handle recovery)
 */
void GSM_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* GSM_DRIVER_H */
