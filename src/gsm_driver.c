/**
 * @file gsm_driver.c
 * @brief SIM7670C AT Command State Machine for Firebase HTTP REST API
 *
 * Rewrites the prototype's simple sendATCommand() into a proper state machine
 * with exponential backoff, mutex protection, and error recovery.
 * Uses char arrays (no String class) for memory safety.
 */

#include "gsm_driver.h"
#include "flash_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static UART_HandleTypeDef s_huart3;
static SemaphoreHandle_t  s_gsmMutex;
static StaticSemaphore_t  s_gsmMutexBuf;

static GsmState_t s_state = GSM_STATE_INIT;
static uint8_t    s_errorCount = 0;
static uint32_t   s_lastResetTick = 0;
static uint8_t    s_signalQuality = 99;

/* Buffers */
static char s_rxBuf[GSM_RX_BUF_SIZE];
static char s_txBuf[GSM_TX_BUF_SIZE];
static char s_urlBuf[256];

/* ═══════════════════════════════════════════════════════════════════════════
 * UART HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void UART_Send(const char* str)
{
    HAL_UART_Transmit(&s_huart3, (uint8_t*)str, strlen(str), 1000);
}

static void UART_SendLine(const char* str)
{
    UART_Send(str);
    UART_Send("\r\n");
}

/**
 * Read UART response with adaptive timeout (resets on each byte received)
 */
static uint16_t UART_ReadResponse(uint32_t timeoutMs)
{
    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();
    uint8_t byte;

    memset(s_rxBuf, 0, GSM_RX_BUF_SIZE);

    while ((HAL_GetTick() - start) < timeoutMs) {
        if (HAL_UART_Receive(&s_huart3, &byte, 1, 10) == HAL_OK) {
            if (idx < GSM_RX_BUF_SIZE - 1) {
                s_rxBuf[idx++] = (char)byte;
            }
            start = HAL_GetTick();  // Reset timeout on each byte
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    s_rxBuf[idx] = '\0';
    return idx;
}

/**
 * Send AT command and wait for expected response
 */
static bool SendAT(const char* cmd, const char* expect, uint32_t timeoutMs)
{
    UART_SendLine(cmd);
    UART_ReadResponse(timeoutMs);
    return (strstr(s_rxBuf, expect) != NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT
 * ═══════════════════════════════════════════════════════════════════════════ */

bool GSM_Init(void)
{
    s_gsmMutex = xSemaphoreCreateMutexStatic(&s_gsmMutexBuf);

    /* Enable USART3 clock */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* TX (PB10) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_GSM_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PIN_GSM_TX_PORT, &gpio);

    /* RX (PB11) */
    gpio.Pin  = PIN_GSM_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PIN_GSM_RX_PORT, &gpio);

    /* Init USART3 */
    s_huart3.Instance          = USART3;
    s_huart3.Init.BaudRate     = 115200;
    s_huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart3.Init.StopBits     = UART_STOPBITS_1;
    s_huart3.Init.Parity       = UART_PARITY_NONE;
    s_huart3.Init.Mode         = UART_MODE_TX_RX;
    s_huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&s_huart3);

    /* Initialization sequence */
    vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for module boot

    if (!SendAT("AT", "OK", 2000)) return false;
    if (!SendAT("AT+CPIN?", "READY", 2000)) return false;

    /* Wait for network registration */
    for (int i = 0; i < 10; i++) {
        if (SendAT("AT+CREG?", "+CREG: 0,1", 2000) ||
            SendAT("AT+CREG?", "+CREG: 0,5", 2000)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    SendAT("AT+CGATT=1", "OK", 3000);

    /* Initialize HTTP service */
    SendAT("AT+HTTPTERM", "OK", 1000);  // Cleanup first
    if (!SendAT("AT+HTTPINIT", "OK", 2000)) return false;
    SendAT("AT+HTTPPARA=\"CID\",1", "OK", 1000);

    s_state = GSM_STATE_HTTP_ACTIVE;
    s_errorCount = 0;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE
 * ═══════════════════════════════════════════════════════════════════════════ */

GsmState_t GSM_GetState(void) { return s_state; }
bool GSM_IsReady(void) { return s_state == GSM_STATE_HTTP_ACTIVE; }

void GSM_Update(void)
{
    switch (s_state) {
    case GSM_STATE_ERROR:
        {
            /* Exponential backoff: 5s, 10s, 20s, 40s, max 60s */
            uint32_t backoff = 5000 * (1 << s_errorCount);
            if (backoff > 60000) backoff = 60000;

            if ((HAL_GetTick() - s_lastResetTick) > backoff) {
                s_state = GSM_STATE_RESETTING;
            }
        }
        break;

    case GSM_STATE_RESETTING:
        if (GSM_Init()) {
            s_state = GSM_STATE_HTTP_ACTIVE;
            s_errorCount = 0;
        } else {
            s_errorCount++;
            if (s_errorCount > 5) s_errorCount = 5;  // Cap at max backoff
            s_lastResetTick = HAL_GetTick();
            s_state = GSM_STATE_ERROR;
        }
        break;

    default:
        break;
    }
}

void GSM_Reset(void)
{
    s_lastResetTick = HAL_GetTick();
    s_state = GSM_STATE_RESETTING;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HTTP REQUEST
 * ═══════════════════════════════════════════════════════════════════════════ */

uint16_t GSM_HttpRequest(HttpMethod_t method, const char* path,
                          const char* jsonBody,
                          char* response, uint16_t respLen)
{
    if (s_state != GSM_STATE_HTTP_ACTIVE) return 0;
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return 0;

    FlashConfig_t* cfg = FlashConfig_Get();
    uint16_t httpStatus = 0;

    /* Build full URL: {baseUrl}/users/{uid}/{path}.json?auth={token} */
    snprintf(s_urlBuf, sizeof(s_urlBuf),
        "%s/users/%s/%s.json?auth=%s",
        cfg->firebaseUrl, cfg->firebaseUid, path, cfg->firebaseToken);

    /* Set URL */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+HTTPPARA=\"URL\",\"%s\"", s_urlBuf);
    if (!SendAT(s_txBuf, "OK", 2000)) goto fail;

    /* For POST/PUT/PATCH, set content type and send body */
    if (jsonBody && (method == HTTP_POST || method == HTTP_PUT || method == HTTP_PATCH)) {
        SendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 1000);

        uint16_t bodyLen = strlen(jsonBody);
        snprintf(s_txBuf, sizeof(s_txBuf), "AT+HTTPDATA=%u,10000", bodyLen);
        if (!SendAT(s_txBuf, "DOWNLOAD", 2000)) goto fail;

        /* Send JSON body */
        UART_Send(jsonBody);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    /* Execute HTTP action */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+HTTPACTION=%d", (int)method);
    UART_SendLine(s_txBuf);
    UART_ReadResponse(6000);

    /* Parse HTTP status from +HTTPACTION: <method>,<status>,<datalen> */
    {
        char* p = strstr(s_rxBuf, "+HTTPACTION:");
        if (p) {
            int m, s, d;
            if (sscanf(p, "+HTTPACTION: %d,%d,%d", &m, &s, &d) >= 2) {
                httpStatus = (uint16_t)s;
            }
        }
    }

    /* Read response body */
    if (response && respLen > 0 && httpStatus >= 200 && httpStatus < 300) {
        SendAT("AT+HTTPREAD=0,500", "+HTTPREAD:", 3000);

        /* Extract body after first newline following +HTTPREAD: */
        char* p = strstr(s_rxBuf, "+HTTPREAD:");
        if (p) {
            char* nl = strchr(p, '\n');
            if (nl) {
                nl++;  // Skip newline
                /* Copy until next +HTTPREAD: or end */
                char* end = strstr(nl, "\r\nOK");
                uint16_t copyLen;
                if (end) {
                    copyLen = (uint16_t)(end - nl);
                } else {
                    copyLen = strlen(nl);
                }
                if (copyLen >= respLen) copyLen = respLen - 1;
                memcpy(response, nl, copyLen);
                response[copyLen] = '\0';

                /* Trim whitespace */
                while (copyLen > 0 && (response[copyLen-1] == '\r' ||
                       response[copyLen-1] == '\n' || response[copyLen-1] == ' ')) {
                    response[--copyLen] = '\0';
                }
            }
        }
    }

    s_errorCount = 0;
    xSemaphoreGive(s_gsmMutex);
    return httpStatus;

fail:
    s_errorCount++;
    if (s_errorCount >= 3) {
        s_state = GSM_STATE_ERROR;
        s_lastResetTick = HAL_GetTick();
    }
    xSemaphoreGive(s_gsmMutex);
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UTILITY
 * ═══════════════════════════════════════════════════════════════════════════ */

bool GSM_GetNetworkTime(char* buf, uint16_t bufLen)
{
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;

    if (SendAT("AT+CCLK?", "+CCLK:", 2000)) {
        char* p = strstr(s_rxBuf, "+CCLK: \"");
        if (p) {
            p += 8;
            char* q = strchr(p, '"');
            if (q) {
                uint16_t len = (uint16_t)(q - p);
                if (len >= bufLen) len = bufLen - 1;
                memcpy(buf, p, len);
                buf[len] = '\0';
                xSemaphoreGive(s_gsmMutex);
                return true;
            }
        }
    }

    xSemaphoreGive(s_gsmMutex);
    return false;
}

uint8_t GSM_GetSignalQuality(void)
{
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return 99;

    if (SendAT("AT+CSQ", "+CSQ:", 2000)) {
        char* p = strstr(s_rxBuf, "+CSQ: ");
        if (p) {
            int rssi;
            if (sscanf(p, "+CSQ: %d", &rssi) == 1) {
                s_signalQuality = (uint8_t)rssi;
            }
        }
    }

    xSemaphoreGive(s_gsmMutex);
    return s_signalQuality;
}
