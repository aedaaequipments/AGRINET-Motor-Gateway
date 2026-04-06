/**
 * @file gsm_driver.c
 * @brief SIM7670C AT Command Driver — MQTT Transport (C1/C2 fix)
 *
 * Replaces Firebase HTTP REST with AT+CMQTT commands for Mosquitto broker.
 * Uses char arrays (no String class) for memory safety.
 * Thread-safe via FreeRTOS mutex with exponential backoff on errors.
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

/* MQTT incoming message callback */
static MqttMessageCallback s_mqttCallback = NULL;
static char s_incomingTopic[128];
static char s_incomingPayload[JSON_BUF_SIZE];

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

/**
 * Send raw data (no \r\n) — used for MQTT topic/payload data input
 */
static void UART_SendRaw(const char* data, uint16_t len)
{
    HAL_UART_Transmit(&s_huart3, (uint8_t*)data, len, 2000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GSM INIT (network registration only — MQTT init is separate)
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

    s_state = GSM_STATE_NETWORK_READY;
    s_errorCount = 0;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE MACHINE
 * ═══════════════════════════════════════════════════════════════════════════ */

GsmState_t GSM_GetState(void) { return s_state; }
bool GSM_IsReady(void) { return s_state == GSM_STATE_MQTT_CONNECTED; }

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
            /* Network ready — caller (CloudSync) will re-establish MQTT */
            s_state = GSM_STATE_NETWORK_READY;
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
    /* Cleanup MQTT service before reset */
    SendAT("AT+CMQTTDISC=0,60", "OK", 2000);
    SendAT("AT+CMQTTREL=0", "OK", 1000);
    SendAT("AT+CMQTTSTOP", "OK", 1000);

    s_lastResetTick = HAL_GetTick();
    s_state = GSM_STATE_RESETTING;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MQTT OPERATIONS (AT+CMQTT — SIM7670C)
 * ═══════════════════════════════════════════════════════════════════════════ */

void GSM_MqttSetCallback(MqttMessageCallback cb)
{
    s_mqttCallback = cb;
}

bool GSM_MqttConnect(const char* brokerIp, uint16_t port,
                     const char* clientId,
                     const char* username, const char* password)
{
    if (s_state != GSM_STATE_NETWORK_READY && s_state != GSM_STATE_MQTT_CONNECTING) {
        return false;
    }
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(10000)) != pdTRUE) return false;

    s_state = GSM_STATE_MQTT_CONNECTING;

    /* Cleanup any previous MQTT session */
    SendAT("AT+CMQTTDISC=0,60", "OK", 2000);
    SendAT("AT+CMQTTREL=0", "OK", 1000);
    SendAT("AT+CMQTTSTOP", "OK", 1000);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Start MQTT service */
    if (!SendAT("AT+CMQTTSTART", "OK", 3000)) {
        /* Check if already started */
        if (strstr(s_rxBuf, "+CMQTTSTART: 0") == NULL) {
            goto mqtt_fail;
        }
    }

    /* Acquire MQTT client (index 0, TCP mode) */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+CMQTTACCQ=0,\"%s\",0", clientId);
    if (!SendAT(s_txBuf, "OK", 3000)) goto mqtt_fail;

    /* Connect to broker */
    if (username && username[0] != '\0') {
        snprintf(s_txBuf, sizeof(s_txBuf),
            "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1,\"%s\",\"%s\"",
            brokerIp, port, username, password ? password : "");
    } else {
        snprintf(s_txBuf, sizeof(s_txBuf),
            "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1",
            brokerIp, port);
    }

    UART_SendLine(s_txBuf);
    UART_ReadResponse(10000);  // Connection can take time

    /* Check for +CMQTTCONNECT: 0,0 (success) */
    if (strstr(s_rxBuf, "+CMQTTCONNECT: 0,0") == NULL) {
        goto mqtt_fail;
    }

    s_state = GSM_STATE_MQTT_CONNECTED;
    s_errorCount = 0;
    xSemaphoreGive(s_gsmMutex);
    return true;

mqtt_fail:
    s_errorCount++;
    if (s_errorCount >= 3) {
        s_state = GSM_STATE_ERROR;
        s_lastResetTick = HAL_GetTick();
    } else {
        s_state = GSM_STATE_NETWORK_READY;
    }
    xSemaphoreGive(s_gsmMutex);
    return false;
}

bool GSM_MqttPublish(const char* topic, const char* payload, uint8_t qos)
{
    if (s_state != GSM_STATE_MQTT_CONNECTED) return false;
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return false;

    uint16_t topicLen = strlen(topic);
    uint16_t payloadLen = strlen(payload);
    bool ok = false;

    /* Step 1: Set topic */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+CMQTTTOPIC=0,%u", topicLen);
    UART_SendLine(s_txBuf);
    UART_ReadResponse(2000);
    if (strstr(s_rxBuf, ">") == NULL) goto pub_done;

    /* Send topic data */
    UART_SendRaw(topic, topicLen);
    UART_ReadResponse(2000);
    if (strstr(s_rxBuf, "OK") == NULL) goto pub_done;

    /* Step 2: Set payload */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+CMQTTPAYLOAD=0,%u", payloadLen);
    UART_SendLine(s_txBuf);
    UART_ReadResponse(2000);
    if (strstr(s_rxBuf, ">") == NULL) goto pub_done;

    /* Send payload data */
    UART_SendRaw(payload, payloadLen);
    UART_ReadResponse(2000);
    if (strstr(s_rxBuf, "OK") == NULL) goto pub_done;

    /* Step 3: Publish */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+CMQTTPUB=0,%u,60", qos);
    UART_SendLine(s_txBuf);
    UART_ReadResponse(5000);

    /* +CMQTTPUB: 0,0 = success */
    ok = (strstr(s_rxBuf, "+CMQTTPUB: 0,0") != NULL);

pub_done:
    if (!ok) {
        s_errorCount++;
        if (s_errorCount >= 3) {
            s_state = GSM_STATE_ERROR;
            s_lastResetTick = HAL_GetTick();
        }
    } else {
        s_errorCount = 0;
    }

    xSemaphoreGive(s_gsmMutex);
    return ok;
}

bool GSM_MqttSubscribe(const char* topic, uint8_t qos)
{
    if (s_state != GSM_STATE_MQTT_CONNECTED) return false;
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return false;

    uint16_t topicLen = strlen(topic);
    bool ok = false;

    /* AT+CMQTTSUB=0,<topic_len>,<qos> then send topic data */
    snprintf(s_txBuf, sizeof(s_txBuf), "AT+CMQTTSUB=0,%u,%u", topicLen, qos);
    UART_SendLine(s_txBuf);
    UART_ReadResponse(2000);
    if (strstr(s_rxBuf, ">") == NULL) goto sub_done;

    /* Send topic string */
    UART_SendRaw(topic, topicLen);
    UART_ReadResponse(5000);

    /* +CMQTTSUB: 0,0 = success */
    ok = (strstr(s_rxBuf, "+CMQTTSUB: 0,0") != NULL);

sub_done:
    xSemaphoreGive(s_gsmMutex);
    return ok;
}

void GSM_MqttProcessIncoming(void)
{
    if (s_state != GSM_STATE_MQTT_CONNECTED) return;
    if (!s_mqttCallback) return;
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    /* Non-blocking check for incoming data */
    uint16_t len = UART_ReadResponse(50);

    if (len > 0 && strstr(s_rxBuf, "+CMQTTRXSTART:") != NULL) {
        /* Parse: +CMQTTRXSTART: 0,<topic_len>,<payload_len> */
        uint16_t topicLen = 0, payloadLen = 0;
        char* p = strstr(s_rxBuf, "+CMQTTRXSTART:");
        if (p) {
            int cl, tl, pl;
            sscanf(p, "+CMQTTRXSTART: %d,%hu,%hu", &cl, &topicLen, &payloadLen);
        }

        /* Extract topic from +CMQTTRXTOPIC: ... */
        p = strstr(s_rxBuf, "+CMQTTRXTOPIC:");
        if (p) {
            /* Skip past the header line to get the topic data */
            char* nl = strchr(p, '\n');
            if (nl) {
                nl++;
                uint16_t copyLen = topicLen;
                if (copyLen >= sizeof(s_incomingTopic)) copyLen = sizeof(s_incomingTopic) - 1;
                memcpy(s_incomingTopic, nl, copyLen);
                s_incomingTopic[copyLen] = '\0';
            }
        }

        /* Extract payload from +CMQTTRXPAYLOAD: ... */
        p = strstr(s_rxBuf, "+CMQTTRXPAYLOAD:");
        if (p) {
            char* nl = strchr(p, '\n');
            if (nl) {
                nl++;
                uint16_t copyLen = payloadLen;
                if (copyLen >= sizeof(s_incomingPayload)) copyLen = sizeof(s_incomingPayload) - 1;
                memcpy(s_incomingPayload, nl, copyLen);
                s_incomingPayload[copyLen] = '\0';
            }
        }

        /* Invoke callback if we got both topic and payload */
        if (s_incomingTopic[0] != '\0') {
            xSemaphoreGive(s_gsmMutex);
            s_mqttCallback(s_incomingTopic, s_incomingPayload, strlen(s_incomingPayload));
            return;
        }
    }

    xSemaphoreGive(s_gsmMutex);
}

void GSM_MqttDisconnect(void)
{
    if (xSemaphoreTake(s_gsmMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return;

    SendAT("AT+CMQTTDISC=0,60", "OK", 3000);
    SendAT("AT+CMQTTREL=0", "OK", 1000);
    SendAT("AT+CMQTTSTOP", "OK", 1000);

    s_state = GSM_STATE_NETWORK_READY;

    xSemaphoreGive(s_gsmMutex);
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
